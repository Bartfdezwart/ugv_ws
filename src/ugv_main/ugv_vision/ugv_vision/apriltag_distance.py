#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import CameraInfo
from ugv_interface.msg import AprilTagArray

class TagTriangulator(Node):
    def __init__(self):
        super().__init__("apriltag_localization")

        self.K_received = False
        self.scale = 2

        # Your tag 3D coordinates (IDENTICAL to your main file)
        tw = 0.158
        self.tag_points_3d = np.array([
            [-tw/2,  tw/2, 0],
            [ tw/2,  tw/2, 0],
            [ tw/2, -tw/2, 0],
            [-tw/2, -tw/2, 0]
        ], dtype=np.float32)

        # Tag world positions (COPY EXACTLY AS IN YOUR SCRIPT)
        self.tag_world_positions = {
            4: np.array([-3.0, 0.0]),
            5: np.array([3.0, 0.0]),
            6: np.array([-3.2, 1.85]),
            7: np.array([-3.66, 1.85]),
            8: np.array([-3.0, 4.5]),
            9: np.array([0.0, 4.5]),
            10: np.array([3.0, 4.5]),
        }

        # Subscribers
        self.tag_sub = self.create_subscription(
            AprilTagArray, "/apriltags", self.tags_callback, 10
        )
        self.camera_sub = self.create_subscription(
            CameraInfo, "/image_rect/camera_info", self.camera_callback, 10
        )

        self.get_logger().info("AprilTag triangulation listener started.")

    # ----------------------------------------------------------------------
    # Load camera intrinsics (IDENTICAL scaling to your script)
    # ----------------------------------------------------------------------
    def camera_callback(self, msg: CameraInfo):
        if self.K_received:
            return

        K = np.array(msg.k, dtype=float).reshape(3,3)
        K_scaled = K.copy()
        K_scaled[0,0] *= self.scale
        K_scaled[1,1] *= self.scale
        K_scaled[0,2] *= self.scale
        K_scaled[1,2] *= self.scale

        self.K = K_scaled
        self.K_received = True
        self.get_logger().info("Camera intrinsics received in triangulation node.")

    # ----------------------------------------------------------------------
    # AprilTag distance + triangulation processing
    # ----------------------------------------------------------------------
    def tags_callback(self, msg: AprilTagArray):
        if not self.K_received:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        if not msg.detections:
            return

        visible_ids = []
        distances = []

        # ------------------------------------------------------------------
        # EXACT distance computation using solvePnP (same as your main file)
        # ------------------------------------------------------------------
        for det in msg.detections:
            tid = det.id

            corners = np.array([[c.x, c.y] for c in det.corners], dtype=np.float32)

            # solvePnP to compute distance
            _, _, tvec = cv2.solvePnP(
                self.tag_points_3d,
                corners,
                self.K,
                None,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            distance = float(tvec[2][0])

            visible_ids.append(tid)
            distances.append(distance)

        # ------------------------------------------------------------------
        # Triangulate position (EXACT copy of your svd_position function)
        # ------------------------------------------------------------------
        xy = self.svd_position(visible_ids, distances)

        if xy is not None:
            self.get_logger().info(f"ROVER POSITION → x={xy[0]:.2f}, y={xy[1]:.2f}")

    # ----------------------------------------------------------------------
    # EXACT COPY of your svd_position (1-to-1)
    # ----------------------------------------------------------------------
    def svd_position(self, tag_ids, distances):

        Ps, Ds = [], []
        for tid, d in zip(tag_ids, distances):
            if tid in self.tag_world_positions:
                Ps.append(self.tag_world_positions[tid])
                Ds.append(d)

        Ps = np.array(Ps, float)
        Ds = np.array(Ds, float)
        n = len(Ps)

        if n < 2:
            return None

        if n == 2:
            P1, P2 = Ps
            d1, d2 = Ds
            D = np.linalg.norm(P2 - P1)
            if D > d1 + d2 or D < abs(d1 - d2):
                return None

            a = (d1*d1 - d2*d2 + D*D) / (2*D)
            h2 = d1*d1 - a*a
            if h2 < 0:
                return None

            P3 = P1 + a * (P2 - P1) / D
            perp = np.array([-(P2[1]-P1[1])/D, (P2[0]-P1[0])/D]) * np.sqrt(h2)

            sol1, sol2 = P3 + perp, P3 - perp

            inside = lambda p: -4.5 <= p[0] <= 4.5 and -3 <= p[1] <= 3
            return sol1 if inside(sol1) else sol2

        # 3+ tags (SVD)
        A = np.column_stack((2*Ps[:,0], 2*Ps[:,1], -np.ones(n)))
        b = (Ps[:,0]**2 + Ps[:,1]**2 - Ds**2).reshape(-1,1)

        U, S, Vt = np.linalg.svd(A, full_matrices=False)
        x = Vt.T @ (np.linalg.inv(np.diag(S)) @ (U.T @ b))
        return x[:2, 0]


def main(args=None):
    rclpy.init(args=args)
    node = TagTriangulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()