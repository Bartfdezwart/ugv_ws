import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from ugv_interface.msg import AprilTag, AprilTagArray, Position
from scipy.optimize import minimize
from geometry_msgs.msg import Point, Pose, PoseStamped


class ApriltagDistance(Node):
    def __init__(self, visualize: bool = False):
        super().__init__('apriltag_distance')
        self.visualize = visualize

        self.tag_sub = self.create_subscription(AprilTagArray, '/apriltags', self.tag_callback, 10)
        self.tags_distance_pub = self.create_publisher(AprilTagArray, '/apriltags_distance', 10)
        self.position_pub = self.create_publisher(Position, '/rover_position', 10)
        self.robot_pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10,)

        # Camera intrinsics
        self.tw = 0.160
        self.K_received = False
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera_info", self.camera_info_callback, 10)

        # AprilTag 3D corner layout
        self.tag_points_3d = np.array([
            [-self.tw/2,  self.tw/2, 0],
            [ self.tw/2,  self.tw/2, 0],
            [ self.tw/2, -self.tw/2, 0],
            [-self.tw/2, -self.tw/2, 0]
        ], dtype=np.float32)

        self.scale = 2
        self.K = []

        # Tag world coordinates
        self.tag_world_positions = {
            1: np.array([0.0, -4.5]),
            2: np.array([-3.0, -2.85]),
            3: np.array([3.0, -2.85]),
            4: np.array([-3.0, 0.0]),
            5: np.array([3.0, 0.0]),

            6: np.array([-3.2, 1.85]),
            7: np.array([3.66, 1.85]),

            8: np.array([-2.84, 4.5]),
            9: np.array([0.0, 4.5]),
            10: np.array([2.84, 4.5]),
        }

    def camera_info_callback(self, msg: CameraInfo):
        if self.K_received:
            return
        print("CAMERA CALLBACK")
        K = np.array(msg.k, dtype=float).reshape(3, 3)

        K_scaled = K.copy()
        K_scaled[0, 0] *= self.scale
        K_scaled[1, 1] *= self.scale
        K_scaled[0, 2] *= self.scale
        K_scaled[1, 2] *= self.scale

        self.K = K_scaled
        self.get_logger().info("Camera intrinsics stored.")
        self.K_received = True


    def tag_callback(self, msg: AprilTagArray):

        if not self.K_received:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        visible_ids = []
        distances = []

        out_msg = AprilTagArray()
        out_msg.header = msg.header
        out_msg.detections = []

        for det in msg.detections:

            corners = np.array([[p.x, p.y] for p in det.corners], dtype=np.float32)

            _, _, tvec = cv2.solvePnP(self.tag_points_3d,corners, self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)

            distance = float(np.linalg.norm(tvec))

            det_out = AprilTag()
            det_out.family = det.family
            det_out.id = det.id
            det_out.hamming = det.hamming
            det_out.goodness = det.goodness
            det_out.decision_margin = det.decision_margin
            det_out.centre = det.centre
            det_out.corners = det.corners
            det_out.homography = det.homography
            det_out.distance = distance

            out_msg.detections.append(det_out)

            visible_ids.append(det.id)
            distances.append(distance)

        rover_xy = self.svd_position(visible_ids, distances)

        self.tags_distance_pub.publish(out_msg)

        if rover_xy is not None:
            pos = Position()
            pos.x = float(rover_xy[0])
            pos.y = float(rover_xy[1])
            self.position_pub.publish(pos)

            pose = PoseStamped(pose=Pose(position=Point(x=float(rover_xy[0]), y=float(rover_xy[1]))), header=msg.header)
            self.robot_pose_pub.publish(pose)

    def svd_position(self, tag_ids, distances):

        Ps, Ds = [], []
        for tid, d in zip(tag_ids, distances):
            if tid in self.tag_world_positions:
                Ps.append(self.tag_world_positions[tid])
                Ds.append(d)

        Ps = np.array(Ps, float)
        Ds = np.array(Ds, float)
        n = len(Ps)

        # Not enough valid tags
        if n < 2:
            return None

        # 2 tags
        if n == 2:
            P1, P2 = Ps
            d1, d2 = Ds
            D = np.linalg.norm(P2 - P1)

            if D > d1 + d2 or D < abs(d1 - d2):
                return None

            a = (d1**2 - d2**2 + D**2) / (2*D)
            h2 = d1**2 - a**2
            if h2 < 0:
                return None

            P3 = P1 + a * (P2 - P1) / D
            perp = np.array([-(P2[1]-P1[1])/D, (P2[0]-P1[0])/D]) * np.sqrt(h2)

            sol1, sol2 = P3 + perp, P3 - perp
            inside = lambda p: -3.0 <= p[0] <= 3.0 and -4.5 <= p[1] <= 4.5

            return sol1 if inside(sol1) else sol2

        # 3+ tags
        A = np.column_stack((2*Ps[:, 0], 2*Ps[:, 1], -np.ones(n)))
        b = (Ps[:,0]**2 + Ps[:,1]**2 - Ds**2).reshape(-1, 1)

        U, S, Vt = np.linalg.svd(A, full_matrices=False)
        x = Vt.T @ (np.linalg.inv(np.diag(S)) @ (U.T @ b))

        position = x[:2, 0]

        refined_position = self.refine_position(position, Ps, Ds)

        return refined_position

    def refine_position(
        self,
        initial_position,
        beacon_positions,
        distances,
        tolerance = 1e-6,
        max_iter = 20,
    ):
        def iterative_trilateration(position):
            distance_errors = np.abs((distances - np.linalg.norm(beacon_positions[:, :2] - position, axis=1)) / distances)
            return np.mean(distance_errors)

        minimization_result = minimize(
            iterative_trilateration,
            initial_position.flatten(),
            method="L-BFGS-B",
            tol=tolerance,
            bounds=(
                (-3,3),
                (-4.5, 4.5)
            ),
            options={
                "maxiter": max_iter
            }
        )
        if not minimization_result.success:
            self.get_logger().warning(f"Refined position in max iterators: {minimization_result.nit}")
        refined_position = minimization_result.x
        return refined_position


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()