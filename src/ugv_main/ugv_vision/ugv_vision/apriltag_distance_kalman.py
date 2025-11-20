import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from ugv_interface.msg import AprilTag, AprilTagArray, Position
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion


class ApriltagDistance(Node):
    def __init__(self, visualize: bool = False):
        super().__init__('apriltag_distance')
        self.visualize = visualize

        self.tag_sub = self.create_subscription(AprilTagArray, '/apriltags', self.tag_callback, 10)
        self.tags_distance_pub = self.create_publisher(AprilTagArray, '/apriltags_distance', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/rover_pose', 10)

        self.tw = 0.160
        self.K_received = False
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera_info", self.camera_info_callback, 10)

        self.tag_points_3d = np.array([
            [-self.tw/2,  self.tw/2, 0],
            [ self.tw/2,  self.tw/2, 0],
            [ self.tw/2, -self.tw/2, 0],
            [-self.tw/2, -self.tw/2, 0]
        ], dtype=np.float32)

        self.scale = 2
        self.K = []

        self.x_kf = np.zeros((4, 1), float)
        self.P_kf = np.eye(4, dtype=float)
        self.R_kf = 0.05 * np.eye(2, dtype=float)
        self.Q_kf_base = 0.01 * np.eye(4, dtype=float)
        self.last_t = None

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

        self.tag_world_rotations = {
            1: 0.0,
            2: 1.5 * np.pi,
            3: 0.5 * np.pi,
            4: 1.5 * np.pi,
            5: 0.5 * np.pi,
            6: 1.5 * np.pi,
            7: 0.5 * np.pi,
            8: np.pi,
            9: np.pi,
            10: np.pi,
        }

        # KALMAN PARAMS
        self.kf_dt = 0.1
        self.kf_F = np.array([
            [1, 0, self.kf_dt, 0],
            [0, 1, 0, self.kf_dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.kf_B = np.zeros((4, 2))    # no control input
        self.kf_H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        self.kf_Q = np.eye(4) * 0.001
        self.kf_R = np.eye(2) * 0.05
        self.kf_x0 = np.zeros((4, 1))
        self.kf_P0 = np.eye(4)


    def camera_info_callback(self, msg: CameraInfo):
        if self.K_received:
            return

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

        self.kalman_predict()

        if rover_xy is not None:
            # update position
            rover_xy = self.kalman_update(rover_xy)
            pos = Position()
            pos.x = float(rover_xy[0])
            pos.y = float(rover_xy[1])
            q_x,q_y,q_z,q_w = self.rover_orientation(visible_ids, distances, rover_xy)
        else:
            # predicted position
            pos = Position()
            pos.x = float(self.x_kf[0])
            pos.y = float(self.x_kf[1])
            q_x,q_y,q_z,q_w = self.rover_orientation(visible_ids, distances, (pos.x, pos.y))
        
        rover_position = PoseStamped(pose=Pose(position=Point(x=pos.x, y=pos.y), orientation=Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)), header=msg.header)

        self.position_pub.publish(rover_position)


    def svd_position(self, tag_ids, distances):
        tags_pos, tags_dist = [], []
        for tid, d in zip(tag_ids, distances):
            if tid in self.tag_world_positions:
                tags_pos.append(self.tag_world_positions[tid])
                tags_dist.append(d)

        tags_pos = np.array(tags_pos, float)
        tags_dist = np.array(tags_dist, float)
        n = len(tags_pos)

        # not enough tags
        if n < 2:
            return None

        # circle intersection with 2 tags
        if n == 2:
            pos1, pos2 = tags_pos
            d1, d2 = tags_dist
            D = np.linalg.norm(pos2 - pos1)

            if D > d1 + d2 or D < abs(d1 - d2):
                return None

            mid_dist = (np.power(d1, 2) - np.power(d2, 2) + np.power(D, 2)) / (2*D)
            discriminant = np.power(d1, 2) - np.power(mid_dist, 2)
            if discriminant < 0:
                return None

            P3 = pos1 + mid_dist * (pos2 - pos1) / D
            perp = np.array([-(pos2[1]-pos1[1])/D, (pos2[0]-pos1[0])/D]) * np.sqrt(discriminant)

            sol1, sol2 = P3 + perp, P3 - perp
            inside = lambda p: -3.0 <= p[0] <= 3.0 and -4.5 <= p[1] <= 4.5

            return sol1 if inside(sol1) else sol2

        # SVD with 3+ tags
        A = np.column_stack((2*tags_pos[:, 0], 2*tags_pos[:, 1], -np.ones(n)))
        b = (np.power(tags_pos[:,0], 2) + np.power(tags_pos[:,1], 2) - np.power(tags_dist, 2)).reshape(-1, 1)

        U, S, Vt = np.linalg.svd(A, full_matrices=False)
        x = Vt.T @ (np.linalg.inv(np.diag(S)) @ (U.T @ b))

        return x[:2, 0]


    def kalman_predict(self):
        self.x_kf = self.kf_F @ self.x_kf
        self.P_kf = self.kf_F @ self.P_kf @ self.kf_F.T + self.kf_Q

        return self.x_kf


    def kalman_update(self, z):
        H = self.kf_H
        R = self.kf_R
        z = np.array(z).reshape(2, 1)
        y = z - H @ self.x_kf
        S = H @ self.P_kf @ H.T + R
        K = self.P_kf @ H.T @ np.linalg.inv(S)
        self.x_kf = self.x_kf + K @ y
        I = np.eye(self.P_kf.shape[0])
        self.P_kf = (I - K @ H) @ self.P_kf

        return self.x_kf[:2, 0]


    def rover_orientation(self, visible_ids, distances, rover_xy):
        yaws = []
        weights = []

        for visible_id, dist in zip(visible_ids, distances):

            tag_pos = self.tag_world_positions[visible_id]
            tag_yaw = self.tag_world_rotations[visible_id]

            v = rover_xy - tag_pos
            angle_tag_to_rover = np.arctan2(v[1], v[0])

            rover_yaw = angle_tag_to_rover + tag_yaw + np.pi
            rover_yaw = (rover_yaw + np.pi) % (2 * np.pi) - np.pi

            w = 1.0 / max(dist, 0.001)
            yaws.append(rover_yaw)
            weights.append(w)

        mean_rot = np.arctan2(
            np.sum(np.sin(yaws) * weights),
            np.sum(np.cos(yaws) * weights)
        )

        # convert to quaternion
        cy = np.cos(mean_rot * 0.5)
        sy = np.sin(mean_rot * 0.5)

        return (0.0, 0.0, sy, cy)


    # def kalman_predict(self):
    #     now = self.get_clock().now().nanoseconds / 1e9
    #     if self.last_t is None:
    #         self.last_t = now
    #     dt = max(now - self.last_t, 1e-3)
    #     self.last_t = now

    #     A = np.array([
    #         [1.0, 0.0, dt,  0.0],
    #         [0.0, 1.0, 0.0, dt ],
    #         [1.0, 0.0, 1.0, 0.0],
    #         [0.0, 1.0, 0.0, 1.0]
    #     ])

    #     Q = np.eye(4) * 0.001

    #     self.x_kf = A @ self.x_kf
    #     self.P_kf = A @ self.P_kf @ A.T + Q


    # def kalman_update(self, z):
    #     H = np.array([[1, 0, 0, 0],
    #                 [0, 1, 0, 0]], float)
    #     R = self.R_kf

    #     z = np.array(z).reshape(2,1)
    #     y = z - H @ self.x_kf
    #     S = H @ self.P_kf @ H.T + R
    #     K = self.P_kf @ H.T @ np.linalg.inv(S)

    #     self.x_kf += K @ y
    #     self.P_kf = (np.eye(4) - K @ H) @ self.P_kf

    #     return self.x_kf[:2,0]


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()