import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from ugv_interface.msg import AprilTag, AprilTagArray
import math
from scipy.optimize import minimize
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from std_msgs.msg import Header

from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

from time import sleep


class ApriltagDistance(Node):
    def __init__(self, visualize: bool = False):
        super().__init__('apriltag_distance')
        self.visualize = visualize

        self.tag_sub = self.create_subscription(AprilTagArray, '/apriltags', self.tag_callback, 10)
        self.tags_distance_pub = self.create_publisher(AprilTagArray, '/apriltags_distance', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/rover_pose', 10)

        # Subscribe to joint states (ugv/joint_states topic)
        self.joint_states_pub = self.create_publisher(JointState, '/ugv/joint_states', 10)

        self.velocity_cmd = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

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
            2: 270,
            3: 90,
            4: 270,
            5: 90,
            6: 270,
            7: 90,
            8: 180,
            9: 180,
            10: 180,
        }

        # Delta time use for the Kalman filter
        self.kf_dt = 0.1

        self.ukf_init()
        self.kalmancall = self.create_timer(self.kf_dt, self.kalman_timer)
        self.pose_update = self.create_timer(0.1, self.update_pose)

        self.get_logger().info("Node started")

    # Initializes the UKF
    def ukf_init(self):
        def fx(x, dt, linear_velocity, angular_velocity):
            px, py, theta = x
            offset = np.pi / 2
            px_new = px + linear_velocity * np.cos(theta + offset) * dt
            py_new = py + linear_velocity * np.sin(theta + offset) * dt
            theta_new = theta + angular_velocity * dt

            return np.array([px_new, py_new, theta_new])

        points = MerweScaledSigmaPoints(3, alpha=0.1, beta=2.0, kappa=-1)

        def x_mean_fn(sigmas, Wm):
            x = np.dot(Wm, sigmas[:, 0])
            y = np.dot(Wm, sigmas[:, 1])

            sin_sum = np.dot(Wm, np.sin(sigmas[:, 2]))
            cos_sum = np.dot(Wm, np.cos(sigmas[:, 2]))
            theta = np.arctan2(sin_sum, cos_sum)

            return np.array([x, y, theta])

        def residual_x(a, b):
            y = a - b
            y[2] = (y[2] + np.pi) % (2*np.pi) - np.pi
            return y

        def make_z_mean_fn(angle_idx=None):
            def z_mean_fn(sigmas, Wm):
                mean = np.dot(Wm, sigmas)
                if angle_idx is not None:
                    sin_sum = np.dot(Wm, np.sin(sigmas[:, angle_idx]))
                    cos_sum = np.dot(Wm, np.cos(sigmas[:, angle_idx]))
                    mean[angle_idx] = np.arctan2(sin_sum, cos_sum)
                return mean
            return z_mean_fn

        def make_residual_z(angle_idx=None):
            def residual_z(a, b):
                y = a - b
                if angle_idx is not None:
                    y[angle_idx] = (y[angle_idx] + np.pi) % (2*np.pi) - np.pi
                return y
            return residual_z

        def make_hx(*, position: bool = False, orientation: bool = False):
            indices = []
            if position:
                indices.extend([0, 1])
            if orientation:
                indices.append(2)

            def hx(x):
                h = x[indices]
                return h
            return hx

        def make_R(
            *,
            position_std: float | None = None,
            orientation_std: float | None = None,
        ):
            diagonal = []
            if position_std:
                diagonal.extend([position_std**2, position_std**2])
            if orientation_std:
                diagonal.append(orientation_std**2)

            R = np.diag(diagonal)
            return R



        self.make_z_mean_fn = make_z_mean_fn
        self.make_residual_z = make_residual_z
        self.make_hx = make_hx
        self.make_R = make_R
        self.ukf = UnscentedKalmanFilter(
            dim_x=3,
            dim_z=3,
            dt=self.kf_dt,
            hx=None,
            fx=fx,
            points=points,
            x_mean_fn=x_mean_fn,
            z_mean_fn=None,
            residual_x=residual_x,
            residual_z=None,
        )

        self.ukf.x = np.array([0., 0., 0.]) # initial state
        self.ukf.P *= 0.2 # initial uncertainty
        self.ukf.Q = np.diag([0.01, 0.01, 0.001])**2

        self.apriltag_position_measurement_std = 0.1
        self.apriltag_orientation_measurement_std = 0.1


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

    def velocity_callback(self, vel_cmd: Twist):
        # Update the rover velocity from the send velocity commands
        self.linear_velocity = vel_cmd.linear.x
        self.angular_velocity = vel_cmd.angular.z

    def tag_callback(self, msg: AprilTagArray):

        if not self.K_received:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        visible_ids = []
        distances = []
        rvecs = {}

        out_msg = AprilTagArray()
        out_msg.header = msg.header
        out_msg.detections = []

        for det in msg.detections:

            corners = np.array([[p.x, p.y] for p in det.corners], dtype=np.float32)

            _, rvec, tvec = cv2.solvePnP(self.tag_points_3d, corners, self.K, None,
                                         flags=cv2.SOLVEPNP_IPPE_SQUARE)

            distance = float(np.linalg.norm(tvec))

            # store rvec
            rvecs[det.id] = rvec

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
        rover_yaw = self.rover_orientation(visible_ids, distances, rvecs)

        self.tags_distance_pub.publish(out_msg)

        # UKF measurement update
        has_position_measure = rover_xy is not None
        has_orientation_measure = rover_yaw is not None

        # Do nothing if there are no measurements
        if not has_position_measure and not has_orientation_measure:
            return

        # Setting the z_mean and residual_z
        # Determine which measurement index corresponds to an angle, if any
        angle_idx = None
        if has_orientation_measure:
            # Start with the first element for orientation if no position measurements
            angle_idx = 0 if not has_position_measure else 2

        self.ukf.z_mean = self.make_z_mean_fn(angle_idx)
        self.ukf.residual_z = self.make_residual_z(angle_idx)

        # Setting the measurement noise
        pos_std = None
        orientation_std = None
        if has_position_measure:
            pos_std = self.apriltag_position_measurement_std
        if has_orientation_measure:
            orientation_std = self.apriltag_orientation_measurement_std
        R = self.make_R(position_std=pos_std, orientation_std=orientation_std)

        # Setting the measurement model
        H = self.make_hx(position=has_position_measure, orientation=has_orientation_measure)

        measurements = self.combine_non_none(rover_xy, rover_yaw)

        self.ukf.update(measurements, R, hx=H)


    def svd_position(self, tag_ids, distances):
        tags_pos, tags_dist = [], []
        for tid, d in zip(tag_ids, distances):
            if tid in self.tag_world_positions:
                tags_pos.append(self.tag_world_positions[tid])
                tags_dist.append(d)

        tags_pos = np.array(tags_pos, float)
        tags_dist = np.array(tags_dist, float)
        n = len(tags_pos)

        if n < 2:
            return None

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

        A = np.column_stack((2*tags_pos[:, 0], 2*tags_pos[:, 1], -np.ones(n)))
        b = (np.power(tags_pos[:,0], 2) + np.power(tags_pos[:,1], 2) - np.power(tags_dist, 2)).reshape(-1, 1)

        U, S, Vt = np.linalg.svd(A, full_matrices=False)
        x = Vt.T @ (np.linalg.inv(np.diag(S)) @ (U.T @ b))

        position = x[:2, 0]

        refined_position = self.refine_position(position, tags_pos, tags_dist)

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

    def rotate_camera(self, x, y, rotation):
        try:
            rover_pos = np.array([x, y])
            distances = []

            for tag_id, tag_pos in self.tag_world_positions.items():
                dist = np.linalg.norm(rover_pos - tag_pos)
                distances.append((dist, tag_id))
            # Get the 2 closest tags
            closest_points = sorted(distances, key=lambda item: item[0])[:2]
            target_tags = [tag_id for _, tag_id in closest_points]

            print("Best target tags: ", target_tags)

            target = (self.tag_world_positions[target_tags[0]] + self.tag_world_positions[target_tags[1]]) / 2.0

            # Calculate angle to target
            delta_x = target[0] - x
            delta_y = target[1] - y
            angle_to_target = rotation - math.atan2(delta_y, delta_x)
            print(f"Rotating camera to angle (rad): {angle_to_target}")

            js_msg = JointState()
            js_msg.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
            js_msg.position = [angle_to_target, 0.0]  # Rotate from -60 to +60 degrees
            self.joint_states_pub.publish(js_msg)
            sleep(0.2)  # Allow time for the camera to rotate
        except Exception as e:
            print(f"Error in rotate_camera: {e}")
            pass


    def kalman_timer(self):
        # Update the next state estimation and uncertainty
        self.ukf.predict(
            self.kf_dt,
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
        )

    def combine_non_none(self, *args):
        values = []

        for arg in args:
            if arg is None:
                continue
            # try to iterate if it’s array-like
            try:
                iter(arg)
            except TypeError:
                # scalar, append as single element
                values.append(arg)
            else:
                # iterable, extend values
                values.extend(arg)

        return np.array(values, dtype=float)

    def rover_orientation(self, visible_ids, distances, rvecs):
        yaw_list = []
        weight_list = []

        for tag_id, dist in zip(visible_ids, distances):

            if tag_id not in rvecs:
                continue

            tag_yaw_world = self.tag_world_rotations[tag_id]

            rvec = rvecs[tag_id]

            R_tag_to_cam, _ = cv2.Rodrigues(rvec)
            R_cam_to_tag = R_tag_to_cam.T

            cam_yaw_tag_rad = math.atan2(R_cam_to_tag[1, 0], R_cam_to_tag[0, 0])
            cam_yaw_tag_deg = math.degrees(cam_yaw_tag_rad) % 360.0

            rover_yaw_world = (tag_yaw_world - cam_yaw_tag_deg) % 360.0

            w = 1.0 / max(dist, 0.001)
            yaw_list.append(rover_yaw_world)
            weight_list.append(w)

        if not yaw_list:
            return None
            yaw_deg = 0.0

        yaw_rad_list = [math.radians(y) for y in yaw_list]
        weights = np.array(weight_list)

        mean_rad = math.atan2(
            np.sum(np.sin(yaw_rad_list) * weights),
            np.sum(np.cos(yaw_rad_list) * weights)
        )
        yaw_deg = (math.degrees(mean_rad) + 180.0) % 360.0
        # yaw_deg = math.degrees(mean_rad) % 360.0
        # yaw_deg = (yaw_deg + 180.0) % 360.0

        yaw_rad = math.radians(yaw_deg)
        return yaw_rad
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)

        return 0.0, 0.0, sy, cy

    def update_pose(self):
        # Get the estimated pose from the UKF
        x = float(self.ukf.x[0])
        y = float(self.ukf.x[1])
        yaw = float(self.ukf.x[2])

        # Transform the yaw of the robot to a quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        rover_position = PoseStamped(
            pose=Pose(
                position=Point(x=x, y=y),
                orientation=Quaternion(x=0.0, y=0.0, z=sy, w=cy)
            ),
            header=Header(stamp=self.get_clock().now().to_msg())
        )
        # Publish the rover position
        self.position_pub.publish(rover_position)


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()