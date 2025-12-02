import numpy as np
import rclpy
from rclpy.node import Node
import math
import threading
import time
from geometry_msgs.msg import PoseStamped, Twist, PointStamped, Point
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
from nav_2d_msgs.msg import Path2D


class ApriltagDrive(Node):
    def __init__(self):
        super().__init__("apriltag_drive")

        self.pose_sub = self.create_subscription(
            PoseStamped, "/rover_pose", self.pose_callback, 10
        )
        self.target_point_pub = self.create_publisher(PointStamped, "target_point", 10)

        self.path_sub = self.create_subscription(Path2D, "/path", self.path_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_yaw_deg = None

        self.path = None
        self.path_point_idx = 0
        self.path_len = None
        self.look_ahead_offset = 5

        self.last_yaw_time = None

        self.yaw_history = []
        self.time_history = []
        self.yaw_rate = 0.0

        self.nav_target = None
        self.timer = self.create_timer(0.05, self.control_loop)

        thread = threading.Thread(target=self.input_thread, daemon=True)
        thread.start()

    def input_thread(self):
        while True:
            try:
                text = input("Enter coordinate (x y): ")
                x, y = map(float, text.split())
                # self.nav_target = (x, y)
                self.target_point_pub.publish(
                    PointStamped(
                        point=Point(x=x, y=y),
                        header=Header(stamp=self.get_clock().now().to_msg()),
                    )
                )
                print(f"Target set: {x:.2f}, {y:.2f}")
            except:
                print("Invalid input.")

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        yaw = Rotation.from_quat(quat).as_euler("xyz")[2]
        self.current_yaw = yaw
        self.current_yaw_deg = (math.degrees(yaw) + 360) % 360

        now = time.time()
        self.last_yaw_time = now

        self.yaw_history.append(self.current_yaw_deg)
        self.time_history.append(now)

        if len(self.yaw_history) > 10:
            self.yaw_history.pop(0)
            self.time_history.pop(0)

        if len(self.yaw_history) >= 2:
            d = self.normalize_angle(self.yaw_history[-1] - self.yaw_history[-2])
            dt = self.time_history[-1] - self.time_history[-2]
            if dt > 0:
                self.yaw_rate = d / dt

    def path_callback(self, path: Path2D):
        self.path = path.poses
        self.path_point_idx = 0
        self.path_len = len(self.path)
        pose = self.path[self.path_point_idx]
        self.nav_target = (pose.x, pose.y)

        self.path_point_target_idx = min(self.path_len -1, self.path_point_idx + self.look_ahead_offset)
        pose = self.path[self.path_point_target_idx]
        self.direction_target = (pose.x, pose.y)

    def control_loop(self):
        if self.nav_target is None or self.current_x is None:
            return

        current_position = np.array([self.current_x, self.current_y])

        pose = self.path[self.path_point_idx]
        current_target = np.array([pose.x, pose.y])
        dist_to_current_target = np.linalg.norm(current_target - current_position)

        # if (self.path_len - self.path_point_idx) < self.look_ahead_offset:
        goal = self.path[-1]
        distance_to_goal = np.linalg.norm(np.array([goal.x, goal.y]) - current_position)
        print("Distance:", distance_to_goal)
        if distance_to_goal < 0.2:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("Reached target.")
            self.nav_target = None
            return

        if dist_to_current_target < 0.05:
            self.path_point_idx = min(self.path_point_idx + 1, self.path_len - 1)
            self.path_point_target_idx = min(self.path_point_target_idx + 1, self.path_len - 1)
            self.get_logger().info(f"Point: {self.path_point_idx}/{self.path_len}")

            # If robot reached the final path point stop driving
            if self.path_point_idx == self.path_len:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info("Reached target.")
                self.nav_target = None
                return

            # If robot reached current target set navigation to next one
            pose = self.path[self.path_point_idx]
            self.nav_target = (pose.x, pose.y)

            pose = self.path[self.path_point_target_idx]
            self.direction_target = (pose.x, pose.y)

        else:
            if self.path_point_idx + 1 < self.path_len:
                # Compute distance to the upcomming point
                future_pose = self.path[self.path_point_idx + 1]
                future_target = np.array([future_pose.x, future_pose.y])
                dist_to_future_target = np.linalg.norm(future_target - current_position)

                # We can go progress to the next point if the distance of the upcomming
                # point is smaller than the distance to the current navigation point
                if dist_to_future_target < dist_to_current_target:
                    self.path_point_idx = min(self.path_point_idx + 1, self.path_len - 1)
                    self.path_point_target_idx = min(self.path_point_target_idx + 1, self.path_len - 1)
                    self.get_logger().info(f"Point: {self.path_point_idx}/{self.path_len}")

                    pose = self.path[self.path_point_idx]
                    self.nav_target = (pose.x, pose.y)

                    pose = self.path[self.path_point_target_idx]
                    self.direction_target = (pose.x, pose.y)


        tx, ty = self.direction_target
        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        desired_yaw = (math.degrees(math.atan2(dy, dx)) + 360) % 360

        # use measured yaw if available
        # if self.current_yaw_deg is not None:
        #     yaw_error = self.normalize_angle(desired_yaw - self.current_yaw_deg)
        # else:
        #     # fallback to predicted yaw
        #     predicted = self.predict_yaw()
        #     if predicted is None:
        #         return
        #     yaw_error = self.normalize_angle(desired_yaw - predicted)

        #     # get accuracy before moving
        #     if abs(yaw_error) > 10:
        #         return

        desired_yaw = math.atan2(-dx, dy)
        yaw_error = desired_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        twist = Twist()

        # if distance < 0.05:
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0
        #     self.cmd_pub.publish(twist)
        #     print("Reached target.")
        #     self.nav_target = None
        #     return

        if abs(yaw_error) > 0.25:
            twist.linear.x = 0.01
        else:
            twist.linear.x = 0.25

        twist.angular.z = 0.8 * yaw_error

        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize_angle(a):
        return (a + 180) % 360 - 180


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
