import rclpy
from rclpy.node import Node
import math
import threading
from geometry_msgs.msg import PoseStamped, Twist
from scipy.spatial.transform import Rotation


class ApriltagDrive(Node):
    def __init__(self):
        super().__init__("apriltag_drive")

        self.pose_sub = self.create_subscription(
            PoseStamped, "/rover_pose", self.pose_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.current_x = None
        self.current_y = None
        self.current_yaw_deg = None

        self.nav_target = None

        self.timer = self.create_timer(0.05, self.control_loop)

        # Launch input thread
        thread = threading.Thread(target=self.input_thread, daemon=True)
        thread.start()

        self.get_logger().info("Waiting for input")

    def input_thread(self): 
        while True:
            try:
                text = input("Enter coordinate: ")
                parts = text.strip().split()
                if len(parts) != 2:
                    print("Invalid")
                    continue
                x = float(parts[0])
                y = float(parts[1])
                self.nav_target = (x, y)
                print(f"Target set to: {x:.2f}, {y:.2f}")
            except Exception as e:
                print(f"Input error: {e}")

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        quat = msg.pose.orientation
        _, _, self.current_yaw_deg = Rotation.from_quat(quat).as_euler(seq='xyz')

    def control_loop(self):
        if self.nav_target is None:
            return
        if self.current_x is None:
            return

        tx, ty = self.nav_target
        dx = tx - self.current_x
        dy = ty - self.current_y

        distance = math.sqrt(dx*dx + dy*dy)
        desired_yaw = (math.degrees(math.atan2(dy, dx)) + 360) % 360

        yaw_error = self.normalize_angle(desired_yaw - self.current_yaw_deg)

        twist = Twist()

        if distance < 0.20:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("Reached target.")
            self.nav_target = None
            return

        if abs(yaw_error) > 5:
            twist.linear.x = 0.0
            twist.angular.z = 0.35 if yaw_error > 0 else -0.35
        else:
            twist.linear.x = 0.25
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize_angle(a):
        a = (a + 180) % 360 - 180
        return a


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()