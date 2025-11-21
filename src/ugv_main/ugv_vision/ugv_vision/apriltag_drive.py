import rclpy
from rclpy.node import Node
import math
import threading
import time
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
                self.nav_target = (x, y)
                print(f"Target set: {x:.2f}, {y:.2f}")
            except:
                print("Invalid input.")

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w] 

        yaw = Rotation.from_quat(quat).as_euler('xyz')[2]
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

    def predict_yaw(self):
        """Predict yaw when no new detection arrives."""
        if self.current_yaw_deg is None or self.last_yaw_time is None:
            return None

        now = time.time()
        dt = now - self.last_yaw_time  

        predicted = (self.current_yaw_deg + self.yaw_rate * dt) % 360
        return predicted

    def control_loop(self):
        if self.nav_target is None or self.current_x is None:
            return

        tx, ty = self.nav_target
        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        desired_yaw = (math.degrees(math.atan2(dy, dx)) + 360) % 360

        # use measured yaw if available
        if self.current_yaw_deg is not None:
            yaw_error = self.normalize_angle(desired_yaw - self.current_yaw_deg)
        else:
            # fallback to predicted yaw
            predicted = self.predict_yaw()
            if predicted is None:
                return
            yaw_error = self.normalize_angle(desired_yaw - predicted)

            # get accuracy before moving
            if abs(yaw_error) > 10:
                return

        twist = Twist()

        if distance < 0.20:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            print("Reached target.")
            self.nav_target = None
            return

        if abs(yaw_error) > 15:
            twist.linear.x = 0.0
            twist.angular.z = 0.35 if yaw_error > 0 else -0.35
        else:
            twist.linear.x = 0.25
            twist.angular.z = 0.0

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