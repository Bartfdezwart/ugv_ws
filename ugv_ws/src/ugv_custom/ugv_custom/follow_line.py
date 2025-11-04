#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class FollowLine(Node):
    def __init__(self):
        super().__init__('follow_line')
        self.get_logger().info('UGV FollowLine node started.')

        # TODO: Subscribe to the DetectLines publisher
        self.subscription = self.create_subscription(Float32MultiArray, '/linedetect', self.follow_line, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.linear_speed = 0.15
        self.angular_speed = 0.005

    # TODO: (actuator script) 
    # Select candidate, continuously find line closest to candidate. 
    # If no line for x frames, switch to new candidate.
    def follow_line(self, msg):
        if not msg.data:
            self.get_logger().warn_throttle(5.0, "No lines detected")
            self.stop()
            # stop but wait for new line to start.
            return
        try:
            self.get_logger().debug(f"Actuator")

            lines = np.array(msg.data).reshape(-1, 4)

            # Pick the longest line (as the main line)
            lengths = np.sqrt((lines[:, 2] - lines[:, 0]) ** 2 + (lines[:, 3] - lines[:, 1]) ** 2)
            main_line = lines[np.argmax(lengths)]
            x1, y1, x2, y2 = main_line

            # Compute line midpoint and slope
            # Angle instead of slope?
            mid_x = (x1 + x2) / 2.0
            mid_y = (y1 + y2) / 2.0
            slope = (y2 - y1) / (x2 - x1 + 1e-6)

            # Steering error relative to image center, fixed img size
            image_center = 320
            error_x = mid_x - image_center

            # Steering
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.kp_angular * error_x 
            self.cmd_pub.publish(twist)

            self.get_logger().info_throttle(1.0, f"Following line: mid_x={mid_x:.1f}, slope={slope:.2f}, ang_z={twist.angular.z:.2f}")

        except Exception as e:
            self.get_logger().error(f"Actuator processing error: {e}")


    # TODO: 
    # If lidar detects wall, stop and wait for reverse or stop input. 
    def reverse(self):
        pass


    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
