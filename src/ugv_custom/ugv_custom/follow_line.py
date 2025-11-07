#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
from ugv_interface.msg import LineArray


class FollowLine(Node):
    def __init__(self):
        super().__init__('follow_line')
        self.get_logger().info('UGV Follow Line node started')

        self.image_sub_once = self.create_subscription(CameraInfo, '/camera_info', self._get_image_center, 10)
        self.line_sub = self.create_subscription(LineArray, '/linedetect_top', self.drive, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self._declare_parameters()

        self.cam_center = []

    def _declare_parameters(self):
        self.declare_parameter("speed", 0.2)


    def _get_image_center(self, msg):
        self.get_logger().info(f"{msg.width}")
        self.get_logger().info(f"{msg.height}")
        self.cam_center = [msg.height/2, msg.width/2]
        self.destroy_subscription(self.image_sub_once)
        self.image_sub_once = None


    def drive(self, msg):
        try:
            self.speed = self.get_parameter("speed").value

            x1, y1, x2, y2 = msg.data[0], msg.data[1], msg.data[2], msg.data[3]
            langle = self._line_angle([x1, y1, x2, y2])
        

            # make line very long
            vx, vy = np.cos(langle), np.sin(langle)
            x3, y3 = x1 - 1000*vx, y1 - 1000*vy
            x4, y4 = x2 + 1000*vx, y2 + 1000*vy

            # compute distance from image center to this vector
            cx, cy = self.cam_center
            num = abs((y4 - y3)*cx - (x4 - x3)*cy + x4*y3 - y4*x3)
            den = np.hypot(y4 - y3, x4 - x3)
            dist = num / den
            self.get_logger().info(f"f{dist}")

            # get angle between center and vector.
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            dx, dy = mx - cx, cy - my
            angle_to_line = np.degrees(np.arctan2(dy, dx))


            # drive based on angle
            cmd = Twist()
 
            # top-left
            if dx < 0 and dy > 0:
                cmd.linear.x = self.speed
                cmd.angular.z = -0.4
            # top-right
            elif dx > 0 and dy > 0:  
                cmd.linear.x = self.speed
                cmd.angular.z = 0.4
            # bottom-left
            elif dx < 0 and dy < 0:  
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
            # bottom-right
            else:  
                cmd.linear.x = 0.0
                cmd.angular.z = -0.5

            self.pub_cmd.publish(cmd)
            self.get_logger().info(f"dist={dist:.1f}, angle={angle_to_line:.1f}, cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})")


        except Exception as e:
            self.get_logger().error(f"drive error: {e}")

    def _line_angle(self, line):
        return np.arctan2(line[3] - line[1], line[2] - line[0])


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