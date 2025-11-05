#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectLinesCustom(Node):
    def __init__(self):
        super().__init__('detect_lines_custom')
        self.get_logger().info('UGV LineDetection node started.')

        # Subscribe to the camera
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Publisher: list of lines as flat float array [x1, y1, x2, y2, ...]
        self.line_pub = self.create_publisher(Float32MultiArray, '/linedetect', 10)

        # Bridge for changing format to cv2
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        try:
            # Convert OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Detect edges

            # TODO: own line detector
            edges = []
            lines = []
            
            # Prepare message
            line_msg = Float32MultiArray()
            data = []
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    data.extend([float(x1), float(y1), float(x2), float(y2)])
            line_msg.data = data

            # Publish list of detected lines
            self.line_pub.publish(line_msg)
            self.get_logger().debug(f"Published {len(data)//4} lines")

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
                


def main(args=None):
    rclpy.init(args=args)
    node = DetectLinesCustom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
