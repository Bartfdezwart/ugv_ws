#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectLinesCanny(Node):
    def __init__(self):
        super().__init__('detect_lines_canny')
        self.get_logger().info('UGV LineDetection node started.')

        # Subscribe to the camera
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Publisher: list of lines as flat float array [x1, y1, x2, y2, ...]
        self.line_pub = self.create_publisher(Float32MultiArray, '/linedetect', 10)

        # Bridge for changing format to cv2
        self.bridge = CvBridge()
        
    # TODO: 
    # - Change white balance to detect ceiling light. 
    # - Select 5 - 10 ?brightest? lines. 
    # - Select longest from those, contender line, 
    # - Publish contender line and keep detecting contender until no longer visisble for x frames.
    # - Stop publishing line if no longer visible, send message to actuators to stop?
    def image_callback(self, msg):
        try:
            # Image preprocessing ---------------
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


            # Line detection --------------------
            edges = cv2.Canny(gray, 100, 200)
            lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
            

            # Message publishing ----------------
            line_msg = Float32MultiArray()
            data = []
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    data.extend([float(x1), float(y1), float(x2), float(y2)])
            line_msg.data = data

            self.line_pub.publish(line_msg)
            self.get_logger().debug(f"Published {len(data)//4} lines")


        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
                


def main(args=None):
    rclpy.init(args=args)
    node = DetectLinesCanny()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
