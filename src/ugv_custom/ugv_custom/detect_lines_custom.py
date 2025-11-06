#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ugv_interface.msg import LineArray

from pylsd2 import LineSegmentDetection
import pyelsed


class DetectLinesCustom(Node):
    def __init__(self):
        super().__init__('detect_lines_custom')
        self.get_logger().info('UGV LineDetection node started.')

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        self.line_pub = self.create_publisher(LineArray, '/linedetect', 10)

        self.top_line_pub = self.create_publisher(LineArray, '/linedetect_top', 10)
        
        self.preprocessed_img = self.create_publisher(Image, '/linedetect_preprocessed_img', 10)

        # Bridge for changing format to cv2
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        try:
            # Preprocessing ----------------------------------------
            # Convert OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_blur = cv2.GaussianBlur(gray, (3,3), 0) 
            gray = cv2.equalizeHist(img_blur)  
            _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
            kernel = np.ones((3, 3), np.uint8)  # You can adjust size (e.g., (5,5))
            erode = cv2.dilate(thresh, kernel, iterations=1)
            dilated = cv2.dilate(erode, kernel, iterations=1)

            # Preprocessing masking
            # mask = np.zeros_like(thresh)

            # h, w = thresh.shape
            # triangle = np.array([
            #     [ [0, 0], [w, 0], [w//2, h//2] ]
            # ], dtype=np.int32)
            # cv2.fillPoly(mask, triangle, 255)

            # h, _ = mask.shape
            # mask[0:h//2, :] = 255

            # thresh_masked = cv2.bitwise_and(thresh, mask)

            preprocessed_msg = self.bridge.cv2_to_imgmsg(dilated, encoding='mono8')
            self.preprocessed_img.publish(preprocessed_msg)


            lines, _ = pyelsed.detect(dilated)
            # lines = LineSegmentDetection(dilated) 

            # TODO: postprocessing

            
            # Prepare message
            line_msg = LineArray()
            data = []

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0], line[1], line[2], line[3]
                    data.extend([float(x1), float(y1), float(x2), float(y2)])
                line_msg.data = data
                self.line_pub.publish(line_msg)


        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
                
    def combinelines(self, lines):
        pass


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
