#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ugv_interface.msg import LineArray

# TODO: Import apriltag


class DetectMarkers(Node):
    def __init__(self):
        super().__init__('detect_markers')
        self.get_logger().info(f'UGV MarkerDetection node started.')

        self._declare_parameters()

        self.image_sub = self.create_subscription(Image, '/image_rect', self.image_callback, 10)
        self.marker_pub = self.create_publisher(LineArray, '/markerdetect', 10)
        self.preprocessed_img = self.create_publisher(Image, '/linedetect_preprocessed_img', 10)

        self.bridge = CvBridge()

        self.best_line = None


    def _declare_parameters(self):
        self.declare_parameter("gaussian_blur", 3)
        self.declare_parameter("binary_threshold_lower", 220)
        self.declare_parameter("binary_threshold_upper", 255)
        self.declare_parameter("opening_kernelsize", 3)


    def image_callback(self, msg):
        try:
            self.gaussian_blur = self.get_parameter("gaussian_blur").value
            self.binary_threshold_lower = self.get_parameter("binary_threshold_lower").value
            self.binary_threshold_upper = self.get_parameter("binary_threshold_upper").value
            self.opening_kernelsize = self.get_parameter("opening_kernelsize").value

            # TODO: change preprocessing for marker detection 
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_blur = cv2.GaussianBlur(gray, (self.gaussian_blur, self.gaussian_blur), 0) 
            gray = cv2.equalizeHist(img_blur)  
            _, thresh = cv2.threshold(gray, self.binary_threshold_lower, self.binary_threshold_upper, cv2.THRESH_BINARY)
            kernel = np.ones((self.opening_kernelsize, self.opening_kernelsize), np.uint8)
            dilated = cv2.dilate(thresh, kernel, iterations=2)

            # publish preprocessed image
            preprocessed_msg = self.bridge.cv2_to_imgmsg(dilated, encoding='mono8', header=msg.header)
            self.preprocessed_img.publish(preprocessed_msg)

            # TODO: Detect markers using apriltag!
            markers = []


            # prepare message
            marker_msg = LineArray()
            data = []
            for marker in markers:
                x1, y1, x2, y2 = marker[0], marker[1], marker[2], marker[3]
                data.extend([float(x1), float(y1), float(x2), float(y2)])
            marker_msg.data = data
            marker_msg.header = msg.header
            self.marker_pub.publish(marker_msg)


        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")



def main(args=None):
    rclpy.init()

    node = DetectMarkers()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
