#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from ugv_interface.msg import LineArray


class DetectLinesCanny(Node):
    def __init__(self):
        super().__init__('detect_lines_canny')
        self.get_logger().info('UGV LineDetection node started.')

        self._declare_parameters()

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        self.line_pub = self.create_publisher(LineArray, '/linedetect', 10)

        self.top_line_pub = self.create_publisher(LineArray, '/linedetect_top', 10)
        
        self.preprocessed_img = self.create_publisher(Image, '/linedetect_preprocessed_img', 10)

        self.bridge = CvBridge()

        self.contender_line = None
        self.missed_frames = 0
        self.max_missed = 20
    
    def _declare_parameters(self):
        # Parameter blur
        self.declare_parameter("gaussian_blur", 3)

        # Canny parameters
        self.declare_parameter("canny_threshold1", 160)
        self.declare_parameter("canny_threshold2", 225)

        # Hough parameters
        self.declare_parameter("hough_rho", 1)
        self.declare_parameter("hough_theta", np.pi/180)
        self.declare_parameter("hough_threshold", 50)
        self.declare_parameter("hough_min_line_length", 80)
        self.declare_parameter("hough_max_line_gap", 10)

    def image_callback(self, msg):
        # Retrieve parameters as attributes
        self.gaussian_blur = self.get_parameter("gaussian_blur").value
        self.canny_threshold1 = self.get_parameter("canny_threshold1").value
        self.canny_threshold2 = self.get_parameter("canny_threshold2").value
        self.hough_rho = self.get_parameter("hough_rho").value
        self.hough_theta = self.get_parameter("hough_theta").value
        self.hough_threshold = self.get_parameter("hough_threshold").value
        self.hough_min_line_length = self.get_parameter("hough_min_line_length").value
        self.hough_max_line_gap = self.get_parameter("hough_max_line_gap").value

        try:
            # Image preprocessing ---------------
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_blur = cv2.GaussianBlur(gray, (self.gaussian_blur, self.gaussian_blur), 0) 
            gray = cv2.equalizeHist(img_blur)  
            _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)

            preprocessed_msg = self.bridge.cv2_to_imgmsg(thresh, encoding='mono8')
            self.preprocessed_img.publish(preprocessed_msg)


            # Line detection --------------------
            edges = cv2.Canny(thresh, self.canny_threshold1, self.canny_threshold2)
            # TODO: tweak params for ceiling lights
            lines = cv2.HoughLinesP(
                edges,
                rho=self.hough_rho,
                theta=self.hough_theta,
                threshold=self.hough_threshold,
                minLineLength=self.hough_min_line_length,
                maxLineGap=self.hough_max_line_gap
            )
            
            # Message publishing ----------------
            line_msg = LineArray()
            data = []

            if lines is not None:
                # flatten lines
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    data.extend([float(x1), float(y1), float(x2), float(y2)])
                line_msg.data = data
                self.line_pub.publish(line_msg)

                # compute line midpoints
                line_midpoints = [((l[0][0] + l[0][2]) / 2, (l[0][1] + l[0][3]) / 2) for l in lines]

                if self.contender_line is None:
                    # select the longest line
                    lines = sorted(lines, key=lambda l: np.hypot(l[0][2]-l[0][0], l[0][3]-l[0][1]), reverse=True)
                    x1, y1, x2, y2 = lines[0][0]
                    self.contender_line = (x1, y1, x2, y2)
                    self.missed_frames = 0
                    self.get_logger().info("Initial contender selected.")
                else:
                    # select line closest to previous contender midpoint
                    cx_prev = (self.contender_line[0] + self.contender_line[2]) / 2
                    cy_prev = (self.contender_line[1] + self.contender_line[3]) / 2

                    # compute distances from previous contender midpoint
                    distances = [np.hypot(cx_prev - mx, cy_prev - my) for (mx, my) in line_midpoints]
                    closest_idx = int(np.argmin(distances))
                    x1, y1, x2, y2 = lines[closest_idx][0]
                    self.contender_line = (x1, y1, x2, y2)
                    self.missed_frames = 0

                # publish contender line
                top_msg = LineArray()
                top_msg.data = [float(x1), float(y1), float(x2), float(y2)]
                self.top_line_pub.publish(top_msg)

            else:
                self.missed_frames += 1
                if self.missed_frames > self.max_missed:
                    self.get_logger().warn("No line detected for a while — stopping rover.")
                    self.top_line_pub.publish(LineArray(data=[]))
                    self.contender_line = None


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
