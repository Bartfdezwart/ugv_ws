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

        # Subscribe to the camera
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Publisher: list of lines as flat float array [x1, y1, x2, y2, ...]
        self.line_pub = self.create_publisher(LineArray, '/linedetect', 10)

        # Flat list of top line.
        self.top_line_pub = self.create_publisher(LineArray, '/linedetect_top', 10)
        
        # Bridge for changing format to cv2
        self.bridge = CvBridge()


        self.contender_line = None
        self.missed_frames = 0
        self.max_missed = 20
        

    def image_callback(self, msg):
        try:
            # Image preprocessing ---------------
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)  
            _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

            # Line detection --------------------
            edges = cv2.Canny(thresh, 100, 200)
            # TODO: tweak params for ceiling lights
            lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
            
            # Message publishing ----------------
            line_msg = LineArray()
            data = []

            if lines is not None:
                # flatten all lines for publication
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    data.extend([float(x1), float(y1), float(x2), float(y2)])
                line_msg.data = data
                self.line_pub.publish(line_msg)

                # compute line midpoints for all
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
                # self.get_logger().info("Tracking contender line")

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
