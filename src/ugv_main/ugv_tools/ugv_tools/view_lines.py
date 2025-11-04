#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import cv2

from ugv_interface.msg import LineArray


class ViewLines(Node):
    def __init__(self):
        super().__init__("canny_detector")
        # Correct subscribers with allow_headerless for headerless messages
        self.lines_subscription = Subscriber(
            self,
            LineArray,
            "/line_detection_array",
        )

        self.image_subscription = Subscriber(
            self,
            Image,
            "/image",
        )

        self.time_sync = ApproximateTimeSynchronizer([self.image_subscription, self.lines_subscription], queue_size=10, slop=0.05, allow_headerless=False)
        self.time_sync.registerCallback(self.view_lines)

        self.bridge = CvBridge()
    
    def view_lines(self, image, lines):
        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

            # Extract lines from Float32MultiArray
            data = lines.data
            # Each line has 4 floats: x1, y1, x2, y2
            for i in range(0, len(data), 4):
                x1, y1, x2, y2 = map(int, data[i:i+4])
                # Draw line on the frame (blue color, thickness=2)
                cv2.line(frame, (x1, y1), (x2, y2), color=(255, 0, 0), thickness=2)

            # Show the resulting image
            cv2.imshow("Lines Overlay", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error drawing lines: {e}")
    
def main(args=None):
    rclpy.init(args=args)
    viewer = ViewLines()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
  main()
