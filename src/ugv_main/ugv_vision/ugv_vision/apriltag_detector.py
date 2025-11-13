import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
from apriltag import apriltag

class ApriltagCtrl(Node):
    def __init__(self):
        super().__init__('apriltag_ctrl')
        # Create a subscription to the image_raw topic
        # self.image_raw_subscription = self.create_subscription(Image, '/image_raw', self.image_callback,10)
        self.image_raw_subscription = self.create_subscription(CompressedImage, '/image_rect/compressed', self.image_callback,10)
        # Create a publisher to the apriltag_ctrl/result topic
        self.apriltag_ctrl_publisher = self.create_publisher(Image, '/apriltag_ctrl/result', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        # Create an apriltag detector object
        # self.detector = apriltag("tag36h11")
        self.detector = apriltag("tagStandard41h12")
        
        # self.focal_length = 554.256  # Focal length in pixels
        # self.focal_length = 413.6420  # Focal length in pixels
        self.focal_length = 289.43385   # Focal length in pixels
        
        # self.tag_width = 0.158  # Tag width in meters
        # self.tag_width = 0.165  # Tag width in meters
        self.tag_width = 0.58  # Small tag width in meters
        
        self.intrinsic_params = np.array([[289.11451,   0.     , 347.23664],
                                          [0.     , 289.75319, 235.67429],
                                          [0.     ,   0.     ,   1.     ]])
        
        self.tag_points_3d = np.array([[-self.tag_width/2, -self.tag_width/2, 0],
                                       [ self.tag_width/2, -self.tag_width/2, 0],
                                       [ self.tag_width/2,  self.tag_width/2, 0],
                                       [-self.tag_width/2,  self.tag_width/2, 0]])
        
    def detect_apritag(self, frame):

        return type
   
    def image_callback(self, msg):

        # Convert the ROS Image message to an OpenCV image
        # frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Convert the image to grayscale
        scale = 2
        frame = cv2.resize(frame, (frame.shape[1] * scale, frame.shape[0] * scale))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply histogram equalization
        # gray = cv2.equalizeHist(gray)

        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=40, tileGridSize=(4,4))
        gray = clahe.apply(gray)
        
        # Sharpen image
        kernel = np.array([[0, -1, 0],
                           [-1, 5,-1],
                           [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)

        # Detect apriltags in the image
        results = self.detector.detect(gray)
        
        # frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Loop through the detected apriltags
        for r in results:
            # Get the corners of the apriltag
            corners = r['lb-rb-rt-lt'].astype(int)
        
            # Draw a polygon around the apriltag
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        
            # Get the center of the apriltag
            center_x, center_y = int(r['center'][0]), int(r['center'][1])
            # Draw a circle at the center of the apriltag
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            width_in_img = np.linalg.norm(corners[0] - corners[1]) + np.linalg.norm(corners[0] - corners[1]) / 2
            detect_distance = (self.tag_width * self.focal_length) / width_in_img
            # detect_distance += 0.11  # Current error margin calibration to camera lens
            detect_distance /= 5
            detect_distance -= 0.02  # Current error margin calibration to camera lens
            
            # rvec, tvec, _ = cv2.solvePnP(self.tag_points_3d,
            #                              r['lb-rb-rt-lt'].astype(np.float32),
            #                              self.intrinsic_params,
            #                              None)
            
            # print(tvec)
            # detect_distance = tvec[2][0]
        
            # Print the ID and center of the apriltag
            print(f'Tag ID: {r["id"]}, Center: ({center_x}, {center_y})')
            print(f'Estimated distance: {detect_distance:.2f} meters')
            # print(r)

        # Convert the OpenCV image back to a ROS Image message
        frame = cv2.resize(frame, (frame.shape[1] // scale, frame.shape[0] // scale))
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # result_img_msg = self.bridge.cv2_to_imgmsg(frame)
        # Publish the result image message
        self.apriltag_ctrl_publisher.publish(result_img_msg)
        # Show the result image
        cv2.imshow('ctrled Image', frame)
        # Wait for 1 millisecond
        cv2.waitKey(1)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # apriltag_ctrl = Apriltagctrl()
    # Create an instance of the ApriltagCtrl node
    apriltag_ctrl = ApriltagCtrl()
    # Spin the node
    rclpy.spin(apriltag_ctrl)
    # Destroy the node
    apriltag_ctrl.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()

