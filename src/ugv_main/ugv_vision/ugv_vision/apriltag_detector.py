import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
from apriltag import apriltag

class ApriltagCtrl(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        # Create a subscription to the image_raw topic
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create a publisher to the apriltag_ctrl/result topic
        self.apriltag_ctrl_publisher = self.create_publisher(Image, '/apriltag_ctrl/result', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        # Create an apriltag detector object
        # self.detector = apriltag("tag36h11")
        self.detector = apriltag("tagStandard41h12")
        
    def detect_apritag(self, frame):

        return type
   
    def image_callback(self, msg):

        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        scale_percent = 300 # percent of original size
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        
        # resize image
        resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

        contrast = 0.3
        resized = cv2.addWeighted(resized, contrast, np.zeros(resized.shape, resized.dtype), 0, 0)

        # Convert the image to grayscale
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

        clahe = cv2.createCLAHE(clipLimit=80, tileGridSize=(2,2))
        # gray = np.clip(clahe.apply(gray) + 30, 0, 255).astype(np.uint8)
        gray = clahe.apply(gray)

        # Create the sharpening kernel
        kernel = np.array([[0, -2, 0], [-2, 9, -2], [0, -2, 0]])
        # Sharpen the image
        sharpened_image = cv2.filter2D(gray, -1, kernel)




        # Detect apriltags in the image
        results = self.detector.detect(sharpened_image)
        vis = cv2.cvtColor(sharpened_image, cv2.COLOR_GRAY2BGR)


        # Loop through the detected apriltags
        for r in results:
            # Get the corners of the apriltag
            corners = r['lb-rb-rt-lt'].astype(int)
        
            # Draw a polygon around the apriltag
            cv2.polylines(vis, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        
            # Get the center of the apriltag
            center_x, center_y = int(r['center'][0]), int(r['center'][1])
            self.get_logger().info(f"{center_x, center_y}")
            # Draw a circle at the center of the apriltag
            cv2.circle(vis, (center_x, center_y), 5, (0, 0, 255), -1)

            # Print the ID and center of the apriltag
            print(f'Tag ID: {r["id"]}, Center: ({center_x}, {center_y})')

        # Convert the OpenCV image back to a ROS Image message
        result_img_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")                                                                                      
        # Publish the result image message
        self.apriltag_ctrl_publisher.publish(result_img_msg)
        # Show the result image


        width = int(frame.shape[1])
        height = int(frame.shape[0])
        dim = (width, height)
        
        # resize image
        resized2 = cv2.resize(vis, dim, interpolation = cv2.INTER_AREA)


        cv2.imshow('ctrled Image', resized2)
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

