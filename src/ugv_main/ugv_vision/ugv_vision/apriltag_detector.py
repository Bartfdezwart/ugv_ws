import argparse

import cv2
import numpy as np
import rclpy
from apriltag import apriltag
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from ugv_interface.msg import AprilTag, AprilTagArray, Point


class ApriltagCtrl(Node):
    def __init__(self, visualize: bool = False):
        super().__init__('apriltag_ctrl')
        self.visualize = visualize

        # Create a subscription to the image_raw topic
        self.image_raw_subscription = self.create_subscription(CompressedImage,'/image_rect/compressed', self.image_callback,10)

        self.image_preprocessed_publisher = self.create_publisher(CompressedImage,'/image_rect/preprocessed',10)
        # Create a publisher to the apriltag_ctrl/result topic
        self.apriltag_ctrl_publisher = self.create_publisher(Image, '/apriltag_ctrl/result', 10)
        # Create a publisher to publish april tags
        self.apriltags_publisher = self.create_publisher(AprilTagArray, '/apriltags', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        # Create an apriltag detector object
        # self.detector = apriltag("tag36h11")
        self.apriltag_family = "tagStandard41h12"
        self.detector = apriltag(self.apriltag_family)
        self.scale = 2

    def detect_apritag(self, frame):
        return type


    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (gray.shape[1] * self.scale, gray.shape[0] * self.scale))

        # Sharpen image
        kernel = np.array([[0, -1, 0],
                           [-1, 5,-1],
                           [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)

        # Publish the preprocessed image
        preprocessed_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        preprocessed_msg = self.bridge.cv2_to_compressed_imgmsg(preprocessed_image)
        preprocessed_msg.header = msg.header
        self.image_preprocessed_publisher.publish(preprocessed_msg)

        # Detect apriltags in the image
        results = self.detector.detect(gray)

        frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Publish the found april tags
        if results:
            print(results[0])
            self.apriltags_publisher.publish(
                AprilTagArray(
                    header=msg.header,
                    detections=[
                        AprilTag(
                            family = self.apriltag_family,
                            id=detection["id"],
                            hamming=detection["hamming"],
                            decision_margin=detection["margin"],
                            centre=Point(x=detection["center"][0], y=detection["center"][1]),
                            corners=[Point(x=corner[0], y=corner[1]) for corner in detection["lb-rb-rt-lt"]],
                        )
                        for detection in results
                    ]
                )
            )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Whether to open a cv2 window to show the april tag detections",
    )

    parsed_args, remaining_args = parser.parse_known_args()

    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of the ApriltagCtrl node
    apriltag_ctrl = ApriltagCtrl(parsed_args.visualize)
    # Spin the node
    rclpy.spin(apriltag_ctrl)
    # Destroy the node
    apriltag_ctrl.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()

