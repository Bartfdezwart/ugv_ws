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
        self.image_raw_subscription = self.create_subscription(Image,'/image_rect', self.image_callback,10)

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
        
        # self.focal_length = 413.6420 
        self.tw = 0.158 

        self.K_received = False
        self.camera_info_sub = self.create_subscription(CameraInfo, "/image_rect/camera_info", self.camera_info_callback,10)
        self.tag_points_3d = np.array([
            [-self.tw/2,  self.tw/2, 0],   # left-bottom  (lb)
            [ self.tw/2,  self.tw/2, 0],   # right-bottom (rb)
            [ self.tw/2, -self.tw/2, 0],   # right-top    (rt)
            [-self.tw/2, -self.tw/2, 0]    # left-top     (lt)
        ], dtype=np.float32)
        self.scale = 2
        self.K = []

    
    def camera_info_callback(self, msg: CameraInfo):
        if self.K_received:
            return
        K = np.array(msg.k, dtype=float).reshape(3, 3)

        K_scaled = K.copy()
        K_scaled[0, 0] *= self.scale
        K_scaled[1, 1] *= self.scale
        K_scaled[0, 2] *= self.scale
        K_scaled[1, 2] *= self.scale

        self.intrinsic_params = K
        self.K = K_scaled                
        self.get_logger().info("Camera intrinsics stored.")

        self.K_received = True


    def detect_apritag(self, frame):

        return type

    def image_callback(self, msg):
        if not self.K_received:
            self.get_logger().warn("Waiting for rectified camera_info...")
            return
        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

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

        if not self.visualize:
            return

        # Loop through the detected apriltags
        for r in results:
            # Get the corners of the apriltag
            corners = r['lb-rb-rt-lt'].astype(np.float32)
            center_x, center_y = int(r['center'][0]), int(r['center'][1])

            # Draw a polygon around the apriltag
            corners_i = corners.astype(int)
            cv2.polylines(frame, [corners_i], isClosed=True, color=(0, 255, 0), thickness=2)  

            # Draw a circle at the center of the apriltag
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
            # width_in_img = np.linalg.norm(corners[0] - corners[1]) + np.linalg.norm(corners[0] - corners[1]) / 2
            # detect_distance = (self.tag_width * self.tw) / width_in_img

            success, rvec, tvec = cv2.solvePnP(self.tag_points_3d, corners, self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE) 
            
            # make sure distance is positive
            # if distance < 0:
            #     distance = -distance

            distance = float(tvec[2][0])
            text = f"{distance:.2f} m"
            text_x = center_x - 40
            text_y = center_y - 25
            cv2.putText(frame,text,(text_x, text_y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0, 255, 0),2,cv2.LINE_AA)


            # Print the ID and center of the apriltag
            print(f'Tag ID: {r["id"]}, Center: ({center_x}, {center_y})')
            print()

        # Convert the OpenCV image back to a ROS Image message
        frame = cv2.resize(frame, (frame.shape[1] // self.scale, frame.shape[0] // self.scale))
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # result_img_msg = self.bridge.cv2_to_imgmsg(frame)
        # Publish the result image message
        self.apriltag_ctrl_publisher.publish(result_img_msg)
        # Show the result image
        cv2.imshow('ctrled Image', frame)
        # # Wait for 1 millisecond
        cv2.waitKey(1)


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
    # apriltag_ctrl = Apriltagctrl()
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

