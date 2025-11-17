import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from ugv_interface.msg import AprilTagArray, Position

import builtin_interfaces.msg


class ApriltagVisualize(Node):
    def __init__(self):
        super().__init__("apriltag_visualize")

        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_image_stamp = None
        self.latest_tags = None
        self.latest_tags_stamp = None
        self.latest_rover_pos = None

        self.image_sub = self.create_subscription(CompressedImage, "/image_rect/preprocessed", self.image_callback, 10)
        self.tag_sub = self.create_subscription(AprilTagArray, "/apriltags_distance", self.tag_callback, 10)
        self.pos_sub = self.create_subscription(Position, "/rover_position", self.position_callback, 10)
        self.get_logger().info("Apriltag visualizer with sync started.")

    def image_callback(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")
            return
        self.latest_image = img
        self.latest_image_stamp = self.to_sec(msg.header.stamp)
        self.draw_and_show()

    def tag_callback(self, msg):
        self.latest_tags = msg.detections
        self.latest_tags_stamp = self.to_sec(msg.header.stamp)
        self.draw_and_show()


    def position_callback(self, msg):
        self.latest_rover_pos = msg
        self.draw_and_show()


    @staticmethod
    def to_sec(stamp: builtin_interfaces.msg.Time):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def draw_and_show(self):

        if self.latest_image is None:
            return

        frame = self.latest_image.copy()

        tags_valid = False
        if self.latest_tags is not None and self.latest_tags_stamp is not None:
            dt = abs(self.latest_image_stamp - self.latest_tags_stamp)
            # 150 ms tolerance
            if dt < 0.15:
                tags_valid = True

        if tags_valid:
            for det in self.latest_tags:

                cx = int(det.centre.x)
                cy = int(det.centre.y)

                # center
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                # ID
                cv2.putText(frame, f"ID {det.id}", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                # distance
                cv2.putText(frame, f"{det.distance:.2f} m", (cx - 40, cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # outline
                pts = [(int(p.x), int(p.y)) for p in det.corners]
                pts_np = np.array(pts, dtype=np.int32)
                cv2.polylines(frame, [pts_np], True, (0, 255, 0), 2)
        if self.latest_rover_pos is not None:
            text = f"Rover: x={self.latest_rover_pos.x:.2f}  y={self.latest_rover_pos.y:.2f}"
            W = frame.shape[1]

            cv2.putText(frame, text, (int(W/2 - 200), 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("AprilTag Visualization", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagVisualize()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()