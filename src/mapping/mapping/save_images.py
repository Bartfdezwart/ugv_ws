#!/usr/bin/env python3
from pathlib import Path
import re
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def find_highest_frame_id(basename: str, folder_path: Path) -> int:
    # Pattern to match filenames like basename_{id}.png
    pattern = fr".*{basename}_(\d+)\.png"

    highest_id = -1  # Start with an invalid ID
    for filename in folder_path.iterdir():
        match = re.match(pattern, str(filename))
        if match:
            # Extract the numeric id from the filename
            frame_id = int(match.group(1))
            # Update the highest_id if this id is greater
            highest_id = max(highest_id, frame_id)
    return None if highest_id == -1 else highest_id


class SaveImages(Node):
    def __init__(self):
        super().__init__("save_images")

        self.declare_parameter("base-filename", "frame")
        self.base_filename = self.get_parameter("base-filename").get_parameter_value().string_value

        self.declare_parameter("dir", str(Path().absolute()))
        self.out_directory = Path(self.get_parameter("dir").get_parameter_value().string_value)

        self.get_logger().info(f"Out directory: {self.out_directory.absolute()}")
        self.get_logger().info(f"Base filename: {self.base_filename}")

        self.out_directory.mkdir(exist_ok=True)

        highest_id = find_highest_frame_id(self.base_filename, self.out_directory)
        self.frame_id = 0
        if highest_id is not None:
            self.frame_id = highest_id + 1

        self.image_sub = self.create_subscription(
            Image, "/image", self.image_callback, 10
        )
        self.bridge = CvBridge()

    def image_callback(self, image: Image):
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        filename = self.out_directory / f"{self.base_filename}_{self.frame_id}.jpeg"
        cv2.imwrite(filename, frame)
        self.get_logger().info(f"Saved image: `{filename}`")
        self.frame_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = SaveImages()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
