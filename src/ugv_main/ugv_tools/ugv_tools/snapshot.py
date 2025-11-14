#!/usr/bin/env python3
import argparse
import re
import select
import sys
import termios
import tty
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def find_highest_snapshot_id(basename: str, folder_path: Path):
    # Pattern to match filenames like snapshot_{id}.png
    pattern = fr".*{basename}_(\d+)\.png"

    highest_id = -1  # Start with an invalid ID
    for filename in folder_path.iterdir():
        match = re.match(pattern, str(filename))
        if match:
            # Extract the numeric id from the filename
            snapshot_id = int(match.group(1))
            # Update the highest_id if this id is greater
            highest_id = max(highest_id, snapshot_id)
    return highest_id


class Snapshot(Node):
    def __init__(self, img_dir: Path):
        super().__init__("snapshot")

        self.img_dir = img_dir
        if not self.img_dir.exists():
            self.img_dir.mkdir(parents=True)

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            "/image",
            self.image_callback,
            10,
        )

        self.latest_image = None
        self.img_basename = "snapshot"
        self.img_idx = find_highest_snapshot_id(self.img_basename, self.img_dir)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        # Set the terminal to raw mode
        tty.setraw(sys.stdin.fileno())
        # Wait for input
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        # If there is input, read it
        if rlist:
            key = sys.stdin.read(1)
        # If there is no input, set key to empty string
        else:
            key = ""
        # Set the terminal back to normal mode
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        # Return the key
        return key

    def image_callback(self, image: Image):
        self.latest_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

    def save_latest_image(self, img_dir: Path):
        if self.latest_image is not None:
            self.img_idx += 1

            img_path = img_dir / f"{self.img_basename}_{self.img_idx}.png"
            cv2.imwrite(img_path, self.latest_image)
            self.get_logger().info(f"Image saved as {img_path}")

    def run(self):
        self.get_logger().info(f"Started snapshot node. Save directory `{self.img_dir.absolute()}`")
        self.get_logger().info("Press 's' to save an image.")
        self.get_logger().info("Press 'q' to quit.")

        while rclpy.ok():
            key = self.getKey()
            if key == "q":
                self.get_logger().info("Exiting...")
                break
            elif key == "s":
                self.save_latest_image(self.img_dir)

            rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--img-dir",
        type=Path,
        required=True,
        help="The path to the directory where images will be saved",
    )

    parsed_args, remaining_args = parser.parse_known_args()

    rclpy.init(args=args)
    node = Snapshot(parsed_args.img_dir)
    node.run()
    # Set the terminal back to normal mode
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
