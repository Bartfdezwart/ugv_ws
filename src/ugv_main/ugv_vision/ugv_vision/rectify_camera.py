from pathlib import Path

import cv2
import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


def resolve_ros_path(path_str: str) -> str:
    """Resolve a ROS path to an absolute path."""
    if path_str.startswith("package://"):
        path = Path(path_str)
        package_name = path.parts[1]
        relative_path = Path(*path.parts[2:])

        package_path = resolve_ros2_package(
            package_name
        )

        if package_path is None:
            raise ValueError(
                f"Could not resolve {path}."
                f"Replace with relative / absolute path, source the correct ROS environment, or install {package_name}."
            )

        return str(package_path / relative_path)
    elif path_str.startswith("file://"):
        return path_str[len("file://") :]
    else:
        return path_str


def resolve_ros2_package(package_name: str) -> str | None:
    try:
        import ament_index_python

        try:
            return ament_index_python.get_package_share_directory(package_name)
        except ament_index_python.packages.PackageNotFoundError:
            return None
    except ImportError:
        return None


class RectifyCamera(Node):
    def __init__(self):
        super().__init__("rectify_camera")

        # Load the camera parameters from file
        self.declare_parameter("camera_parameter_file", "package://ugv_vision/config/better_camera_params.yaml")
        camera_parameter_file = self.get_parameter("camera_parameter_file").get_parameter_value().string_value
        if camera_parameter_file == "":
            raise RuntimeError("Required parameter 'camera_parameter_file' not set!")

        camera_parameter_file = Path(resolve_ros_path(camera_parameter_file))
        self.get_logger().info(f"Loading camera parameters from `{camera_parameter_file.absolute()}`")
        # Read the parameters from the yaml file
        with camera_parameter_file.open(mode="r") as file:
            config = yaml.safe_load(file)
            cam_mtx = config["camera_matrix"]
            # Set the camera matrix
            self.camera_matrix = np.array(cam_mtx["data"]).reshape(
                (cam_mtx["rows"], cam_mtx["cols"])
            )
            # Set the distortion coefficients
            self.dist_coef = np.array(config["distortion_coefficients"]["data"])

        # Initialize the cvbridge for the cv-ros image conversion
        self.bridge = CvBridge()

        # Image subscriber
        self.image_sub = self.create_subscription(
            Image, "image", self.image_callback, 10
        )

        # Publisher for the rectified image
        self.rect_pub = self.create_publisher(Image, "image_rect", 10)
        # Publihser for the compressed rectified image
        self.rect_compressed_pub = self.create_publisher(
            CompressedImage, "image_rect/compressed", 10
        )

        self.get_logger().info("Started rectify camera node")

    def image_callback(self, image_msg: Image):
        # Count the number of subscribers so we can determine later if we should publish
        rect_subscriber_count = self.rect_pub.get_subscription_count() > 0
        rect_compressed_subscriber_count = self.rect_compressed_pub.get_subscription_count() > 0

        # Do nothing when both rectified and rectified compressed topics have no subscribers
        if not (rect_subscriber_count or rect_compressed_subscriber_count):
            return

        # Convert ros image to cv2 image
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        # Rectify the image with the camera matrix and distortion coefficients
        rectified_image = cv2.undistort(image, self.camera_matrix, self.dist_coef)

        # Publish rectified image
        if rect_subscriber_count:
            self.rect_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    rectified_image, encoding="bgr8", header=image_msg.header
                )
            )

        # Publish a compressed version of the rectified image
        if rect_compressed_subscriber_count:
            msg = self.bridge.cv2_to_compressed_imgmsg(
                rectified_image,
            )
            msg.header = image_msg.header
            self.rect_compressed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RectifyCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
