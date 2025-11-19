#!/usr/bin/env python3
import datetime
from functools import partial
from pathlib import Path

import numpy as np
import rclpy
import rerun as rr
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, JointState
from ugv_interface.msg import AprilTagArray, LineArray

from ugv_tools.urdf_loader import URDFLogger

# The root is the ugv_ws/
WS_ROOT = Path(__file__).parents[6]
if WS_ROOT.parts[-1] != "ugv_ws":
    raise NotImplementedError


class RerunLogging(Node):
    def __init__(self):
        super().__init__("rerun_logging")

        # sink determines where logs are sent: "stream" or "file"
        self.declare_parameter("sink", "stream")
        sink = self.get_parameter("sink").get_parameter_value().string_value

        self.application_id = "rover"
        rr.init(self.application_id, spawn=False)

        if sink == "file":
            self.init_file_sink()
        elif sink == "stream":
            self.init_stream_sink()
        else:
            self.get_logger().error(
                f"invalid sink `{sink}`. Expected `file` or `stream`"
            )
            return

        # Image subscribers
        self.compressed_rect_image_sub = self.create_subscription(
            CompressedImage,
            "/image_rect/compressed",
            partial(self.log_compressed_image, image_name="rect"),
            10,
        )

        self.compressed_preprocessed_image_sub = self.create_subscription(
            CompressedImage,
            "/image_rect/preprocessed",
            partial(self.log_compressed_image, image_name="preprocessed"),
            10,
        )

        # Movement update subscribers
        self.robot_pose_sub = self.create_subscription(
            Pose2D,
            "/robot_pose",
            self.log_robot_pose,
            10,
        )

        self.camera_pose_sub = self.create_subscription(
            JointState,
            "/ugv/joint_states",
            self.log_camera_pose,
            10
        )

        # Detection subscribers
        self.top_lines_sub = self.create_subscription(
            LineArray,
            "/linedetect",
            partial(self.log_lines, line_name="detected_lines", rgb_color=(255, 0, 0)),
            10,
        )

        self.top_lines_sub = self.create_subscription(
            LineArray,
            "/linedetect_top",
            partial(self.log_lines, line_name="best_lines", rgb_color=(0, 0, 255)),
            10,
        )

        self.apriltags_sub = self.create_subscription(
            AprilTagArray,
            "/apriltags",
            self.log_april_tag,
            10,
        )
        self.clear_apriltags_timer = self.create_timer(0.2, self.clear_apriltags)
        self.apriltags_are_cleared = True

        self.bridge = CvBridge()

        self.log_urdf()
        self.log_field()

    def init_stream_sink(self):
        self.declare_parameter("rerun_ip", "127.0.0.1")
        rerun_port = 9876
        rerun_ip = self.get_parameter("rerun_ip").get_parameter_value().string_value
        rerun_addr = f"{rerun_ip}:{rerun_port}"

        rr.connect_tcp(rerun_addr)
        self.get_logger().info(f"started logging to rerun at {rerun_addr}")

    def init_file_sink(self):
        # Declare the log directory where all recording files will be stored of different
        # runs
        self.declare_parameter("log_dir", "rerun_logs")
        log_dir_name = self.get_parameter("log_dir").get_parameter_value().string_value
        log_dir = WS_ROOT / log_dir_name
        log_dir.mkdir(exist_ok=True)

        # Show a warning if there are a lot of recording files in the log directory
        number_of_recordings = sum(1 for _ in log_dir.iterdir())
        if number_of_recordings > 10:
            self.get_logger().warning(
                f"Stored {number_of_recordings} recordings in {log_dir}"
            )

        # Create the log file path
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file_name = f"recording_{timestamp}.rrd"
        log_file_path = log_dir / log_file_name
        rr.save(log_file_path)

        # Logs the file size of the current recording
        def display_file_size():
            size_bytes = log_file_path.stat().st_size
            gb_size = size_bytes / (1024**3)
            self.get_logger().info(f"File size: {gb_size:.2f} GB")

        # Each 5 seconds log to the terminal the size of the recording
        self.create_timer(5, display_file_size)

        self.get_logger().info(f"logging data to `{log_file_path}`")

    def log_urdf(self):
        urdf_folder = WS_ROOT / "src/ugv_main/ugv_description/urdf"
        rover_urdf = urdf_folder / "ugv_rover.urdf"

        urdf_logger = URDFLogger(rover_urdf, "world/rover")
        recording_stream = rr.get_global_data_recording()
        urdf_logger.log(recording_stream)

        for joint in urdf_logger.urdf.joints:
            entity_path = urdf_logger.joint_entity_path(joint)
            urdf_logger.log_joint(entity_path, joint, recording_stream)

    def log_field(self):
        field_path = WS_ROOT / "assets" / "field.glb"
        rr.log("/world/field", rr.Asset3D(path=field_path), static=True)

    def log_image(self, image: Image, image_name: str):
        cv_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        time_nanos = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log(f"camera/{image_name}", rr.Image(cv_img, rr.ColorModel.BGR))

    def log_compressed_image(
        self, compressed_image: CompressedImage, image_name: str
    ):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(
            compressed_image, desired_encoding="bgr8"
        )

        time_nanos = (
            compressed_image.header.stamp.sec * 1_000_000_000
            + compressed_image.header.stamp.nanosec
        )
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log(f"camera/compressed/{image_name}", rr.Image(cv_img, rr.ColorModel.BGR))

    def log_camera_pose(self, joint_states: JointState):
        try:
            index = joint_states.name.index("pt_base_link_to_pt_link1")
            angle = rr.Angle(rad=joint_states.position[index])
            rr.log(
            "world/rover/base_footprint/base_link/pt_base_link/pt_link1",
            rr.Transform3D(
                clear=False,
                rotation=rr.RotationAxisAngle(axis=(0, 0, 1), angle=angle),
            ),
        )
        # list.index(a) gives a ValueError when `a` can not be found in `list`
        except ValueError:
            pass

        try:
            index = joint_states.name.index("pt_link1_to_pt_link2")
            angle = rr.Angle(rad=joint_states.position[index])
            rr.log(
            "world/rover/base_footprint/base_link/pt_base_link/pt_link1/pt_link2",
            rr.Transform3D(
                clear=False,
                rotation=rr.RotationAxisAngle(axis=(0, -1, 0), angle=angle),
            ),
        )
        # list.index(a) gives a ValueError when `a` can not be found in `list`
        except ValueError:
            pass

    def log_robot_pose(self, pose: Pose2D):
        x = pose.x
        y = pose.y
        yaw = pose.theta

        rr.log(
            "world/rover",
            rr.Transform3D(
                translation=(x, y, 0),
                rotation=rr.RotationAxisAngle((0,0,1), rr.Angle(rad=yaw))
            )
        )


    def log_lines(self, lines: LineArray, line_name: str, rgb_color: tuple[int]):
        data = lines.data

        line_strips = []
        for i in range(0, len(data), 4):
            x1, y1, x2, y2 = data[i : i + 4]
            # Represent as a list of two points
            line_strips.append([(x1, y1), (x2, y2)])

        if not line_strips:
            return

        time_nanos = lines.header.stamp.sec * 1_000_000_000 + lines.header.stamp.nanosec
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log(
            f"lines/{line_name}",
            rr.LineStrips2D(strips=line_strips, colors=[rgb_color] * len(line_strips)),
        )

    def log_april_tag(self, apriltags: AprilTagArray):
        time_nanos = (
            apriltags.header.stamp.sec * 1_000_000_000 + apriltags.header.stamp.nanosec
        )
        rr.set_time_nanos("ros_time", time_nanos)

        centers = []
        lines = []
        half_sizes = []
        labels = []
        for tag in apriltags.detections:
            center = np.array((tag.centre.x, tag.centre.y))

            corners = np.array([(corner.x, corner.y) for corner in tag.corners])
            lines.append(np.vstack([corners, corners[0]]))

            min_xy = corners.min(axis=0)
            max_xy = corners.max(axis=0)
            half_size = (max_xy - min_xy) / 2.0

            centers.append(center)
            half_sizes.append(half_size)
            labels.append(f"Tag {tag.id}")

        # Log detected apriltags
        rr.log(
            "apriltag",
            rr.LineStrips2D(
                lines,
                colors=[(0, 255, 0)] * len(lines),
                labels=labels,
                show_labels=True,
            ),
        )

        # Reset the clear apriltags timer
        self.clear_apriltags_timer.reset()
        self.apriltags_are_cleared = False

    def clear_apriltags(self):
        if not self.apriltags_are_cleared:
            # Clear the apriltag entity
            rr.log("apriltag", rr.Clear(recursive=False))
            self.apriltags_are_cleared = True


def main(args=None):
    rclpy.init(args=args)
    logger = RerunLogging()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
