#!/usr/bin/env python3
import datetime
from functools import partial
from pathlib import Path

import numpy as np
import rclpy
import rerun as rr
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, JointState, LaserScan
from std_msgs.msg import Header, Int8MultiArray, MultiArrayLayout, MultiArrayDimension

from ugv_interface.msg import AprilTagArray, LineArray
from nav_2d_msgs.msg import Path2D
from ament_index_python.packages import get_package_share_directory

from ugv_tools.urdf_loader import URDFLogger, origin_to_transform

IMAGE_DETECTION_SIZE = np.array([1280, 960]) * 2
IMAGE_STREAM_SIZE = np.array([640, 480])
IMAGE_DETECTION_TO_STREAM_SCALE = IMAGE_DETECTION_SIZE / IMAGE_STREAM_SIZE

from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve()

for parent in SCRIPT_PATH.parents:
    if parent.name == "ugv_ws":
        WS_ROOT = parent
        break
else:
    raise RuntimeError(f"Could not find workspace root starting from {SCRIPT_PATH}")

print(f"Workspace root detected as: {WS_ROOT}")

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
            PoseStamped,
            "/rover_pose",
            self.log_robot_pose,
            10,
        )

        self.camera_pose_sub = self.create_subscription(
            JointState, "/ugv/joint_states", self.log_camera_pose, 10
        )

        # Line detection subscribers
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

        # April tag logging
        self.apriltags_sub = self.create_subscription(
            AprilTagArray,
            "/apriltags_distance",
            self.log_april_tag,
            10,
        )
        self.clear_apriltags_timer = self.create_timer(0.2, self.clear_apriltags)
        self.apriltags_are_cleared = True

        self.target_point_sub = self.create_subscription(
            PointStamped, "target_point", self.log_target_point, 10
        )

        self.beacon_sub = self.create_subscription(
            PoseArray, "beacon_pose", self.log_beacons, 10
        )

        # Lidar logging
        # self.lidar_sub = self.create_subscription(
        #     LaserScan, "/scan", self.log_lidar, 10
        # )
        
        self.robot_detection_sub = self.create_subscription(
            PoseArray, "/robot_detection", self.log_lidar, 10
        )

        # Path logging
        self.path_sub = self.create_subscription(
            Path2D, "/path", self.log_path, 10
        )

        # Grid logging
        self.grid_sub = self.create_subscription(
            Int8MultiArray, "/planning_grid", self.log_planning_grid, 10
        )

        self.bridge = CvBridge()
        self.log_urdf()
        self.log_field()
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

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

        dynamic_joints = []
        for joint in urdf_logger.urdf.joints:
            entity_path = urdf_logger.joint_entity_path(joint)
            if "pt_link" in entity_path.split("/")[-1]:
                dynamic_joints.append((entity_path, joint))
            else:
                urdf_logger.log_joint(entity_path, joint, recording_stream)

        for entity_path, joint in dynamic_joints:
            rr.set_time_nanos("ros_time", nanos=self.get_clock().now().nanoseconds)
            transform = origin_to_transform(joint.origin)
            rr.log(
                entity_path,
                transform,
            )

    def log_field(self):
        field_path = WS_ROOT / "assets" / "field.glb"
        rr.log("/world/field", rr.Asset3D(path=field_path), static=True)

    def log_beacons(self, beacon_poses: PoseArray):
        translations = []
        quaternions = []
        for pose in beacon_poses.poses:
            translations.append([pose.position.x, pose.position.y, pose.position.z])
            quaternions.append(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )
        translations = np.array(translations)
        quaternions = np.array(quaternions)

        for i, (t, q) in enumerate(zip(translations, quaternions)):
            rr.log(
                f"world/beacons/beacon_{i}/plane",
                rr.Boxes3D(sizes=[0.3, 0.28 * 2, 0.05], fill_mode="solid"),
            )
            rr.log(
                f"world/beacons/beacon_{i}/plane",
                rr.Transform3D(translation=[0.0, 0.0, -0.05]),
            )
            rr.log(
                f"world/beacons/beacon_{i}/xyz",
                rr.Arrows3D(
                    vectors=[[0.25, 0, 0], [0, 0.25, 0], [0, 0, 0.25]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                    radii=[0.02, 0.02, 0.02],
                ),
            )
            rr.log(
                f"world/beacons/beacon_{i}",
                rr.Transform3D(translation=t, rotation=rr.Quaternion(xyzw=q)),
            )

            # rr.log(
            #     f"world/beacons/beacon_{i}/xyz",
            #     rr.Transform3D(rotation=rr.Quaternion(xyzw=q)),
            # )

        self.destroy_subscription(self.beacon_sub)

    def log_image(self, image: Image, image_name: str):
        cv_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        time_nanos = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log(f"camera/{image_name}", rr.Image(cv_img, rr.ColorModel.BGR).compress())

    def log_compressed_image(self, compressed_image: CompressedImage, image_name: str):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(
            compressed_image, desired_encoding="bgr8"
        )

        time_nanos = (
            compressed_image.header.stamp.sec * 1_000_000_000
            + compressed_image.header.stamp.nanosec
        )
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log(
            f"camera/compressed/{image_name}",
            rr.Image(cv_img, rr.ColorModel.BGR),
        )

    def log_camera_pose(self, joint_states: JointState):
        timestamp = joint_states.header.stamp
        if timestamp.sec == 0 and timestamp.nanosec == 0:
            rr.set_time_nanos("ros_time", nanos=self.get_clock().now().nanoseconds)
        else:
            time_nanos = timestamp.sec * 1_000_000_000 + timestamp.nanosec
            rr.set_time_nanos("ros_time", time_nanos)

        index = joint_states.name.index("pt_base_link_to_pt_link1")
        angle = rr.Angle(rad=joint_states.position[index])
        rr.log(
            "world/rover/base_footprint/base_link/pt_base_link/pt_link1",
            rr.Transform3D(
                clear=False,
                rotation=rr.RotationAxisAngle((0, 0, 1), angle),
            ),
        )

        index = joint_states.name.index("pt_link1_to_pt_link2")
        rad = joint_states.position[index]
        rr.log(
            "world/rover/base_footprint/base_link/pt_base_link/pt_link1/pt_link2",
            rr.Transform3D(
                clear=False,
                rotation=rr.RotationAxisAngle(axis=(0, -1, 0), angle=rad),
            ),
        )

    def log_robot_pose(self, stamped_pose: PoseStamped):
        time_nanos = (
            stamped_pose.header.stamp.sec * 1_000_000_000
            + stamped_pose.header.stamp.nanosec
        )
        rr.set_time_nanos("ros_time", time_nanos)

        pose = stamped_pose.pose
        point = pose.position

        x = point.y
        y = -point.x
        orientation = pose.orientation

        rr.log(
            "world/rover",
            rr.Transform3D(
                translation=(x, y, 0),
                rotation=rr.Quaternion(
                    xyzw=(orientation.x, orientation.y, orientation.z, orientation.w)
                ),
            ),
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
        distances = []
        class_ids = []
        for tag in apriltags.detections:
            center = (
                np.array((tag.centre.x, tag.centre.y)) / IMAGE_DETECTION_TO_STREAM_SCALE
            )

            corners = (
                np.array([(corner.x, corner.y) for corner in tag.corners])
                / IMAGE_DETECTION_TO_STREAM_SCALE
            )
            lines.append(np.vstack([corners, corners[0]]))

            min_xy = corners.min(axis=0)
            max_xy = corners.max(axis=0)
            half_size = (max_xy - min_xy) / 2.0

            centers.append(center)
            half_sizes.append(half_size)
            distances.append(f"{tag.distance:.3f}")
            class_ids.append(tag.id)

        # Log detected apriltags
        rr.log(
            "apriltags/outlines",
            rr.LineStrips2D(
                lines,
                class_ids=class_ids,
                radii=1.0,
            ),
        )
        rr.log(
            "apriltags/center",
            rr.Points2D(centers, radii=1.5, class_ids=class_ids),
        )
        rr.log(
            "apriltags/distance",
            rr.Points2D(
                centers, labels=distances, class_ids=class_ids, show_labels=True
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

    def log_target_point(self, stamped_point: PointStamped):
        stamp = stamped_point.header.stamp
        rr.set_time_nanos("ros_time", stamp.sec * 1_000_000_000 + stamp.nanosec)

        point = stamped_point.point

        rr.log(
            "world/target_point",
            rr.Ellipsoids3D(
                centers=[[point.y, -point.x, 0]],
                fill_mode=rr.components.FillMode.Solid,
                half_sizes=[[0.075, 0.075, 0.075]],
                colors=[[255, 0, 0]],
            ),
        )

    # def log_lidar(self, laser_scan: LaserScan):
    #     stamp = laser_scan.header.stamp
    #     rr.set_time_nanos("ros_time", stamp.sec * 1_000_000_000 + stamp.nanosec)

    #     range_min = laser_scan.range_min
    #     range_max = laser_scan.range_max

    #     points = [
    #         (scan_range, idx * laser_scan.angle_increment)
    #         for idx, scan_range in enumerate(laser_scan.ranges)
    #         if scan_range < range_max and scan_range > range_min
    #     ]

    #     points_xy = map(
    #         lambda polar_point: (
    #             polar_point[0] * np.cos(polar_point[1]),
    #             polar_point[0] * np.sin(polar_point[1]),
    #             0.0,
    #         ),
    #         points,
    #     )
    #     rr.log("/world/rover/base_footprint/base_link/base_lidar_link/lidar", rr.Points3D(list(points_xy)))


    def log_lidar(self, pose_array: PoseArray):

        stamp = pose_array.header.stamp
        rr.set_time_nanos("ros_time", stamp.sec * 1_000_000_000 + stamp.nanosec)

        # convert to centimeters if needed for visibility
        pts = [(p.position.x, p.position.y, 0.0) for p in pose_array.poses]

        rr.log(
            "/world/rover/base_footprint/base_link/base_lidar_link/lidar",
            rr.Points3D(pts)
        )


    def log_planning_grid(self, msg: Int8MultiArray):
        h = msg.layout.dim[0].size
        w = msg.layout.dim[1].size
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        img = (data * 255).astype(np.uint8)

        rr.set_time_nanos("ros_time", self.get_clock().now().nanoseconds)
        rr.log(
            "world/path_planning/grid",
            rr.Image(img),
        )


    def log_path(self, path: Path2D):
        stamp = path.header.stamp
        rr.set_time_nanos("ros_time", stamp.sec * 1_000_000_000 + stamp.nanosec)

        path_points = [(point.y, -point.x, 0.04) for point in path.poses]

        rr.log(
            "world/path/point",
            rr.Points3D(
                path_points,
            ),
        )


def main(args=None):
    rclpy.init(args=args)
    logger = RerunLogging()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
