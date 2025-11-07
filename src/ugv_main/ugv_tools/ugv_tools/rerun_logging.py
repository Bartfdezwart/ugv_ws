#!/usr/bin/env python3
import datetime
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs import msg
from ugv_interface.msg import LineArray

import rerun as rr
from ugv_tools.urdf_loader import URDFLogger

# The root is the ugv_ws/
WS_ROOT = Path("~/ugv_ws").expanduser()
# The docker has a slightly different workspace root
if not WS_ROOT.exists():
    WS_ROOT = Path("/home/ws/ugv_ws")


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

        self.lines_subscription = self.create_subscription(
            LineArray, "/linedetect", self.log_detected_lines, 10
        )

        self.top_lines_subscription = self.create_subscription(
            LineArray, "/linedetect_top", self.log_best_lines, 10
        )

        self.raw_image_subscription = self.create_subscription(
            msg.Image,
            "/image_raw",
            self.log_raw_image,
            10,
        )

        self.preprocessed_image_subscription = self.create_subscription(
            msg.Image,
            "/linedetect_preprocessed_img",
            self.log_preprocessed_image,
            10,
        )

        self.bridge = CvBridge()

        # self.frame = 0
        # self.timer = self.create_timer(0.1, self.move)
        # self.log_urdf()

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
        urdf_folder = Path("/home/rick/ugv_ws/src/ugv_main/ugv_description/urdf/")
        rover_urdf = urdf_folder / "ugv_rover.urdf"

        urdf_logger = URDFLogger(rover_urdf, "rover")
        recording_stream = rr.get_global_data_recording()
        urdf_logger.log(recording_stream)

    def log_image(self, image: msg.Image, image_name: str):
        cv_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        time_nanos = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log(f"camera/{image_name}", rr.Image(cv_img, rr.ColorModel.BGR))

    def log_raw_image(self, image: msg.Image):
        cv_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        time_nanos = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log("camera/raw_image", rr.Image(cv_img, rr.ColorModel.BGR))

    def log_preprocessed_image(self, image: msg.Image):
        cv_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        time_nanos = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        rr.set_time_nanos("ros_time", time_nanos)
        rr.log("camera/preprocessed_image", rr.Image(cv_img, rr.ColorModel.BGR))

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

    def log_detected_lines(self, lines: LineArray):
        self.log_lines(lines, "detected_lines", (255, 0, 0))

    def log_best_lines(self, lines: LineArray):
        self.log_lines(lines, "best_lines", (0, 0, 255))

    def log_movement(self):
        rr.set_time_sequence("sequence", self.frame)
        self.frame += 1
        rr.log(
            "rover", rr.Transform3D(clear=False, translation=(self.frame * 0.1, 0, 0))
        )


def main(args=None):
    rclpy.init(args=args)
    logger = RerunLogging()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
