#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ugv_interface.msg import LineArray
import argparse

import pyelsed


class DetectLinesSelect(Node):
    def __init__(self, detector_type="custom"):
        super().__init__('detect_lines_select')
        self.get_logger().info(f'UGV LineDetection node started. Detector: {detector_type}')
        self.detector_type = detector_type.lower()

        self._declare_parameters()

        self.image_sub = self.create_subscription(Image, '/image_rect', self.image_callback, 10)
        self.line_pub = self.create_publisher(LineArray, '/linedetect', 10)
        self.top_line_pub = self.create_publisher(LineArray, '/linedetect_top', 10)
        self.preprocessed_img = self.create_publisher(Image, '/linedetect_preprocessed_img', 10)

        self.bridge = CvBridge()

        self.best_line = None


    def _declare_parameters(self):
        self.declare_parameter("gaussian_blur", 3)
        self.declare_parameter("binary_threshold_lower", 220)
        self.declare_parameter("binary_threshold_upper", 255)
        self.declare_parameter("opening_kernelsize", 3)

        # Canny parameters
        self.declare_parameter("canny_threshold1", 160)
        self.declare_parameter("canny_threshold2", 225)

        # Hough parameters
        self.declare_parameter("hough_rho", 1)
        self.declare_parameter("hough_theta", np.pi/180)
        self.declare_parameter("hough_threshold", 50)
        self.declare_parameter("hough_min_line_length", 80)
        self.declare_parameter("hough_max_line_gap", 10)

    def image_callback(self, msg):
        try:
            self.gaussian_blur = self.get_parameter("gaussian_blur").value
            self.binary_threshold_lower = self.get_parameter("binary_threshold_lower").value
            self.binary_threshold_upper = self.get_parameter("binary_threshold_upper").value
            self.opening_kernelsize = self.get_parameter("opening_kernelsize").value
            
            self.canny_threshold1 = self.get_parameter("canny_threshold1").value
            self.canny_threshold2 = self.get_parameter("canny_threshold2").value
            
            self.hough_rho = self.get_parameter("hough_rho").value
            self.hough_theta = self.get_parameter("hough_theta").value
            self.hough_threshold = self.get_parameter("hough_threshold").value
            self.hough_min_line_length = self.get_parameter("hough_min_line_length").value
            self.hough_max_line_gap = self.get_parameter("hough_max_line_gap").value

            # preprocessing 
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_blur = cv2.GaussianBlur(gray, (self.gaussian_blur, self.gaussian_blur), 0) 
            gray = cv2.equalizeHist(img_blur)  
            _, thresh = cv2.threshold(gray, self.binary_threshold_lower, self.binary_threshold_upper, cv2.THRESH_BINARY)
            kernel = np.ones((self.opening_kernelsize, self.opening_kernelsize), np.uint8)
            dilated = cv2.dilate(thresh, kernel, iterations=2)

            # publish preprocessed image
            preprocessed_msg = self.bridge.cv2_to_imgmsg(dilated, encoding='mono8', header=msg.header)
            self.preprocessed_img.publish(preprocessed_msg)

            # select canny or custom line detection.
            if self.detector_type == "canny":
                edges = cv2.Canny(dilated, self.canny_threshold1, self.canny_threshold2)
                lines = cv2.HoughLinesP(
                    edges,
                    rho=self.hough_rho,
                    theta=self.hough_theta,
                    threshold=self.hough_threshold,
                    minLineLength=self.hough_min_line_length,
                    maxLineGap=self.hough_max_line_gap
                )
                if lines is not None:
                    lines = [l[0] for l in lines]  # flatten shape (N,1,4) to (N,4)
                else:
                    lines = []
            else:
                lines, _ = pyelsed.detect(dilated)
            # ---------------------------------------------

            # filter out short lines and merge long lines.
            filtered_lines = self.filter_lines(lines)
            merged_lines = self.merge_lines(filtered_lines)

            # prepare message
            line_msg = LineArray()
            data = []
            for line in merged_lines:
                x1, y1, x2, y2 = line[0], line[1], line[2], line[3]
                data.extend([float(x1), float(y1), float(x2), float(y2)])
            line_msg.data = data
            line_msg.header = msg.header
            self.line_pub.publish(line_msg)

            # select top line.
            h, w = dilated.shape
            img_center = np.array([w / 2, h / 2])
            best_line, min_dist = self.best_line, 1e9

            for l in merged_lines:
                x1, y1, x2, y2 = l
                c = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
                d = np.linalg.norm(c - img_center)
                if d < min_dist:
                    best_line, min_dist = l, d

            if best_line is not None:
                new_angle = np.degrees(np.arctan2(best_line[3] - best_line[1],
                                                  best_line[2] - best_line[0]))

                if hasattr(self, "prev_top_angle") and self.prev_top_angle is not None:
                    angle_diff = abs(new_angle - self.prev_top_angle)
                    angle_diff = min(angle_diff, 180 - angle_diff)
                    if angle_diff > 20:
                        best_line = self.best_line
                    else:
                        self.prev_top_angle = new_angle
                else:
                    self.prev_top_angle = new_angle

            if best_line is not None:
                self.best_line = best_line
                msg_top = LineArray()
                msg_top.data = list(map(float, best_line))
                msg_top.header = msg.header
                self.top_line_pub.publish(msg_top)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")


    # remove lines under median length
    def filter_lines(self, lines):
        lines = np.asarray(lines, dtype=np.float32)
        lengths = np.array([self._line_length(l) for l in lines])
        cutoff = np.median(lengths)
        filtered = lines[lengths >= cutoff]
        return filtered.tolist()


    def merge_lines(self, lines, angle_thresh=np.deg2rad(8), dist_thresh=15):
        lines = np.asarray(lines, dtype=np.float32)

        changed = True
        while changed:
            changed = False
            merged, used = [], np.zeros(len(lines), dtype=bool)

            angles = np.array([self._line_angle(l) for l in lines])
            dirs   = np.array([self._line_direction(l) for l in lines])
            centers = np.array([self._line_center(l) for l in lines])

            for i in range(len(lines)):
                if used[i]:
                    continue

                group, angle_i, dir_i = [i], angles[i], dirs[i]

                for j in range(i + 1, len(lines)):
                    if used[j]:
                        continue

                    angle_diff = abs(angles[j] - angle_i)
                    angle_diff = min(angle_diff, np.pi - angle_diff)
                    if angle_diff > angle_thresh:
                        continue

                    dist_center = centers[j] - centers[i]
                    perp_dist = abs(dist_center[0] * (-dir_i[1]) + dist_center[1] * dir_i[0])
                    if perp_dist < dist_thresh:
                        group.append(j)
                        used[j] = True
                        changed = True

                points = np.vstack((lines[group, :2], lines[group, 2:]))
                vx, vy, x, y = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()

                proj = (points - np.array([x, y])) @ np.array([vx, vy])
                pmin, pmax = proj.min(), proj.max()
                start = np.array([x, y]) + np.array([vx, vy]) * pmin
                end   = np.array([x, y]) + np.array([vx, vy]) * pmax

                merged.append([start[0], start[1], end[0], end[1]])
                used[i] = True

            lines = np.array(merged, np.float32)

        return lines


    def _line_length(self, line):
        return np.hypot(line[2] - line[0], line[3] - line[1])

    def _line_center(self, line):
        return np.array([(line[0] + line[2]) / 2, (line[1] + line[3]) / 2], dtype=np.float32)

    def _line_angle(self, line):
        return np.arctan2(line[3] - line[1], line[2] - line[0])

    def _line_direction(self, line):
        dx, dy = line[2] - line[0], line[3] - line[1]
        length = np.hypot(dx, dy) + 1e-6
        return np.array([dx / length, dy / length], dtype=np.float32)



def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--linedetect", type=str, default="custom",
                        choices=["custom", "canny"],
                        help="choose  'custom' (pyelsed) or 'canny' ")
    
    parsed_args, remaining_args = parser.parse_known_args()

    rclpy.init(args=remaining_args)

    detector_type = parsed_args.linedetect

    node = DetectLinesSelect(detector_type)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
