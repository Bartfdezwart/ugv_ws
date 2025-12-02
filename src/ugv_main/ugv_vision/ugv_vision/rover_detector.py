import argparse
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from scipy.spatial.transform import Rotation
import math


class RoverDetect(Node):
    def __init__(self):
        super().__init__('rover_detector')

        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.rover_detector, 10)
        self.lidar_pub = self.create_publisher(PoseArray, "/robot_detection", 10)
        self.position_sub = self.create_subscription(PoseStamped, "/rover_pose", self.pose_callback, 10)

        # World pose of the rover (field frame)
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0


    def pose_callback(self, msg: PoseStamped):
        """Store rover x,y,yaw in WORLD frame."""
        self.rover_x = msg.pose.position.x
        self.rover_y = msg.pose.position.y

        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        self.rover_yaw = Rotation.from_quat(quat).as_euler('xyz')[2]


    def rover_detector(self, laser_scan: LaserScan):
        range_min = laser_scan.range_min
        range_max = laser_scan.range_max

        pose_array = PoseArray()
        pose_array.header.stamp = laser_scan.header.stamp
        pose_array.header.frame_id = "map"

        cos_yaw = math.cos(self.rover_yaw)
        sin_yaw = math.sin(self.rover_yaw)
        print(self.rover_yaw)

        for idx, scan_range in enumerate(laser_scan.ranges):

            if not (range_min < scan_range < range_max):
                continue

            theta = laser_scan.angle_min + idx * laser_scan.angle_increment

            if scan_range < 0.15:
                continue


            x_lidar = scan_range * math.cos(theta + self.rover_yaw + np.pi)
            y_lidar = scan_range * math.sin(theta + self.rover_yaw + np.pi)


            x_world = self.rover_x + x_lidar
            y_world = self.rover_y + y_lidar

            # Check field bounds
            if not (-3.0 <= x_world <= 3.0):
                continue
            if not (-4.5 <= y_world <= 4.5):
                continue

            pose = Pose()
            pose.position.x = float(x_world)
            pose.position.y = float(y_world)
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

        # Send WORLD-frame obstacles
        self.lidar_pub.publish(pose_array)


def main(args=None):
    rclpy.init()
    node = RoverDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()