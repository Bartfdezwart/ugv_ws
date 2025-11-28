import argparse
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from scipy.spatial.transform import Rotation


class RoverDetect(Node):
    def __init__(self):
        super().__init__('rover_detector')

        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.rover_detector, 10)
        self.lidar_pub = self.create_publisher(PoseArray, "/robot_detection", 10)
        self.position_sub = self.create_subscription(PoseStamped, "/rover_pose", self.pose_callback, 10)

        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0


    def pose_callback(self, msg: PoseStamped):
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
        pose_array.header.frame_id = "map"  # field frame

        points_xy = []
        for idx, scan_range in enumerate(laser_scan.ranges):
            if range_min < scan_range < range_max:
                theta = laser_scan.angle_min + idx * laser_scan.angle_increment

                # skip lidar points behind the rover (otherwise it detects the camera )
                if not ((np.pi)-np.pi/1.2 <= theta <= (np.pi) + np.pi/1.2):
                    continue
                
                # ignore points too close to the rover.
                if scan_range < 0.15:
                    continue

                x_lidar = scan_range * np.cos(theta)
                y_lidar = scan_range * np.sin(theta)


                if not (-3 <= x_lidar <= 3):
                    continue
                if not (-4.5 <= y_lidar <= 4.5):
                    continue

                pose = Pose()
                pose.position.x = float(x_lidar)
                pose.position.y = float(y_lidar)
                pose.orientation.w = 1.0

                pose_array.poses.append(pose)

        self.lidar_pub.publish(pose_array)


def main(args=None):
    rclpy.init()
    node = RoverDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    