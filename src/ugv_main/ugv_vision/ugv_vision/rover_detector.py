import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from nav_2d_msgs.msg import Path2D, Pose2DStamped
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation


class RoverDetect(Node):
    def __init__(self):
        super().__init__('rover_detector')

        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.rover_detector, 10)
        self.lidar_pub = self.create_publisher(Pose2DStamped, "/robot_detection", 10)
        self.position_sub = self.create_subscription(PoseStamped,"/rover_pose", self.pose_callback,10)

        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0


    def pose_callback(self, msg: PoseStamped):
        self.rover_x = msg.pose.position.x
        self.rover_y = msg.pose.position.y

        quat = msg.pose.orientation
        _, _, self.rover_yaw = Rotation.from_quat(quat).as_euler(seq='xyz')


    def rover_detector(self, laser_scan: LaserScan):
        range_min = laser_scan.range_min
        range_max = laser_scan.range_max

        points = []
        for idx, r in enumerate(laser_scan.ranges):
            if not (range_min < r < range_max):
                continue

            angle = idx * laser_scan.angle_increment + laser_scan.angle_min
            angle_world = angle + np.pi / 2
            bearing = angle_world - self.rover_yaw
            bearing = np.arctan2(np.sin(bearing), np.cos(bearing))

            # 120 degrees in front of rover
            if not (-np.pi/3 <= bearing <= np.pi/3):
                continue

            points.append((r, angle))

        points_xy = [(r * np.cos(a), r * np.sin(a)) for r, a in points]

        cos_y = np.cos(self.rover_yaw)
        sin_y = np.sin(self.rover_yaw)

        world_points = []
        for px, py in points_xy:
            world_x = self.rover_x + (-sin_y * px - cos_y * py)
            world_y = self.rover_y + ( cos_y * px - sin_y * py)

            # assume no points within 10 cm of the edges of the field
            if -2.9 <= world_x <= 2.9 and -4.4 <= world_y <= 4.4:
                world_points.append((world_x, world_y))

        clusters = self.cluster_points(world_points, threshold=0.30)

        for cluster in clusters:

            # skip sparse clusters.
            if(len(cluster) < 3):
                continue

            cx = float(np.mean([p[0] for p in cluster]))
            cy = float(np.mean([p[1] for p in cluster]))

            msg = Pose2DStamped()
            msg.header.stamp = laser_scan.header.stamp
            msg.pose.x = cx
            msg.pose.y = cy
            msg.pose.theta = 0.0
            self.lidar_pub.publish(msg)

            print(cx, cy)



    def cluster_points(self, points, threshold=0.20):
        clusters = []

        for p in points:
            added = False

            for c in clusters:
                if any(np.linalg.norm(np.array(p) - np.array(q)) < threshold for q in c):
                    c.append(p)
                    added = True
                    break
            
            if not added:
                clusters.append([p])
        
        return clusters


def main(args=None):
    rclpy.init()
    node = RoverDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




# import argparse

# import cv2
# import numpy as np
# import rclpy
# from cv_bridge import CvBridge
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
# from ugv_interface.msg import AprilTag, AprilTagArray, Point
# from nav_2d_msgs.msg import Path2D, Pose2DStamped
# from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


# class RoverDetect(Node):
#     def __init__(self):
#         super().__init__('rover_detector')
#         self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.rover_detector, 10)
#         self.lidar_pub = self.create_publisher(Pose2DStamped, "/robot_detection", self.rover_detector, 10)
#         self.position_pub = self.create_subscription(PoseStamped, '/rover_pose', 10)


#     def rover_detector(self, laser_scan: LaserScan):
#         stamp = laser_scan.header.stamp

#         range_min = laser_scan.range_min
#         range_max = laser_scan.range_max

#         points = [
#             (scan_range, idx * laser_scan.angle_increment)
#             for idx, scan_range in enumerate(laser_scan.ranges)
#             if scan_range < range_max and scan_range > range_min
#         ]

#         points_xy = map(
#             lambda polar_point: (
#                 polar_point[0] * np.cos(polar_point[1]),
#                 polar_point[0] * np.sin(polar_point[1]),
#                 0.0,
#             ),
#             points,
#         )

        

# def main(args=None):
#     parser = argparse.ArgumentParser()
#     rclpy.init(args=args)
#     apriltag_ctrl = RoverDetect()
#     rclpy.spin(apriltag_ctrl)
#     apriltag_ctrl.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



