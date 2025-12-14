import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class CostmapSaver(Node):
    def __init__(self):
        super().__init__('costmap_saver')
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.cb,
            1)

    def cb(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1] = 127

        known = data >= 0
        img[known] = 255 - (data[known] * 255 // 100)

        cv2.imwrite("nav2_costmap.png", img)
        self.get_logger().info("Saved nav2_costmap.png with inflation preserved")
        rclpy.shutdown()

rclpy.init()
node = CostmapSaver()
rclpy.spin(node)