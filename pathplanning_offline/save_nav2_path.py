import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np
import math

class PathSaver(Node):
    def __init__(self):
        super().__init__('path_saver')
        self.sub = self.create_subscription(
            Path,
            '/plan',
            self.callback,
            1
        )

    def callback(self, msg: Path):
        pts = []

        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y

            q = p.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z),
                1.0 - 2.0 * (q.z * q.z)
            )

            pts.append([x, y, yaw])

        pts = np.array(pts)

        np.savetxt(
            "nav2_path.csv",
            pts,
            delimiter=",",
            header="x,y,yaw",
            comments=""
        )

        self.get_logger().info(
            f"Saved nav2_path.csv with {len(pts)} points"
        )

        rclpy.shutdown()


def main():
    rclpy.init()
    node = PathSaver()
    rclpy.spin(node)

if __name__ == "__main__":
    main()