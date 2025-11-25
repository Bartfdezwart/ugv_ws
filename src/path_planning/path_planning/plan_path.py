import numpy as np
import rclpy
from rclpy.node import Node
from path_planning.astar import Astar
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Path2D, Pose2DStamped


class PathPlanning(Node):
    def __init__(self):
        super().__init__("path_planning")

        self.grid_size = (300, 450)
        self.grid = np.zeros(self.grid_size)
        self.start = None
        self.goal = None
        self.planning_algorithm = Astar

        self.robot_detect_sub = self.create_subscription(
            Pose2DStamped, "/robot_detection", self.robot_detection_callback
        )

        self.path_pub = self.create_publisher(Path2D, "/path", 10)

    def robot_detection_callback(self, pose: Pose2DStamped) -> None:
        # Update grid
        ...

        self.plan_path()

    def plan_path(self):
        planning_model = Astar(self.grid, self.start, self.goal)
        path = planning_model.find_path()

        if path is None:
            self.get_logger().warning("Failed to find a path")
            return

        # Publish the path
        header = Header(time=self.get_clock().now().to_msg())
        path_points = [Pose2D(x=point[0], y=point[1]) for point in path]
        self.path_pub.publish(Path2D(header=header, poses=path_points))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
