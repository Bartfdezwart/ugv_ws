import numpy as np
import rclpy
from rclpy.node import Node
from path_planning.astar import Astar
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, PoseStamped, PointStamped
from nav_2d_msgs.msg import Path2D, Pose2DStamped


class PathPlanning(Node):
    def __init__(self):
        super().__init__("path_planning")

        self.scale = 4
        self.grid_size = (np.array((600, 900)) / self.scale).astype(int)
        self.grid = np.zeros(self.grid_size)
        self.start = None
        self.goal = None

        self.robot_detect_sub = self.create_subscription(
            Pose2DStamped, "/robot_detection", self.robot_detection_callback, 10
        )
        self.target_point_sub = self.create_subscription(
            PointStamped, "target_point", self.target_point_callback, 10
        )

        self.robot_pose_sub = self.create_subscription(
            PoseStamped, "/rover_pose", self.robot_pose_callback, 10
        )

        self.path_pub = self.create_publisher(Path2D, "/path", 10)

    def robot_pose_callback(self, pose: PoseStamped):
        position = pose.pose.position

        position = np.array([position.x, position.y])
        # shift position
        position += np.array([3.0, 4.5])
        # Meter to centimeter conversion
        position *= 100
        # downscale to grid size
        position /= self.scale
        self.start = (int(position[0]), int(position[1]))

    def target_point_callback(self, stamped_point: PointStamped):
        point = stamped_point.point

        point = np.array([point.x, point.y])
        # shift position
        point += np.array([3.0, 4.5])
        # Meter to centimeter conversion
        point *= 100
        # downscale to grid size
        point /= self.scale

        # Update goal point
        self.goal = (int(point[0]), int(point[1]))
        # Recompute path
        self.plan_path()

    def robot_detection_callback(self, pose: Pose2DStamped) -> None:
        # Update grid

        self.plan_path()

    def plan_path(self):
        self.get_logger().info(f"Planning path from {self.start} to {self.goal} (grid {self.grid.shape})")
        planning_model = Astar(self.grid, self.start, self.goal)
        path = planning_model.find_path()

        if path is None:
            self.get_logger().warning("Failed to find a path")
            return

        # Publish the path
        header = Header(stamp=self.get_clock().now().to_msg())
        path_points = [
            Pose2D(x=float(point[0] / 100 * self.scale) - 3.0, y=float(point[1] / 100 * self.scale) - 4.5) for point in path
        ]
        self.path_pub.publish(Path2D(header=header, poses=path_points))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
