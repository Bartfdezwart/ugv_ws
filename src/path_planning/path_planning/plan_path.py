import numpy as np
import rclpy
from rclpy.node import Node
from path_planning.astar import Astar
from std_msgs.msg import Header, Int8MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Pose2D, PoseStamped, PointStamped, PoseArray, Pose, Point
from nav_2d_msgs.msg import Path2D


class PathPlanning(Node):
    def __init__(self):
        super().__init__("path_planning")

        self.scale = 4
        self.grid_size = (np.array((600, 900)) / self.scale).astype(int)
        self.grid = np.zeros(self.grid_size)
        self.start = None
        self.goal = None

        self.robot_detect_sub = self.create_subscription(
            PoseArray, "/robot_detection", self.robot_detection_callback, 10
        )
        self.target_point_sub = self.create_subscription(
            PointStamped, "target_point", self.target_point_callback, 10
        )

        self.robot_pose_sub = self.create_subscription(
            PoseStamped, "/rover_pose", self.robot_pose_callback, 10
        )

        self.path_pub = self.create_publisher(Path2D, "/path", 10)
        self.grid_walls_pub = self.create_publisher(PoseArray, "/grid_walls", 10)

    def robot_pose_callback(self, pose: PoseStamped):
        position = pose.pose.position

        position = np.array([position.x, position.y])
        position += np.array([3.0, 4.5])
        position *= 100
        position /= self.scale

        self.start = (int(position[0]), int(position[1]))

    def target_point_callback(self, stamped_point: PointStamped):
        point = stamped_point.point

        point = np.array([point.x, point.y])
        point += np.array([3.0, 4.5])
        point *= 100
        point /= self.scale

        self.goal = (int(point[0]), int(point[1]))
        self.plan_path()

    def publish_grid(self):
        msg = PoseArray(header=Header(stamp=self.get_clock().now().to_msg()))

        # msg.layout = MultiArrayLayout(
        #     dim=[
        #         MultiArrayDimension(
        #             label="height", size=self.grid.shape[0], stride=self.grid.size
        #         ),
        #         MultiArrayDimension(
        #             label="width", size=self.grid.shape[1], stride=self.grid.shape[1]
        #         ),
        #     ],
        #     data_offset=0,
        # )

        wall_coords = np.argwhere(self.grid == 1)

        if wall_coords.size > 0:
            wall_coords = wall_coords.astype(float) / 100
            wall_coords *= self.scale
            wall_coords -= np.array([3.0, 4.5])

            msg.poses = [Pose(position=Point(x=coord[0], y=coord[1])) for coord in wall_coords]
        self.grid_walls_pub.publish(msg)

    def robot_detection_callback(self, pose_array: PoseArray):
        self.grid.fill(0)

        inflation = 6

        for pose in pose_array.poses:
            x_field = pose.position.x
            y_field = pose.position.y

            # convert field coords → grid (col = x, row = y)
            col = int(((x_field + 3.0) * 100) / self.scale)
            row = int(((y_field + 4.5) * 100) / self.scale)

            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    ix = col + dx  # column index (x)
                    iy = row + dy  # row index (y)

                    # bounds: grid[row, col]
                    if 0 <= ix < self.grid.shape[0] and 0 <= iy < self.grid.shape[1]:
                        self.grid[ix, iy] = 1

        self.publish_grid()
        self.plan_path()

    def plan_path(self):
        if self.start is None or self.goal is None:
            return

        self.get_logger().info(
            f"Planning path from {self.start} to {self.goal} (grid {self.grid.shape})"
        )

        planning_model = Astar(self.grid, self.start, self.goal)
        path = planning_model.find_path()

        if path is None:
            self.get_logger().warning("Failed to find a path")
            return

        header = Header(stamp=self.get_clock().now().to_msg())

        path_points = [
            Pose2D(
                x=float(point[0] / 100 * self.scale) - 3.0,
                y=float(point[1] / 100 * self.scale) - 4.5,
            )
            for point in path
        ]

        self.publish_grid()
        self.path_pub.publish(Path2D(header=header, poses=path_points))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
