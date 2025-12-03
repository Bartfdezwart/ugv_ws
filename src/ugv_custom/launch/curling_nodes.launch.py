from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    april_detector_node = Node(
        package="ugv_vision",
        executable="apriltag_detector",
    )

    april_distance_kalman_node = Node(
        package="ugv_vision",
        executable="apriltag_distance_kalman",
    )

    # april_drive_node = Node(
    #     package="ugv_vision",
    #     executable="apriltag_drive",
    # )

    plan_path_node = Node(
        package="path_planning",
        executable="plan_path"
    )

    rover_detector_node = Node(
        package="ugv_vision",
        executable="rover_detector"
    )

    return LaunchDescription([
        april_detector_node,
        april_distance_kalman_node,
        plan_path_node,
        rover_detector_node,
    ])
