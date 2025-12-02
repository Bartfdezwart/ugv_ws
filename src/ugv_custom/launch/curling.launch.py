import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ugv_custom"), "launch", "robot.launch.py"
            )
        )
    )

    curling_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ugv_custom"),
                "launch",
                "curling_nodes.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            robot_bringup_launch,
            curling_nodes_launch,
        ]
    )
