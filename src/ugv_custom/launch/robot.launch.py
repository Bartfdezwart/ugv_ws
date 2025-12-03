import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_node = Node(
        package="ugv_bringup",
        executable="ugv_bringup",
    )

    driver_node = Node(
        package="ugv_bringup",
        executable="ugv_driver",
    )

    camera_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ugv_vision"), "launch", "camera.launch.py"
            )
        )
    )

    # Include laser lidar launch file
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ldlidar"), "launch", "ldlidar.launch.py"
            )
        )
    )

    return LaunchDescription([
        bringup_node,
        driver_node,
        camera_bringup_launch,
        laser_bringup_launch
    ])
