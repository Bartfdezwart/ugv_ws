import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory for the ugv_vision package
    pkg_dir = get_package_share_directory('ugv_vision')
    # Get the path to the params.yaml file
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml') 

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[param_file]
        ),
        Node(
            package='ugv_vision',
            executable='rectify_camera',
            remappings=[
                ('/image', '/image_raw'),
            ]
        ),
    ])