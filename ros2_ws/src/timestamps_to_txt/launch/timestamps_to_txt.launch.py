import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'timestamps_to_txt'
    params_file = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable=package_name,
            name=package_name,
            output='screen',
            parameters=[params_file],
        )
    ])