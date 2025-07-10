import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the LaunchDescription for the path_offset_pkg.

    This launch file starts the path_offset_node and loads its parameters
    from a YAML configuration file.
    """

    # Get the path to the config file
    config_dir = os.path.join(get_package_share_directory('gt_path_offset'), 'config')
    config_filepath = os.path.join(config_dir, 'path_offset.yaml')

    # Ensure the config directory exists (for robustness, though `install` should handle it)
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)

    # Define the path_offset_node
    path_offset_node = Node(
        package='gt_path_offset',
        executable='gt_path_offset_node',
        name='gt_path_offset_node',
        output='screen',
        parameters=[config_filepath], # Load parameters from the YAML file
        emulate_tty=True # Required for output='screen' to show logs
    )

    return LaunchDescription([
        path_offset_node
    ])

