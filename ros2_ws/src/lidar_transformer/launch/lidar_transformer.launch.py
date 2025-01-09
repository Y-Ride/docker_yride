## Launch file without visualization
# from launch import LaunchDescription
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_share = get_package_share_directory('lidar_transformer')
#     parameter_file = os.path.join(pkg_share, "config", "params.yaml")

#     return LaunchDescription([
#         Node(
#             package='lidar_transformer',
#             executable='lidar_transformer_node',
#             name='lidar_transformer_node',
#             output='screen',
#             parameters=[
#                 parameter_file,
#             ]
#         )
#     ])


## Launch file with RViz2 visualization
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_transformer')
    parameter_file = os.path.join(pkg_share, "config", "params.yaml")
    rviz_config_file = os.path.join(pkg_share, "config", "rviz_config.rviz")

    # Declare a launch argument for 'viz'
    viz_arg = DeclareLaunchArgument(
        'viz',
        default_value='false',
        description="Set to 'true' to launch RViz2"
    )

    # RViz2 node, conditional on the 'viz' argument
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('viz'))  # Launch if 'viz' is true
    )

    # Main lidar transformer node
    lidar_node = Node(
        package='lidar_transformer',
        executable='lidar_transformer_node',
        name='lidar_transformer_node',
        output='screen',
        parameters=[parameter_file]
    )

    return LaunchDescription([
        viz_arg,  # Declare the argument
        lidar_node,
        rviz_node  # Conditionally include RViz2
    ])
