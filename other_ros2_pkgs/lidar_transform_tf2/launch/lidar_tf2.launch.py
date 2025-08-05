from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_top',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'yride_lidar', '--child-frame-id', 'os_lidar_top'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_right',
            arguments=[
                '--x', '-0.088900', '--y', '0.730250', '--z', '-0.241300',
                '--qx', '0.154430', '--qy', '0.372827', '--qz', '0.845313', '--qw', '-0.350140',
                '--frame-id', 'yride_lidar', '--child-frame-id', 'os_lidar_right'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_left',
            arguments=[
                '--x', '-0.088900', '--y', '-0.730250', '--z', '-0.241300',
                '--qx', '0.158392', '--qy', '-0.382393', '--qz', '0.841029', '--qw', '0.348365',
                '--frame-id', 'yride_lidar', '--child-frame-id', 'os_lidar_left'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_front',
            arguments=[
                '--x', '-1.752600', '--y', '0.000000', '--z', '-0.203200',
                '--qx', '-0.123326', '--qy', '-0.297735', '--qz', '0.362267', '--qw', '0.874590',
                '--frame-id', 'yride_lidar', '--child-frame-id', 'os_lidar_front'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_back',
            arguments=[
                '--x', '1.752600', '--y', '0.000000', '--z', '-0.203200',
                '--qx', '0.235222', '--qy', '0.097432', '--qz', '0.893434', '--qw', '0.370072',
                '--frame-id', 'yride_lidar', '--child-frame-id', 'os_lidar_back'
            ]
        ),
        Node(
            package='lidar_transform_tf2',
            executable='lidar_transform_tf2_node',
            name='lidar_transform_tf2_node'
        ),
    ])