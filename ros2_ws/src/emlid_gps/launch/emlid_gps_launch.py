# emlid_gps/launch/emlid_gps_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    # Get the path to the config file
    config_dir = os.path.join(get_package_share_directory('emlid_gps'), 'config')
    param_file = os.path.join(config_dir, 'emlid_params.yaml')

    log_dir = '/root/ros2_ws/src/emlid_gps/log_emlid'

    def remove_directory_contents_and_self(path):
        # Iterate through the directory tree in reverse order (bottom-up)
        # This ensures that files are deleted before their containing directories
        # and subdirectories are deleted before their parent directories.
        for dirpath, dirnames, filenames in os.walk(path, topdown=False): #
            for filename in filenames:
                file_path = os.path.join(dirpath, filename)
                try:
                    os.remove(file_path)  # Delete the file
                    print(f"Deleted file: {file_path}")
                except OSError as e:
                    print(f"Error deleting file {file_path}: {e}")
            
            for dirname in dirnames:
                dir_to_remove = os.path.join(dirpath, dirname)
                try:
                    os.rmdir(dir_to_remove) # Delete the empty directory
                    print(f"Deleted directory: {dir_to_remove}")
                except OSError as e:
                    print(f"Error deleting directory {dir_to_remove}: {e}")
        
        # Finally, remove the top-level directory itself
        try:
            os.rmdir(path)
            print(f"Deleted top-level directory: {path}")
        except OSError as e:
            print(f"Error deleting top-level directory {path}: {e}")


    if os.path.exists(log_dir):
        remove_directory_contents_and_self(log_dir)

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_LOG_DIR', value=log_dir),
        Node(
            package='emlid_gps',
            executable='emlid_gps',
            name='emlid_gps', # Make sure this name matches the one in your YAML file!
            output='screen',
            parameters=[param_file] # Load parameters from the YAML file
        )
    ])