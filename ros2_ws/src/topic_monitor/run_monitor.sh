#!/bin/bash

# Source the ROS 2 setup file (adjust path if different)
# This assumes you have your ROS 2 environment sourced in your current shell
# If not, you might need: source /opt/ros/humble/setup.bash (or your ROS distro)
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "Sourcing ROS 2 setup from /opt/ros/$ROS_DISTRO/setup.bash"
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "Warning: Could not find ROS 2 setup file. Please ensure ROS 2 is sourced."
fi

# Build the package
echo "Building topic_monitor package..."
colcon build --packages-select topic_monitor --symlink-install

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "Error: colcon build failed."
    exit 1
fi

# Source the workspace setup file
echo "Sourcing workspace setup..."
source install/setup.bash

# Run the topic_monitor_node
echo "Running topic_monitor_node for 5 seconds..."
ros2 run topic_monitor topic_monitor_node --ros-args -p topics_file:=topics.yaml

echo "Script finished."