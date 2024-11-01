#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if the current directory is the same as the script directory
if [ "$(pwd)" != "$SCRIPT_DIR" ]; then
    echo "Error: This script must be run in the directory: $SCRIPT_DIR"
    exit 1
fi

# Create the log_ouster directory if it doesn't exist
LOG_DIR="$SCRIPT_DIR/log_ouster"
if [ ! -d "$LOG_DIR" ]; then
    mkdir "$LOG_DIR"
    echo "Created log directory: $LOG_DIR"
fi

# Variables common to all sensors
    # Retrieve the primary link-local IP address in the 169.254.x.x range 
    # *** This is the IP address of the host machine. DOUBLE CHECK! ***
# Retrieve the primary link-local IP address in the 169.254.x.x range
# udp_address=$(ip addr show scope link | grep -oP 'inet 169\.254\.\d+\.\d+' | awk '{print $2}')
UDP_ADDRESS=169.254.219.81
echo "UDP address: $UDP_ADDRESS"

USE_SYSTEM_DEFAULT_QOS=true

METADATA_FILE_PATH="$SCRIPT_DIR/log_ouster/metadata"
if [ ! -d "$METADATA_FILE_PATH" ]; then
    mkdir "$METADATA_FILE_PATH"
    echo "Created metadata directory: $METADATA_FILE_PATH"
fi

# Launch the sensors and redirect outputs to log files
# Top # Set min range to 2.0 to exclude the top of the cart
ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=${SENSOR_HOSTNAME_TOP} \
    udp_dest:=${UDP_ADDRESS} \
    use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
    metadata:="$METADATA_FILE_PATH/top_lidar.json" \
    viz:=true \
    ouster_ns:="ouster_top" \
    sensor_frame:="os_sensor_top" \
    lidar_frame:="os_lidar_top" \
    imu_frame:="os_imu_top" \
    min_range:="0.0" \
    > "$LOG_DIR/ouster_top.log" 2>&1 &

# # Front
# ros2 launch ouster_ros sensor.launch.xml \
#     sensor_hostname:=${SENSOR_HOSTNAME_FRONT} \
#     udp_dest:=$UDP_ADDRESS \
#     use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
#     metadata:="$METADATA_FILE_PATH/front_lidar.json" \
#     viz:=false \
#     ouster_ns:="ouster_front" \
#     azimuth_window_start:="45000" \
#     azimuth_window_end:="225000" \
#     > "$LOG_DIR/ouster_front.log" 2>&1 &

# # Right
# ros2 launch ouster_ros sensor.launch.xml \
#     sensor_hostname:=${SENSOR_HOSTNAME_RIGHT} \
#     udp_dest:=$UDP_ADDRESS \
#     use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
#     metadata:="$METADATA_FILE_PATH/right_lidar.json" \
#     viz:=false \
#     ouster_ns:="ouster_right" \
#     azimuth_window_start:="135000" \
#     azimuth_window_end:="315000" \
#     > "$LOG_DIR/ouster_right.log" 2>&1 &

# # Left
# ros2 launch ouster_ros sensor.launch.xml \
#     sensor_hostname:=${SENSOR_HOSTNAME_LEFT} \
#     udp_dest:=$UDP_ADDRESS \
#     use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
#     metadata:="$METADATA_FILE_PATH/left_lidar.json" \
#     viz:=false \
#     ouster_ns:="ouster_left" \
#     azimuth_window_start:="45000" \
#     azimuth_window_end:="225000" \
#     > "$LOG_DIR/ouster_left.log" 2>&1 &

# Back
ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=${SENSOR_HOSTNAME_BACK} \
    udp_dest:=$UDP_ADDRESS \
    use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
    metadata:="$METADATA_FILE_PATH/back_lidar.json" \
    viz:=false \
    ouster_ns:="ouster_back" \
    sensor_frame:="os_sensor_back" \
    lidar_frame:="os_lidar_back" \
    imu_frame:="os_imu_back" \
    azimuth_window_start:="135000" \
    azimuth_window_end:="315000" \
    > "$LOG_DIR/ouster_back.log" 2>&1 &

echo "Sensors launched. Output is in $LOG_DIR/"
echo "Press Ctrl+C to stop the sensors"

wait
