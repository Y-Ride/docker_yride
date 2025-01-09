#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if the current directory is the same as the script directory
if [ "$(pwd)" != "$SCRIPT_DIR" ]; then
    echo "Error: This script must be run in the directory: $SCRIPT_DIR"
    exit 1
fi

# Define flags
viz_flag=false

# Get flags
while getopts "v" opt; do
    case "$opt" in
        v) viz_flag=true ;;
        ?) echo "Usage: $0 [-v]" ; exit 1 ;;
    esac
done

## NOTE: The xhost is not available in the container
# # Check if the xhost command is available
# if ! command -v xhost &> /dev/null; then
#     # Allow xhost access to the container
#     xhost +local:docker

#     # Define cleanup function
#     cleanup() {
#         echo "Cleaning up..."
#         xhost -local:docker
#     }

#     # Set up trap to call cleanup on exit or interrupt
#     trap cleanup EXIT
# fi

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
UDP_ADDRESS=169.254.0.1
echo "UDP address: $UDP_ADDRESS"

USE_SYSTEM_DEFAULT_QOS=true
SET_LIDAR_PERSIST=false

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
    viz:=$viz_flag \
    ouster_ns:="ouster_top" \
    sensor_frame:="os_sensor_top" \
    lidar_frame:="os_lidar_top" \
    imu_frame:="os_imu_top" \
    min_range:="2.0" \
    persist_config:=${SET_LIDAR_PERSIST} \
    > "$LOG_DIR/ouster_top.log" 2>&1 &

# Front
ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=${SENSOR_HOSTNAME_FRONT} \
    udp_dest:=${UDP_ADDRESS} \
    use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
    metadata:="$METADATA_FILE_PATH/front_lidar.json" \
    viz:=false \
    ouster_ns:="ouster_front" \
    sensor_frame:="os_sensor_front" \
    lidar_frame:="os_lidar_front" \
    imu_frame:="os_imu_front" \
    azimuth_window_start:="45000" \
    azimuth_window_end:="225000" \
    persist_config:=${SET_LIDAR_PERSIST} \
    > "$LOG_DIR/ouster_front.log" 2>&1 &

# Right
ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=${SENSOR_HOSTNAME_RIGHT} \
    udp_dest:=${UDP_ADDRESS} \
    use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
    metadata:="$METADATA_FILE_PATH/right_lidar.json" \
    viz:=false \
    ouster_ns:="ouster_right" \
    sensor_frame:="os_sensor_right" \
    lidar_frame:="os_lidar_right" \
    imu_frame:="os_imu_right" \
    azimuth_window_start:="135000" \
    azimuth_window_end:="315000" \
    persist_config:=${SET_LIDAR_PERSIST} \
    > "$LOG_DIR/ouster_right.log" 2>&1 &

# Left
ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=${SENSOR_HOSTNAME_LEFT} \
    udp_dest:=${UDP_ADDRESS} \
    use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
    metadata:="$METADATA_FILE_PATH/left_lidar.json" \
    viz:=false \
    ouster_ns:="ouster_left" \
    sensor_frame:="os_sensor_left" \
    lidar_frame:="os_lidar_left" \
    imu_frame:="os_imu_left" \
    azimuth_window_start:="45000" \
    azimuth_window_end:="225000" \
    persist_config:=${SET_LIDAR_PERSIST} \
    > "$LOG_DIR/ouster_left.log" 2>&1 &

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
    persist_config:=${SET_LIDAR_PERSIST} \
    > "$LOG_DIR/ouster_back.log" 2>&1 &

echo "Sensors launched. Output is in $LOG_DIR/"

## NOTE: No point to source since script occupies terminal until sensors are stopped
# # Source the bag_alias.sh script to use the record_bag function
# if [ -f "$SCRIPT_DIR/bags/bag_alias.sh" ]; then
#     echo "Sourcing bags/bag_alias.sh"
#     source "$SCRIPT_DIR/bags/bag_alias.sh"
# else
#     echo "Error: bags/bag_alias.sh not found."
# fi

echo "Press Ctrl+C to stop the sensors"

wait
