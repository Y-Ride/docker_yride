#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if the current directory is the same as the script directory
if [ "$(pwd)" != "$SCRIPT_DIR" ]; then
    # echo "Error: This script must be run in the directory: $SCRIPT_DIR"
    # exit 1
    echo "Changing to script's directory: $SCRIPT_DIR"
    cd "$SCRIPT_DIR"
fi

# Define flags
viz_flag=false
top_flag=false
right_flag=false
left_flag=false
front_flag=false
back_flag=false
all_flag=true

# Get flags
while getopts "vtlrfb" opt; do
    case "$opt" in
        v) viz_flag=true ;;
        t) top_flag=true ;;
        l) left_flag=true ;;
        r) right_flag=true ;;
        f) front_flag=true ;;
        b) back_flag=true ;;
        ?) echo "Usage: $0 [-v] [-t] [-l] [-r] [-f] [-b]" ; exit 1 ;;
    esac
done

# If any of the flags are set, then the all flag is false
if [ "$top_flag" = true ] || [ "$left_flag" = true ] || [ "$right_flag" = true ] || [ "$front_flag" = true ] || [ "$back_flag" = true ]; then
    all_flag=false
fi

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
METADATA_FILE_PATH="$LOG_DIR/metadata"

if [ -d "$LOG_DIR" ]; then
    rm -r $LOG_DIR
    echo "Recreating log directory: $LOG_DIR"
fi

mkdir -p "$METADATA_FILE_PATH"

# Variables common to all sensors
    # Retrieve the primary link-local IP address in the 169.254.x.x range 
    # *** This is the IP address of the host machine. DOUBLE CHECK! ***
# Retrieve the primary link-local IP address in the 169.254.x.x range
# udp_address=$(ip addr show scope link | grep -oP 'inet 169\.254\.\d+\.\d+' | awk '{print $2}')
UDP_ADDRESS=192.168.68.45
echo "UDP address: $UDP_ADDRESS"

USE_SYSTEM_DEFAULT_QOS=true
SET_LIDAR_PERSIST=false
UDP_OFFSET=-37.0
TIMESTAMP_MODE="TIME_FROM_PTP_1588"
UDP_LIDAR_PORT=7502
PROC_MASK="PCL|IMU"     # This controls what topics are published. The options are IMU|PCL|SCAN|IMG|TLM.
POINT_TYPE="original"      # Options to use for cart are 'original' and 'native'

# Launch the sensors and redirect outputs to log files

# Top # Set min range to 2.0 to exclude the top of the cart
if [ "$all_flag" = true ] || [ "$top_flag" = true ]; then
    echo "Launching top sensor"
    ros2 launch ouster_ros sensor.launch.xml \
        sensor_hostname:=${LIDAR_IP_TOP} \
        udp_dest:=${UDP_ADDRESS} \
        use_system_default_qos:=${USE_SYSTEM_DEFAULT_QOS} \
        metadata:="$METADATA_FILE_PATH/top_lidar.json" \
        viz:=false \
        ouster_ns:="ouster_top" \
        sensor_frame:="os_sensor_top" \
        lidar_frame:="os_lidar_top" \
        imu_frame:="os_imu_top" \
        min_range:="2.0" \
        timestamp_mode:=${TIMESTAMP_MODE} \
        ptp_utc_tai_offset:=${UDP_OFFSET} \
        lidar_port:=${UDP_LIDAR_PORT} \
        imu_port:=$((UDP_LIDAR_PORT + 1)) \
        proc_mask:=$PROC_MASK \
        point_type:=${POINT_TYPE} \
        persist_config:=${SET_LIDAR_PERSIST} \
        > "$LOG_DIR/ouster_top.log" 2>&1 &
fi

# Left
if [ "$all_flag" = true ] || [ "$left_flag" = true ]; then
    echo "Launching left sensor"
    ros2 launch ouster_ros sensor.launch.xml \
        sensor_hostname:=${LIDAR_IP_LEFT} \
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
        timestamp_mode:=${TIMESTAMP_MODE} \
        ptp_utc_tai_offset:=${UDP_OFFSET} \
        lidar_port:=$((UDP_LIDAR_PORT + 2)) \
        imu_port:=$((UDP_LIDAR_PORT + 3)) \
        proc_mask:=$PROC_MASK \
        point_type:=${POINT_TYPE} \
        persist_config:=${SET_LIDAR_PERSIST} \
        > "$LOG_DIR/ouster_left.log" 2>&1 &

    ros2 run tf2_ros static_transform_publisher \
        --frame-id os_sensor_top \
        --child-frame-id os_sensor_left \
        --x 0.09017880252546773 --y 0.7600579556864714 --z -0.2667973327865499 \
        --qx -0.16166175716485556 --qy 0.3921429595683178 --qz 0.8392065015549289 --qw 0.3403260543645007 \
        >> "$LOG_DIR/ouster_left.log" 2>&1 &
fi

# Right
if [ "$all_flag" = true ] || [ "$right_flag" = true ]; then
    echo "Launching right sensor"
    ros2 launch ouster_ros sensor.launch.xml \
        sensor_hostname:=${LIDAR_IP_RIGHT} \
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
        timestamp_mode:=${TIMESTAMP_MODE} \
        ptp_utc_tai_offset:=${UDP_OFFSET} \
        lidar_port:=$((UDP_LIDAR_PORT + 4)) \
        imu_port:=$((UDP_LIDAR_PORT + 5)) \
        proc_mask:=$PROC_MASK \
        point_type:=${POINT_TYPE} \
        persist_config:=${SET_LIDAR_PERSIST} \
        > "$LOG_DIR/ouster_right.log" 2>&1 &

    ros2 run tf2_ros static_transform_publisher \
        --frame-id os_sensor_top \
        --child-frame-id os_sensor_right \
        --x 0.07847588612408217 --y -0.7328592975248958 --z -0.29798150553149555 \
        --qx -0.14565451134417173 --qy -0.37157075868986944 --qz 0.8457973337117132 --qw -0.35404350706923343 \
        >> "$LOG_DIR/ouster_right.log" 2>&1 &
fi

# Front
if [ "$all_flag" = true ] || [ "$front_flag" = true ]; then
    echo "Launching front sensor"
    ros2 launch ouster_ros sensor.launch.xml \
        sensor_hostname:=${LIDAR_IP_FRONT} \
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
        timestamp_mode:=${TIMESTAMP_MODE} \
        ptp_utc_tai_offset:=${UDP_OFFSET} \
        lidar_port:=$((UDP_LIDAR_PORT + 6)) \
        imu_port:=$((UDP_LIDAR_PORT + 7)) \
        proc_mask:=$PROC_MASK \
        point_type:=${POINT_TYPE} \
        persist_config:=${SET_LIDAR_PERSIST} \
        > "$LOG_DIR/ouster_front.log" 2>&1 &
    
    ros2 run tf2_ros static_transform_publisher \
        --frame-id os_sensor_top \
        --child-frame-id os_sensor_front \
        --x 1.762221953004467 --y 0.031395653998010795 --z -0.23664207446855884 \
        --qx 0.11682054035827495 --qy 0.29901197980482735 --qz 0.3678992311745233 --qw 0.8726940775465323 \
        >> "$LOG_DIR/ouster_front.log" 2>&1 &
fi

# Back
if [ "$all_flag" = true ] || [ "$back_flag" = true ]; then
    echo "Launching back sensor"
    ros2 launch ouster_ros sensor.launch.xml \
        sensor_hostname:=${LIDAR_IP_BACK} \
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
        timestamp_mode:=${TIMESTAMP_MODE} \
        ptp_utc_tai_offset:=${UDP_OFFSET} \
        lidar_port:=$((UDP_LIDAR_PORT + 8)) \
        imu_port:=$((UDP_LIDAR_PORT + 9)) \
        proc_mask:=$PROC_MASK \
        point_type:=${POINT_TYPE} \
        persist_config:=${SET_LIDAR_PERSIST} \
        > "$LOG_DIR/ouster_back.log" 2>&1 &
    
    ros2 run tf2_ros static_transform_publisher \
        --frame-id os_sensor_top \
        --child-frame-id os_sensor_back \
        --x -1.7610962891439355 --y -0.02243506545853049 --z -0.17876566558874363 \
        --qx -0.24144946126196618 --qy -0.10417180529640005 --qz 0.8914838079012679 --qw 0.3689268394783042 \
        >> "$LOG_DIR/ouster_back.log" 2>&1 &
fi

if [ "$viz_flag" = true ]; then
    echo "Launching Rviz2"
    if [ "$top_flag" = true ] && [ "$left_flag" = false ] && [ "$right_flag" = false ] && [ "$front_flag" = false ] && [ "$back_flag" = false ]; then
        RVIZ_FILE_PATH="../src/rviz_config/BasicTopLidar.rviz"
    else
        RVIZ_FILE_PATH="../src/rviz_config/All5LidarTF.rviz"
    fi
    
    rviz2 -d "$RVIZ_FILE_PATH" > "$LOG_DIR/rviz.log" 2>&1 &
fi

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
