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

# Launch the sensors and redirect outputs to log files
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=${SENSOR_HOSTNAME_TOP} viz:=false ouster_ns:="ouster_top" > "$LOG_DIR/ouster_top.log" 2>&1 &
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=${SENSOR_HOSTNAME_FRONT} viz:=false ouster_ns:="ouster_front" > "$LOG_DIR/ouster_front.log" 2>&1 &
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=${SENSOR_HOSTNAME_RIGHT} viz:=false ouster_ns:="ouster_right" > "$LOG_DIR/ouster_right.log" 2>&1 &
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=${SENSOR_HOSTNAME_LEFT} viz:=false ouster_ns:="ouster_left" > "$LOG_DIR/ouster_left.log" 2>&1 &
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=${SENSOR_HOSTNAME_BACK} viz:=false ouster_ns:="ouster_back" > "$LOG_DIR/ouster_back.log" 2>&1 &

echo "Sensors launched. Output is in $LOG_DIR/"
echo "Press Ctrl+C to stop the sensors"

wait
