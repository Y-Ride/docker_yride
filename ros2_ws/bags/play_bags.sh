#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if the current directory is the same as the script directory
if [ "$(pwd)" != "$SCRIPT_DIR" ]; then
    echo "Error: This script must be run in the directory: $SCRIPT_DIR"
    exit 1
fi

# Create the log_bag directory if it doesn't exist
LOG_DIR="$SCRIPT_DIR/log_bag"
if [ ! -d "$LOG_DIR" ]; then
    mkdir "$LOG_DIR"
    echo "Created log directory: $LOG_DIR"
fi

# Play the specified bag file from the first argument
# Play for 10 seconds and log to a file
ros2 bag play "$1" \
    > "$LOG_DIR/$(basename "$1" .bag).log" 2>&1 &

# Wait for a moment to ensure topics are being published
sleep 5

# Log headers (or entire messages) of topics ending with /points
# POINTS_LOG="$LOG_DIR/$(basename "$1" .bag)_points.log"
# echo "Logging /points topics to $POINTS_LOG"

# Get a list of topics ending with /points
TOPICS=$(ros2 topic list | grep '/points$')

# Check if any /points topics are found
if [ -z "$TOPICS" ]; then
    echo "No topics ending with /points found."
    exit 1
fi

# Subscribe to each /points topic and log headers
for topic in $TOPICS; do
    echo "Subscribing to $topic..."
    
    # Extract the topic base name (e.g., /ouster_top/points -> ouster_top)
    base_name=$(basename "$(dirname "$topic")")

    # Set the log file path
    log_file="$LOG_DIR/${base_name}.log"
    
    echo "Logging headers of $topic to $log_file"
    ros2 topic echo "$topic" --no-arr > "$log_file" 2>&1 &
done

# Wait for the playback to finish
wait
echo "Finished logging topics."
