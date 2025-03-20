#!/bin/bash

# Define environment variables for topics
export OUSTER_TOP_TOPICS="/ouster_top/points /ouster_top/imu /ouster_top/metadata"
export OUSTER_FRONT_TOPICS="/ouster_front/points /ouster_front/metadata"
export OUSTER_RIGHT_TOPICS="/ouster_right/points /ouster_right/metadata"
export OUSTER_LEFT_TOPICS="/ouster_left/points /ouster_left/metadata"
export OUSTER_BACK_TOPICS="/ouster_back/points /ouster_back/metadata"
export INERTIAL_SENSE_TOPICS="/inertial_sense/imu_raw /inertial_sense/odom_ins_ned /inertial_sense/gps1/info /inertial_sense/gps1/pos_vel /inertial_sense/gps2/info /inertial_sense/gps2/pos_vel"
export OTHER_TOPICS="/tf_static /tf"

# Function to record topics based on flags
record_bag() {
    local topics=()
    local output_path=""

    # Reset OPTIND to ensure we start from the beginning
    OPTIND=1

    # Check flags and add corresponding topics
    while getopts "frltbao:" opt; do
        case ${opt} in
            f)
                topics+=($OUSTER_FRONT_TOPICS)
                ;;
            r)
                topics+=($OUSTER_RIGHT_TOPICS)
                ;;
            l)
                topics+=($OUSTER_LEFT_TOPICS)
                ;;
            t)
                topics+=($OUSTER_TOP_TOPICS)
                ;;
            b)
                topics+=($OUSTER_BACK_TOPICS)
                ;;
            a)
                # No action needed for -a flag here
                ;;
            o)
                output_path="$OPTARG"  # Capture the output file path
                ;;
            \?)
                echo "Invalid option: -$OPTARG" >&2
                return 1
                ;;
        esac
    done

    # If -a is specified, record all topics
    if [[ " ${@} " =~ " -a " ]]; then
        topics=($OUSTER_FRONT_TOPICS $OUSTER_RIGHT_TOPICS $OUSTER_LEFT_TOPICS $OUSTER_TOP_TOPICS $OUSTER_BACK_TOPICS $OTHER_TOPICS)
    fi

    # If no topics are selected, print usage
    if [ ${#topics[@]} -eq 0 ]; then
        echo "Usage: bag [-f] [-r] [-l] [-t] [-b] [-a] [-o file_path]"
        return 1
    fi

    # Add inertial_sense imu/gps topics
    if [ -n "$INERTIAL_SENSE_TOPICS" ]; then
        topics+=($INERTIAL_SENSE_TOPICS)
    fi

    # Add other_topics to topics if it's not empty
    if [ -n "$OTHER_TOPICS" ]; then
        topics+=($OTHER_TOPICS)
    fi

    # Construct the ros2 bag record command
    local command="ros2 bag record ${topics[@]}"
    
    # Add the output path if specified
    if [ -n "$output_path" ]; then
        command+=" -o $output_path"
    fi

    echo "Executing: $command"
    
    # Execute the command directly
    ${command[@]}
}

# Allow the function to be called directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    record_bag "$@"
fi
