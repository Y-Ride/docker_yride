#!/bin/bash

# Allow xhost access to the container
# xhost +local:docker # run this in launch file instead and remove at end of launch file

COMPOSE_FILE=*compose.yaml

# Function for compose up with timing
compose() {
    # Check if there is more than one file matching the pattern
    local files=($(ls ${COMPOSE_FILE} 2>/dev/null))
    local file_count=${#files[@]}

    if [ "$file_count" -gt 1 ]; then
        echo "Error: More than one file matches the pattern ${COMPOSE_FILE}."
        return 1
    elif [ "$file_count" -eq 0 ]; then
        echo "Error: No file matches the pattern ${COMPOSE_FILE}."
        return 1
    fi

    # If exactly one file is found, print it and proceed with the compose command
    echo "Using file: ${files[0]}"

    if [ "$1" == "up" ]; then
        shift
        docker compose -f "${files[0]}" up "$@"
    elif [ "$1" == "down" ]; then
        shift
        docker compose -f "${files[0]}" down "$@"
    else
        echo "Invalid option. Use 'compose up' or 'compose down'."
    fi
}

# Export the function so it's available in the current shell session
export -f compose
