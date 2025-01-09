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
        docker compose -f "${files[0]}" up -d "$@"
        set_container_name "${files[0]}"
        docker exec -it $CONTAINER_NAME bash
    elif [ "$1" == "down" ]; then
        shift
        docker compose -f "${files[0]}" down "$@"
    else
        echo "Invalid option. Use 'compose up' or 'compose down'."
    fi
}

set_container_name() {
    # Check if a Docker Compose file is provided
    if [ -z "$1" ]; then
        echo "Usage: $0 <docker-compose-file>"
        exit 1
    fi

    DOCKER_COMPOSE_FILE="$1"

    # Extract the container name using yq or grep/sed (prefer yq if available)
    if command -v yq &>/dev/null; then
        CONTAINER_NAME=$(yq '.services[] | .container_name' "$DOCKER_COMPOSE_FILE")
    else
        CONTAINER_NAME=$(grep 'container_name:' "$DOCKER_COMPOSE_FILE" | awk '{print $2}' | head -n 1)
    fi

    # Check if we got a container name
    if [ -z "$CONTAINER_NAME" ]; then
        echo "No container_name found in $DOCKER_COMPOSE_FILE"
        exit 1
    fi

    # Set the environment variable
    export CONTAINER_NAME="$CONTAINER_NAME"
    echo "CONTAINER_NAME is set to: $CONTAINER_NAME"

    # # Optionally, save it to your shell environment
    # echo "export CONTAINER_NAME=$CONTAINER_NAME" >> ~/.bashrc

}

# Export the function so it's available in the current shell session
export -f compose
export -f set_container_name
