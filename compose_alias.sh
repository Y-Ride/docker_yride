#!/bin/bash

COMPOSE_FILE=ws_docker_compose.yaml

# Function for compose up with timing
compose() {
    if [ "$1" == "up" ]; then
        shift
        docker compose -f ${COMPOSE_FILE} up "$@"
    elif [ "$1" == "down" ]; then
        shift
        docker compose -f ${COMPOSE_FILE} down "$@"
    else
        echo "Invalid option. Use 'compose up' or 'compose down'."
    fi
}

# Export the function so it's available in the current shell session
export -f compose
