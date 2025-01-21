#!/bin/bash

ccbi() {
    # pushd "$(dirname "$(realpath $0)")" > /dev/null

    clear
    colcon build
    source ./install/setup.bash

    # popd > /dev/null
}

colcon_clean() {
    rm -rf install/ build/ log/
    echo "Removed install/ build/ and log/"
}

export -f colcon_clean
export -f ccbi