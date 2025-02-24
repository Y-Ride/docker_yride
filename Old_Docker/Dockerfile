# This is to create a Docker image for the Y-Ride project.
# Copywrite 2024: Ben Boyack @ Brigham Young University

ARG ROS_DISTRO=humble
# ARG ROS_DOMAIN_ID=42

FROM ros:${ROS_DISTRO}-ros-core

# # nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros2bag \
    python3-rospkg          \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-venv \
    nano \
    && rm -rf /var/lib/apt/lists/* 

# Set up a virtual environment for non-apt packages
RUN python3 -m venv /opt/venv

# Activate the virtual environment and install ouster-sdk via pip
RUN . /opt/venv/bin/activate && pip install --upgrade pip && pip install ouster-sdk

# Ensure the virtual environment is used by default
ENV PATH="/opt/venv/bin:$PATH"

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro ${ROS_DISTRO}

# install ros2 packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-humble-desktop \
#     && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install ros-humble-gtsam cmake vim tmux -y && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc 
RUN apt-get update && apt-get install cmake vim tmux -y && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc 

# Install Ouster package
RUN mkdir -p ~/docker_ros2_ws/ && cd ~/docker_ros2_ws/

# # Install Nvidia Container Toolkit for GPU support
# RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#   && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
#     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && apt-get update && apt-get install -y nvidia-container-toolkit


# Set the entrypoint to keep the container running
# Or you can use `["tail", "-f", "/dev/null"]` to keep it running
CMD ["bash"]  