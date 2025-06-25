# Other destros may not be supported with all of the packages listed below
ARG ROS_DISTRO=humble 

# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:${ROS_DISTRO}

# nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install nano and pip
RUN apt-get update && apt-get install -y nano \
    ros-${ROS_DISTRO}-gtsam \
    ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    libgeographic-dev \
    cmake \
    vim \
    tmux \
    python3-pip \
    && pip3 install --upgrade pip \
    && rm -rf /var/lib/apt/lists/*

# Install Segmentation Libraries
RUN python3 -m pip install numpy matplotlib opencv-python open3d ouster-sdk 

# Install Ouster and LIO-SAM
WORKDIR /docker_yride/other_drivers_ros2_ws/src
RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && \
    git clone -b ros2 --recurse-submodules https://github.com/TixiaoShan/LIO-SAM.git

# # The RAW param is thorwing an error. 
# RUN cd ouster-ros/ouster-ros/config/ && \
#   sed -i 's/proc_mask: IMU|PCL|SCAN|IMG|RAW/proc_mask: IMU|PCL|SCAN|IMG/' driver_params.yaml && \
#   sed -i 's/use_system_default_qos: false/use_system_default_qos: true/' driver_params.yaml
    
# Change a few minor things in LIO-SAM
ADD ros2_ws/src/liosam_params.yaml params.yaml
RUN mv params.yaml LIO-SAM/config/params.yaml

# Build Ouster and LIO-SAM
# Done in separate parts to avoid the build failing due to the size of the build
RUN cd .. && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ouster_sensor_msgs ouster_ros
RUN cd .. && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select lio_sam

# Install Inertial Sense SDK to be able to read the custom messages from the imu
# The build_ros2.sh script also runs colcon build on the inertial sense packages only
RUN git clone --recursive https://github.com/Y-Ride/inertial-sense-sdk.git && \
    ./inertial-sense-sdk/scripts/build_ros2.sh

# Build LIO-RF and LIORF-Localization in the src directory
WORKDIR /docker_yride/ros2_ws/src
RUN git clone -b liorf-ros2 --recurse-submodules https://github.com/Y-Ride/liorf.git && \
    git clone -b liorf_localization-ros2 --recurse-submodules https://github.com/Y-Ride/liorf_localization.git
RUN cd .. && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select liorf
RUN cd .. && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select liorf_localization

# RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#     && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
#     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && apt-get update && apt-get install -y nvidia-container-toolkit

# # Add useful aliases, functions, and environment variables to the bashrc
# ARG HOST_HOME=/home/yride
# WORKDIR /yride_env_setup
# COPY --dereference ./yride_env_setup /yride_env_setup
# RUN ./initial_setup.sh

###########################################################################################################
#                                        Do this Last                                                     #
###########################################################################################################
# Changes ownership of the container to the host user so that files created in the
# container are owned by the host user. This is necessary for the host user to be able to
# delete or modify files created in the container after the container has stopped.
# Define build arguments for UID and GID (Set in docker_compose.yml)
# Following arguments are passed from docker_compose.yml
# ARG UID 
# ARG GID
# ARG UNAME

# # Create group and user with provided UID and GID
# RUN groupadd -g $GID $UNAME && \
#     useradd -m -u $UID -g $GID -s /bin/bash $UNAME

# # Run container as the newly created user
# USER $UNAME

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /docker_yride/other_drivers_ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /docker_yride/ros2_ws/install/setup.bash" >> ~/.bashrc

# Include the custom environment setup script
RUN echo "if [ -f /yride_env_setup/custom_env.sh ]; then" >> ~/.bashrc && \
    echo "  source /yride_env_setup/custom_env.sh" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc
 