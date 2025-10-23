# Dockerfile for base ROS docker image
# Amended from: https://github.com/sea-bass/turtlebot3_behavior_demos/blob/main/docker/Dockerfile
ARG ROS_DISTRO=jazzy
#######################################
# Base Image for Drone Simulation #
#######################################
FROM ros:${ROS_DISTRO} AS base
ARG PX4_VERSION=1.16.0
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PX4_VERSION=${PX4_VERSION}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
 sudo curl wget git python3-pip
 # libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 # python3-opencv python3-tk python3-pyqt5.qtwebengine

# Install additional Python modules
# RUN pip3 install --break-system-packages matplotlib transforms3d

# Exchange middleware
# RUN apt-get update && apt-get install -y --no-install-recommends \
#  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install PX4
# From: https://docs.px4.io/main/en/ros2/user_guide#install-px4
RUN git clone --branch v${PX4_VERSION} --depth 1 https://github.com/PX4/PX4-Autopilot.git --recursive \
 && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

WORKDIR /PX4-Autopilot
RUN make px4_sitl

# Install XRCE DDS Agent to interface with ROS
# From: https://docs.px4.io/main/en/ros2/user_guide#setup-micro-xrce-dds-agent-client
# From Source (snap does not work in containers):
WORKDIR /
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
 && cd /Micro-XRCE-DDS-Agent \
 && mkdir -p build \
 && cd ./build \
 && cmake .. && make && make install \
 && ldconfig

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

###############################
# Image for Utilities #
###############################
FROM base AS overlay

# Create Colcon workspace with util dependencies
RUN mkdir -p /itr_ros_x500/src
WORKDIR /itr_ros_x500

# Utility Packages:
# At a later point these can be spun off into their own git repository to enable new projects
COPY ./itr_sim_x500/ ./src/itr_sim_x500/
COPY ./itr_mocap_x500/ ./src/itr_mocap_x500/
COPY ./itr_description_x500/ ./src/itr_description_x500/
COPY ./itr_px4_x500/ ./src/itr_px4_x500/
# Main Packages:
COPY ./itr_controller_x500/ ./src/itr_controller_x500/
COPY ./itr_statemachine_x500/ ./src/itr_statemachine_x500/
COPY ./itr_mission_x500/ ./src/itr_mission_x500/
# Add px4_msgs package:
RUN PX4_VER=$(echo "${PX4_VERSION}" | cut -d'.' -f1,2) \
 && git clone --branch release/${PX4_VER} https://github.com/PX4/px4_msgs.git ./src/px4_msgs

# Install the packages with Colcon:
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay AS dev

# Dev container arguments
# ARG USERNAME=devuser
# ARG UID=1000
# ARG GID=${UID}

# # Install extra tools for development
# RUN apt-get update && apt-get install -y --no-install-recommends \
#  gdb gdbserver nano

# # In Ubuntu 24.04, there is already a user named "ubuntu" with UID 1000.
# # Delete this in the (common) event that the user on the host also has this UID.
# RUN touch /var/mail/ubuntu \
#     && chown ubuntu /var/mail/ubuntu \
#     && userdel -r ubuntu

# # Create new user and home directory
# RUN groupadd --gid $GID $USERNAME \
#  && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
#  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
#  && chmod 0440 /etc/sudoers.d/${USERNAME} \
#  && mkdir -p /home/${USERNAME} \
#  && chown -R ${UID}:${GID} /home/${USERNAME}

# # Set the ownership of the overlay workspace to the new user
# RUN chown -R ${UID}:${GID} /itr_ros_x500/
RUN chmod +rx /entrypoint.sh

# # Set up the entrypoint, including it in the .bashrc for interactive shells
# USER ${USERNAME}
# RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc