# Dockerfile for base ROS docker image
# Amended from: https://github.com/sea-bass/turtlebot3_behavior_demos/blob/main/docker/Dockerfile
ARG ROS_DISTRO=jazzy
###################################
# BASE IMAGE for Drone Simulation #
###################################
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

# Install the gazebo<->ros bridge and py-trees-ros-viewer:
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-gz-bridge

# Get the PX4-models repository which has the gazebo launch helper
RUN git clone https://github.com/PX4/PX4-gazebo-models.git

# Set up a python virtual environment
RUN mkdir -p /itr_ros_x500/src
WORKDIR /itr_ros_x500
COPY requirements.txt requirements.txt
RUN apt-get install -y --no-install-recommends python3-pip python3-venv \
 && python3 -m venv .venv \
 && touch ./.venv/COLCON_IGNORE \
 && ./.venv/bin/pip install -r requirements.txt

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1


#############
# GCS IMAGE #
#############
# Follows: https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html
# "apt-get install fuse" had to be added
FROM base AS gcs
ARG QGC_VERSION=5.0.8
ENV QGC_VERSION=${QGC_VERSION}

WORKDIR /

# QGC can't be run as root
RUN useradd -m -s /bin/bash qgcuser

RUN usermod -a -G dialout qgcuser \
 && apt-get remove -y modemmanager

RUN apt-get install -y --no-install-recommends gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
 fuse libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev

RUN curl -fsSL -o /usr/local/bin/QGroundControl.AppImage \
    https://github.com/mavlink/qgroundcontrol/releases/download/v${QGC_VERSION}/QGroundControl-x86_64.AppImage \
 && chmod +x /usr/local/bin/QGroundControl.AppImage

RUN chown -R qgcuser:qgcuser /usr/local/bin/QGroundControl.AppImage
USER qgcuser

############################
# PYTREES VISUALIZER IMAGE #
############################
# Should not be run as root, so add user
FROM base AS fsm

WORKDIR /

RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-yasmin ros-${ROS_DISTRO}-yasmin-*

###########################################
# OVERLAY IMAGE with Source Code from Git #
###########################################
FROM base AS overlay

# Create Colcon workspace with util dependencies

WORKDIR /itr_ros_x500

# Get Source Code:
RUN git clone --branch v0.1.5 https://github.com/antoniosteiger/itr_ros_x500.git ./src \
 && rm -rf ./src/docker \
 && rm ./src/docker-compose.yml

# Add px4_msgs package:
RUN PX4_VER=$(echo "${PX4_VERSION}" | cut -d'.' -f1,2) \
 && git clone --branch release/${PX4_VER} https://github.com/PX4/px4_msgs.git ./src/px4_msgs

# Install the packages with Colcon:
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Get Models:
WORKDIR /PX4-gazebo-models
RUN python3 simulation-gazebo --dryrun 

# Place some files:
# Follows: https://docs.px4.io/main/en/sim_gazebo_gz/#adding-new-worlds-and-models
WORKDIR /itr_ros_x500
COPY ./itr_description_x500/ ./src/itr_description_x500/
RUN cp ./src/itr_description_x500/config/1868_x500_itr_sim /PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ \
 && cp ./src/itr_description_x500/config/1868_x500_itr_real /PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/ \
 && sed -i '73a 1868_x500_itr_sim' /PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt \
 && sed -i '68a 1868_x500_itr_real' /PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt \
 && mkdir /root/.simulation-gazebo/models/x500_itr/ \
 && cp ./src/itr_description_x500/models/x500_itr.sdf /root/.simulation-gazebo/models/x500_itr/model.sdf \
 && mkdir /PX4-Autopilot/Tools/simulation/gz/models/x500_itr/ \
 && cp ./src/itr_description_x500/models/x500_itr.sdf /PX4-Autopilot/Tools/simulation/gz/models/x500_itr/model.sdf \
 && cp ./src/itr_description_x500/worlds/itr.sdf /root/.simulation-gazebo/worlds/

WORKDIR /PX4-Autopilot
RUN make px4_sitl
 

##################################################
# DEV IMAGE with source code sync from host disk #
##################################################
FROM overlay AS dev

WORKDIR /itr_ros_x500
# Utility Packages:
COPY ./itr_sim_x500/ ./src/itr_sim_x500/
COPY ./itr_mocap_x500/ ./src/itr_mocap_x500/
COPY ./itr_description_x500/ ./src/itr_description_x500/
COPY ./itr_px4_x500/ ./src/itr_px4_x500/
# Main Packages:
COPY ./itr_controller_x500/ ./src/itr_controller_x500/
COPY ./itr_statemachine_x500/ ./src/itr_statemachine_x500/
COPY ./itr_mission_x500/ ./src/itr_mission_x500/

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
RUN chmod +rx /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]