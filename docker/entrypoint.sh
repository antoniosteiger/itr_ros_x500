#!/bin/bash
# Basic entrypoint for ROS Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the overlay workspace, if built
if [ -f /itr_ros_x500/install/setup.bash ]
then
  source /itr_ros_x500/install/setup.bash
  echo "Sourced ITR x500 main workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"