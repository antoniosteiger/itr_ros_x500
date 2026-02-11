# ITR ROS2 Workspace for x500 Drone
![Screenshot of setup](docs/images/screenshot.png)

This is a modern ROS2 Workspace for control research with the HolyBro x500v2 quadcopter. \
For an overview of the high-level architecture, refer to [this chapter](##Architecture-Overview) down below.

**Features**:
- Cross platform, Ubuntu not needed (Docker containerization; Only tested on Arch so far)
  - Self-documenting: To understand the setup, check out the [dockerfile](docker/Dockerfile) and the [docker-compose.yml](docker-compose.yml) file
  - Enables Exchanging Linux kernel for real-time version such as on [CachyOS](https://wiki.cachyos.org/features/kernel/)
  - Optimize & modify network stack
- State machine using [YASMIN](https://github.com/uleroboticsgroup/yasmin)
  - Real-time visualization of state machine
  - Convenience states provided: Arm, Liftoff, Hover, SafeState, ControllerState
- Mission system:
  - Create custom missions by adding states, add custom states
  - Automatic safe-state fallback on mission errors
- Physics simulation with visualization using Gazebo
- PX4 flight controller of the drone runs as part of simulation in a software-in-the-loop configuration
- Indoor configuration: Motion capture and itr lab geofencing (+- 2m in all directions)
- Comprehensive communication library for PX4 flight controller (enable offboard mode and more)
- ROS2:
  - Very up to date: ROS Jazzy
  - ROS2 DDS middleware using: [ros_gz_bridge](https://gazebosim.org/docs/latest/ros2_integration/) for Gazebo and [MicroXRCEAgent](https://docs.px4.io/main/en/middleware/uxrce_dds) for PX4
  - Use all native ros tools: rosbag, rostopic, ...

**Current Status and TODOs**:
- NOT USED FOR REAL FLIGHT TESTING YET and under active development
- Currently implementing System Level Synthesis with the [Clarabel Solver](https://clarabel.org/stable/)
- Need to implement ingestion of motion capture data into ROS2 network
- Planned: Exchange DDS Middleware for [Zenoh](https://zenoh.io/)
- Implement better example & testing scripts
- Decouple from real-time for faster sims
- Need to implement velocity and angle limiting


## Installation
1. Install Docker and Docker-Compose: \
on Arch: `sudo pacman -S docker docker-compose` \
on Ubuntu: `sudo apt install docker docker-compose` \
on other Operating Systems: Refer to [Docker Documentation](https://docs.docker.com/engine/install/) and [Docker-Compose Documentation](https://docs.docker.com/compose/install/). 
2. Check additional requirements:
  - X11 display server or Xwayland (on Linux)
  - Firewall allows multicast in local network and allows all the ports used by PX4, QGroundControl and Gazebo (given on most Linux distributions)
2. Clone this git repository: `git clone https://github.com/antoniosteiger/itr_ros_x500.git`
3. Add a `.env` file to the project root specifying the software versions you want (the following are recommended and tested):
```
ROS_DISTRO=jazzy
PX4_VERSION=1.16.0
```
4. Build the docker images:\
`cd itr_ros_x500` \
`docker-compose build`
5. A successful installation is denoted by:\
✔ base         Built        
✔ dev          Built\
✔ gcs          Built\
✔ px4          Built\
✔ world        Built\
✔ fsm          Built


## How to Use:
### Simulation:
To start, simply run the start script:
`./start.sh`
You can look at [the script](start.sh) to understand what it does. NOTE: This runs `xhost +local:root` for you, implement a workaround if you are uncomfortable with this. This is needed so that Docker can open windows for you on your machine.\
Three windows will open: Gazebo simulation, YASMIN state machine viewer, QGroundControl and a bash shell attached to a dev container. As soon as these are open and working, you can run controllers and scripts from the dev container. Remember to source the necessary environment and install additional ros packages you are experimenting with yourself:
```
cd /itr_ros_x500
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
```
You can find more commands like these in the dockerfiles of this project. There are example missions in itr_mission_x500/itr_mission_x500/scripts. You can start a test mission from the dev container shell:
`python3 itr_mission_x500/itr_mission_x500/scripts/test_mission.py`

To restart any container, such as PX4, run:
`docker compose restart px4`

To attach a shell to a running container (e.g. px4) for debugging/development purposes run:
`docker compose exec -it px4 bash`

### Real Deployment:
WORK IN PROGRESS!


## Development:
You can use VSCode with the Dev Container extension to attach to/start an instance of the Dev container. To start writing your own missions and custom states, check out the example scripts like [test_mission.py](itr_mission_x500/itr_mission_x500/scripts/test_mission.py) or [simple_mpc_mission.py](itr_mission_x500/itr_mission_x500/scripts/simple_mpc_mission.py) (Not stable at the moment). Generally, you would create your own mission by adding ready-made states like arm and takeoff and a custom controller state. Your controller state implements the base ControllerState from the statemachine package in itr_statemachine_x500.


## Architecture Overview
### Simulation:
![Simulation Architecture](docs/images/simulation_architecture.png)

### Experiments:
![Experiment Architecture](docs/images/experiment_architecture.png)


## Repository Structure:
- itr_controller_x500: ROS2 workspace for actual drone controller implementations (e.g. trajectory tracking)
- itr_statemachine_x500: ROS2 workspace for State machine for the drone (arm/disarm, takeoff, control, return to home, landing, failsafes, ...)
- itr_mission_x500: ROS2 workspace for launching simulations or real flights using controller and state machine
- itr_description_x500: Drone Geometry, Sensors, Rotors, World Geometry, ...
- itr_mocap_x500: Republish qualisys motion capture data as ROS topics
- itr_sim_x500: Gazebo launch script and ROS topic mappings
- itr_px4_x500: PX4 & PX4 ROS bridge launch script + ROS translation script for PX4 inputs
- docker: Base Dockerfile describing ROS2 image used for everything


## Sources
- Repository Structure: https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
- Repository Structure: https://github.com/sea-bass/turtlebot3_behavior_demos
- Repository Structure: https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation
