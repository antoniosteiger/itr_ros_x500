# ITR ROS2 Workspace for x500 Drone

This is a ROS2 Workspace for quadcopter control developed to be used through docker, a cross-platform containerization engine. This should allow using this repository on almost any operating system (with some modifications). 

## How to Install:
1. Install Docker and Docker-Compose: \
on Arch: `sudo pacman -S docker docker-compose` \
on Ubuntu: `sudo apt install docker docker-compose` \
on other Operating Systems: Refer to [Docker Documentation](https://docs.docker.com/engine/install/) and [Docker-Compose Documentation](https://docs.docker.com/compose/install/). 
2. Clone this git repository: `git clone https://github.com/antoniosteiger/itr_ros_x500.git`
3. Build the docker images:\
`cd itr_ros_x500` \
`docker-compose build`
4. A successful installation is denoted by:\
✔ base         Built        
✔ dev          Built\
✔ gcs          Built\
✔ itr-agent    Built\
✔ itr-control  Built\
✔ itr-real     Built\
✔ itr-world    Built\
✔ itr-world    Built
5. Give docker containers the permission to open windows on the host computer:\
`xhost +local:root`
6. You can persist this rule by adding it to your shell startup script (example for bash):\
`echo 'xhost +local:root >/dev/null 2>&1' >> ~/.bashrc`

## How to Use:
### Simulation:
To start a simulation, three ingredients are needed: An instance of the quadcopter flight software (PX4) running together with a simulated quadcopter in Gazebo (Physics Simulator), a ROS2 translation agent and a ground control station (GCS). When all of these are running, a controller can start commanding the simulated drone using ROS2.

Start PX4 and the Simulator with: `docker compose up -d itr-world`\
Start the ROS2 bridge with: `docker compose up -d itr-agent`\
Start the ground control station with: `docker compose up -d itr-gcs`

Now you can start a ROS2 based controller that uses px4_msgs nessage definitions to retrieve measurements and send actuator commands.

### Real Deployment:
TBD.

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
