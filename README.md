# ITR ROS Workspace for x500 Drone

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
