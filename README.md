# ros2_project
This project consists in developing a CLI interface for basic control of a robot using a camera driver. 

The `install_pks.sh` file will install required packages when executed.

## Instructions

- install ROS distro (Humble for Ubuntu 22 or Jazzy for Ubuntu 24)
- install docker
- colcon build
- source /opt/ros/{humble OR jazzy}/setup.bash
- source install/setup.bash
- download ur3 driver
- download packages to ros2_ws/src/
- install with rosdep
- colcon build
- source install/setup.bash

- ros2 run ur_robot_driver start_ursim.sh -m ur3
- ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.56.101 launch_rviz:=true

- run usb_cam
- run camera_driver





