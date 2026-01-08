# ros2_project
This project consists in developing a CLI interface for basic control of a robot using a camera driver. 

The `install_pks.sh` file will install required packages when executed.

## Instructions

- install ROS distro (Humble for Ubuntu 22 or Jazzy for Ubuntu 24)
- install docker
- Create ros2_ws folder

*Inside ros2_ws*:
- `source /opt/ros/{humble OR jazzy}/setup.bash`
- `colcon build`
- `source install/setup.bash`
- download ur3 driver: `apt install ros-${ROS_DISTRO}-ur-robot-driver`
- install required packages with `rosdep install --from-paths src -y --ignore-src --rosdistro {humble OR jazzy}`
- `colcon build`
- `source install/setup.bash`

Start the UR simulation:
- `ros2 run ur_robot_driver start_ursim.sh -m ur3`

*wait a few seconds*
- `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.56.101 launch_rviz:=true`

*If the robot is white and crawled up like a ball, close Rviz and run the last command again.*
Now, in your browser, go to `http://192.168.56.101:6080/` and conenct to the robot. Turn it on and start it. Go to Program > URCap > External Control. After adding
it to the main program, run it by clicking the "play" button.

To run the robot controller:
- `ros2 run usb_cam usb_cam_node_exe`
- `ros2 run camera_driver camera_node` or `ros2 run camera_driver camera_node --ros-args -p rect_size:={size}` to choose a specific square size.




