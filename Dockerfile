FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]


WORKDIR /ros2_ws

COPY ./ros2_ws/src ./src


RUN apt-get update && rosdep update && rosdep install -i --from-paths src --rosdistro jazzy -y
RUN apt-get install ros-jazzy-usb-cam -y 

RUN source /opt/ros/jazzy/setup.bash && colcon build
COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["bash"]