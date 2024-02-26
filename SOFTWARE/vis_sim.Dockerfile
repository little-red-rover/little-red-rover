FROM ros:iron as base
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends curl npm ros-$ROS_DISTRO-foxglove-bridge

CMD bash -c "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
