FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends ros-$ROS_DISTRO-foxglove-bridge ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

CMD bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
