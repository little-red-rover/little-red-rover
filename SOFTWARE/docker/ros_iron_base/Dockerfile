# ros_iron_base

FROM ros:iron

SHELL ["/bin/bash", "-c"]

# https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/

# Create a workspace and download the micro-ROS tools
RUN mkdir microros_ws
WORKDIR "/microros_ws"
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
RUN apt update && rosdep update
RUN rosdep install --from-paths src --ignore-src -y

# Install pip
RUN apt -y install --no-install-recommends python3-pip

# Build micro-ROS tools and source them
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build
RUN source install/local_setup.bash
