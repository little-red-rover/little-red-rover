FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends curl ros-$ROS_DISTRO-foxglove-bridge wget
# gazebo setup
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
apt-get update && \
apt-get install -y --no-install-recommends gz-garden ros-$ROS_DISTRO-ros-gzgarden 

RUN curl -O https://raw.githubusercontent.com/gazebosim/gz-launch/main/examples/websocket.gzlaunch

CMD bash -c "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"


# ARG ROS_VERSION=humble
#
# FROM ros:$ROS_VERSION
#
# RUN apt-get update && apt-get install -y --no-install-recommends wget curl
#
# ARG GAZEBO_VERSION=garden
#
# RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
# apt-get update && \
# apt-get install -y --no-install-recommends gz-$GAZEBO_VERSION ros-$ROS_DISTRO-ros-gz$GAZEBO_VERSION
#
# RUN curl -O https://raw.githubusercontent.com/gazebosim/gz-launch/main/examples/websocket.gzlaunch
#
# CMD bash -c "gz sim -s -v 4 shapes.sdf & gz launch -v 4 websocket.gzlaunch"
#
