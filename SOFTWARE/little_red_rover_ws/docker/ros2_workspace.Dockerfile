FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /ros2_setup.bash

### tooling
RUN apt-get update && \
apt-get install -y --no-install-recommends python3 python3-pip wget

RUN pip3 install protobuf cryptography pathlib

### gazebo setup
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
apt-get update && \
apt-get install -y --no-install-recommends ros-$ROS_DISTRO-ros-gz

# By default gazebo calculates lidar data using the GPU, but GPU operations aren't possible cross platform in Docker.
# Setting this ENV variable causes it to be calculated on the CPU, but slower.
# https://github.com/gazebosim/gz-sensors/issues/26
RUN echo "export LIBGL_ALWAYS_SOFTWARE=true" >> /ros2_setup.bash

### micro ros setup
WORKDIR "/micro_ros_ws"
RUN git clone --depth=1 -b $ROS_DISTRO --single-branch --shallow-submodules https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN apt update && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y

RUN source /ros2_setup.bash && colcon build
RUN echo "source /micro_ros_ws/install/local_setup.bash" >> /ros2_setup.bash

RUN source /ros2_setup.bash && \ 
	ros2 run micro_ros_setup create_agent_ws.sh && \
	ros2 run micro_ros_setup build_agent.sh

### Setup ROS workspace
RUN mkdir -p /little_red_rover_ws/src
COPY ../src /little_red_rover_ws/src

WORKDIR /little_red_rover_ws

RUN apt update && \
	apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rosbridge-server && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y 

### Dev env setup
RUN apt-get update && \
apt-get install -y --no-install-recommends black iputils-ping python3-venv && \
pip3 install black

RUN source /ros2_setup.bash && colcon build
RUN echo "source /little_red_rover_ws/install/local_setup.bash" >> /ros2_setup.bash

RUN echo "alias lrr_build='(cd /little_red_rover_ws && colcon build --symlink-install)'" >> /root/.bashrc
RUN echo "alias lrr_run='ros2 launch little_red_rover lrr.launch.py'" >> /root/.bashrc
RUN echo "alias lrr_connect='. /tools/wifi_auth.bash'" >> /root/.bashrc

RUN echo "export IDF_PATH=/tools/idf" >> /root/.bashrc

RUN echo "source /ros2_setup.bash" >> /root/.bashrc

CMD bash -c "bash"
