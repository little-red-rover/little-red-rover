FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /ros2_setup.bash

RUN apt-get update && \
apt-get install -y --no-install-recommends wget curl ros-${ROS_DISTRO}-rosbridge-server clang-format

# gazebo setup
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
apt-get update && \
apt-get install -y --no-install-recommends gz-garden ros-$ROS_DISTRO-ros-gzgarden python3 python3-pip iputils-ping

RUN curl -O https://raw.githubusercontent.com/gazebosim/gz-launch/main/examples/websocket.gzlaunch

# micro ros setup
WORKDIR "/micro_ros_ws"
RUN git clone --depth=1 -b $ROS_DISTRO --single-branch --shallow-submodules https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN apt update && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y && \
	apt-get install -y --no-install-recommends python3-pip

RUN source /ros2_setup.bash && colcon build
RUN echo "source /micro_ros_ws/install/local_setup.bash" >> /ros2_setup.bash

RUN source /ros2_setup.bash && \ 
	ros2 run micro_ros_setup create_agent_ws.sh && \
	ros2 run micro_ros_setup build_agent.sh

# Setup ROS workspace
RUN mkdir -p /little_red_rover_ws/src
COPY /little_red_rover_ws/src /little_red_rover_ws/src

WORKDIR /little_red_rover_ws

RUN apt update && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y 

RUN pip3 install protobuf cryptography pathlib

RUN source /ros2_setup.bash && colcon build
RUN echo "source /little_red_rover_ws/install/local_setup.bash" >> /ros2_setup.bash

# By default gazebo calculates Lidar data using the GPU.
# Setting this ENV variable causes it to be calculated on the CPU, but slower.
# Can be solved per computer by linking the GPU through docker-compose.yml
# https://github.com/gazebosim/gz-sensors/issues/26
RUN echo "export LIBGL_ALWAYS_SOFTWARE=true" >> /ros2_setup.bash

RUN echo "alias start_sim_vis='colcon build && ros2 launch llr_base llr_vis_sim.launch.py'" >> /root/.bashrc
RUN echo "alias lrr_connect='. /tools/wifi_auth.bash'" >> /root/.bashrc

RUN echo "export IDF_PATH=/tools/idf" >> /root/.bashrc

RUN echo "source /ros2_setup.bash" >> /root/.bashrc

CMD bash -c "bash"
