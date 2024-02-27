FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /ros2_setup.bash

RUN mkdir -p /micro_ros_ws/src

WORKDIR "/micro_ros_ws"
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN apt update && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y && \
	apt-get install -y --no-install-recommends python3-pip

RUN source /ros2_setup.bash && colcon build
RUN echo "source /micro_ros_ws/install/local_setup.bash" >> /ros2_setup.bash

RUN source /ros2_setup.bash && \ 
	ros2 run micro_ros_setup create_agent_ws.sh && \
	ros2 run micro_ros_setup build_agent.sh

RUN echo "source /ros2_setup.bash" >> /root/.bashrc

CMD bash -c "source /ros2_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -d"
