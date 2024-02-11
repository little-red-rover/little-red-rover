# micro_ros_iron_agent

FROM ros_iron_base

RUN apt-get -y install build-essential nghttp2 libnghttp2-dev libssl-dev

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup create_agent_ws.sh

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup build_agent.sh && \
	source install/local_setup.bash

CMD bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source install/local_setup.bash && bash"
