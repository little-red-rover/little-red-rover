# micro_ros_iron_flash

FROM ros_iron_base

RUN apt-get -y install build-essential nghttp2 libnghttp2-dev libssl-dev

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup build_firmware.sh

CMD bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source install/local_setup.bash && ros2 run micro_ros_setup flash_firmware.sh"
