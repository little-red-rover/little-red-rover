# micro_ros_iron_firmware

FROM ros_iron_base

RUN apt-get -y install libssl-dev bear

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

# Create a "fake" app to make microROS generate build files on layers previous to copying in the actual app
RUN mkdir /microros_ws/firmware/freertos_apps/apps/esp32_firmware && \
	touch /microros_ws/firmware/freertos_apps/apps/esp32_firmware/app.c && \
	touch /microros_ws/firmware/freertos_apps/apps/esp32_firmware/app-colcon.meta && \
	echo "void appMain(void *arg){}" > /microros_ws/firmware/freertos_apps/apps/esp32_firmware/app.c

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup configure_firmware.sh esp32_firmware --transport serial

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	bear -- ros2 run micro_ros_setup build_firmware.sh

RUN mv compile_commands.json /compile_commands.json
