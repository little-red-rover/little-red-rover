# micro_ros_iron_flash

FROM micro_ros_iron_firmware

COPY esp32_firmware /microros_ws/firmware/freertos_apps/apps/esp32_firmware

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup build_firmware.sh

CMD bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source install/local_setup.bash && ros2 run micro_ros_setup flash_firmware.sh"
