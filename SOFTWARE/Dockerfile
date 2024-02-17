#####
# Setup workspace
#####
FROM ros:iron as base
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /ros2_setup.bash

RUN mkdir -p /little_red_rover_ws/src

#####
# Setup microROS tools in the workspace
#####

FROM base as microros_setup

WORKDIR "/little_red_rover_ws"
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN apt update && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y && \
	apt-get install -y --no-install-recommends python3-pip

RUN source /ros2_setup.bash &&  colcon build
RUN echo "source /little_red_rover_ws/install/local_setup.bash" >> /ros2_setup.bash

RUN apt-get install -y --no-install-recommends libssl-dev bear

RUN source /ros2_setup.bash && \
	ros2 run micro_ros_setup create_agent_ws.sh && \
	ros2 run micro_ros_setup build_agent.sh && \
	ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

# Create a "fake" app to make microROS generate build files on layers previous to copying in the actual app
RUN mkdir /little_red_rover_ws/firmware/freertos_apps/apps/esp32_firmware && \
	touch /little_red_rover_ws/firmware/freertos_apps/apps/esp32_firmware/app.c && \
	touch /little_red_rover_ws/firmware/freertos_apps/apps/esp32_firmware/app-colcon.meta && \
	echo "void appMain(void *arg){}" > /little_red_rover_ws/firmware/freertos_apps/apps/esp32_firmware/app.c

RUN source /ros2_setup.bash && \
	ros2 run micro_ros_setup configure_firmware.sh esp32_firmware --transport serial && \
	bear -- ros2 run micro_ros_setup build_firmware.sh && \
	mv compile_commands.json /compile_commands.json


#####
# Build ESP32 firmware
#####

FROM microros_setup as build_user_firmware

COPY /esp32_firmware /microros_ws/firmware/freertos_apps/apps/esp32_firmware

RUN source /ros2_setup.bash && \
	ros2 run micro_ros_setup build_firmware.sh

RUN rm -rf /little_red_rover_ws/firmware/toolchain/esp-idf/.git

#####
# Build ROS2 code
#####

FROM build_user_firmware as build_user_code

RUN mkdir -p /little_red_rover_ws/src
WORKDIR /little_red_rover_ws

COPY ../little_red_rover_ws/src/* /little_red_rover_ws/src/

RUN apt-get update && \
	rosdep update && \ 
	rosdep install -i --from-path src --rosdistro iron -y && colcon build

#####
# Setup local dev environment (Optional, just my personal dev env)
#####

FROM base as dev_env_setup

RUN apt-get update && apt-get install -y --no-install-recommends curl tmux openssh-client unzip npm

RUN curl -OL https://github.com/neovim/neovim/releases/download/stable/nvim-linux64.tar.gz && \
	tar -C /usr/local --strip-components 1 -xf nvim-linux64.tar.gz && \
	rm nvim-linux64.tar.gz && \
	nvim --headless "+Lazy! sync" +qa

ENV TMUX_PLUGIN_MANAGER_PATH="~/.tmux/plugins/"

RUN cd $HOME && \
	echo ".cfg" >> .gitignore && \
	git clone --depth=1 --bare https://github.com/usedhondacivic/.dotfiles.git $HOME/.cfg && \
	/usr/bin/git --git-dir=$HOME/.cfg/ --work-tree=$HOME checkout && \
	/usr/bin/git --git-dir=$HOME/.cfg/ --work-tree=$HOME config --local status.showUntrackedFiles no
	
RUN git clone --depth=1 https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm && \
	~/.tmux/plugins/tpm/bin/install_plugins

#####
# Final image
#####

FROM dev_env_setup

WORKDIR "/little_red_rover_ws"

COPY --from=build_user_firmware /little_red_rover_ws /little_red_rover_ws

RUN apt-get update && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y && \
	apt-get install -y --no-install-recommends tmux

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash && source /little_red_rover_ws/install/local_setup.bash" >> /ros2_setup.bash

RUN echo "alias micro_ros_flash='ros2 run micro_ros_setup build_firmware.sh && ros2 run micro_ros_setup flash_firmware.sh'" >> /ros2_setup.bash

RUN echo "alias micro_ros_agent='ros2 run micro_ros_agent micro_ros_agent serial --dev \$(ls /dev/serial/by-id/*)'" >> /ros2_setup.bash

RUN echo "source /ros2_setup.bash" >> /root/.bashrc

CMD bash -c "source /ros2_setup.bash && bash"

