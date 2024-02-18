#####
# Setup workspace
#####
FROM ros:iron as base
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /ros2_setup.bash

RUN mkdir -p /little_red_rover_ws/src

#####
# Setup esp-idf and the micro-ros component
#####

FROM base as esp_setup

RUN apt-get update && \
	apt-get install -y --no-install-recommends git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

RUN mkdir -p ~/esp && \
	cd ~/esp && \
	git clone --depth=1 -b v5.2 --recursive --shallow-submodules https://github.com/espressif/esp-idf.git

RUN cd ~/esp/esp-idf && \
	./install.sh esp32,esp32c3

RUN . $HOME/esp/esp-idf/export.sh && \
	pip3 install catkin_pkg lark-parser colcon-common-extensions


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

#####
# Build ROS2 code
#####

FROM base as build_user_code

WORKDIR /little_red_rover_ws

COPY /little_red_rover_ws/src /little_red_rover_ws/src

RUN apt-get update && \
	rosdep update && \ 
	rosdep install -i --from-path src --rosdistro iron -y && \
	source /ros2_setup.bash && \
	colcon build

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

COPY --from=build_user_code /little_red_rover_ws /little_red_rover_ws
COPY --from=esp_setup /root/esp /root/esp
COPY --from=esp_setup /root/.espressif /root/.espressif

RUN apt-get update && \
	rosdep update && \ 
	rosdep install --from-paths src --ignore-src -y && \
	apt-get install -y --no-install-recommends tmux git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash && source /little_red_rover_ws/install/local_setup.bash" >> /ros2_setup.bash

RUN echo "alias micro_ros_agent='ros2 run micro_ros_agent micro_ros_agent serial --dev \$(ls /dev/serial/by-id/*)'" >> /ros2_setup.bash

RUN echo "alias micro_ros_flash='(. $HOME/esp/esp-idf/export.sh && cd /esp32_firmware && idf.py build flash monitor)'" >> /root/.bashrc

RUN echo "source /ros2_setup.bash" >> /root/.bashrc

CMD bash -c "source /ros2_setup.bash && bash"
