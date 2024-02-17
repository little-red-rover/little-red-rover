# ros_iron_dev

FROM micro_ros_iron_firmware

# Setup my dev env

RUN apt-get install tmux openssh-client unzip npm -y

RUN curl -OL https://github.com/neovim/neovim/releases/download/stable/nvim-linux64.tar.gz && \
	tar -C /usr/local --strip-components 1 -xf nvim-linux64.tar.gz && \
	rm nvim-linux64.tar.gz

ENV TMUX_PLUGIN_MANAGER_PATH="~/.tmux/plugins/"

RUN cd $HOME && \
	echo ".cfg" >> .gitignore && \
	git clone --bare https://github.com/usedhondacivic/.dotfiles.git $HOME/.cfg && \
	/usr/bin/git --git-dir=$HOME/.cfg/ --work-tree=$HOME checkout && \
	/usr/bin/git --git-dir=$HOME/.cfg/ --work-tree=$HOME config --local status.showUntrackedFiles no
	
RUN git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm && \
	~/.tmux/plugins/tpm/bin/install_plugins

RUN mkdir -p /little_red_rover_ws/src
WORKDIR /little_red_rover_ws

COPY ../little_red_rover_ws/src /little_red_rover_ws/src
RUN rosdep install -i --from-path src --rosdistro iron -y
RUN colcon build

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash && source /microros_ws/install/local_setup.bash && source /little_red_rover_ws/install/local_setup.bash" >> /root/.bashrc

WORKDIR /little_red_rover_ws/src

CMD bash -c "bash"
