# ros_iron_dev

FROM micro_ros_iron_firmware

# Setup my dev env

RUN apt-get install tmux openssh-client unzip npm -y

RUN curl -OL https://github.com/neovim/neovim/releases/download/stable/nvim-linux64.tar.gz && \
	tar -C /usr/local --strip-components 1 -xf nvim-linux64.tar.gz

ENV TMUX_PLUGIN_MANAGER_PATH="~/.tmux/plugins/"

RUN git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm && \
	~/.tmux/plugins/tpm/bin/install_plugins

RUN cd $HOME && \
	echo ".cfg" >> .gitignore && \
	git clone --bare https://github.com/usedhondacivic/.dotfiles.git $HOME/.cfg && \
	/usr/bin/git --git-dir=$HOME/.cfg/ --work-tree=$HOME checkout && \
	/usr/bin/git --git-dir=$HOME/.cfg/ --work-tree=$HOME config --local status.showUntrackedFiles no
	
CMD bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source install/local_setup.bash && ~/.tmux/plugins/tpm/bin/install_plugins && bash"
