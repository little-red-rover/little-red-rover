FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
apt-get install -y --no-install-recommends wget curl

# gazebo setup
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
apt-get update && \
apt-get install -y --no-install-recommends gz-garden ros-$ROS_DISTRO-ros-gzgarden 

RUN curl -O https://raw.githubusercontent.com/gazebosim/gz-launch/main/examples/websocket.gzlaunch

# xpra setup
# RUN apt-get update && \
# apt-get install -y --no-install-recommends wget curl lz4 python3-pip python3-gi python3-gi-cairo gir1.2-gtk-4.0 libxxhash-dev libvpx-dev libxcb-fixes0-dev libxcb-damage0-dev libx11-dev libxkbfile-dev
#
# RUN pip3 install Cython PyGObject pyopengl
#
# RUN git clone --depth=1 --single-branch --shallow-submodules https://github.com/Xpra-org/xpra && \
# cd xpra && \
# python3 ./setup.py install
# RUN git clone --depth=1 --single-branch --shallow-submodules https://github.com/Xpra-org/xpra-html5 && \
# cd xpra-html5 && \ 
# ./setup.py install

RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends xpra

CMD bash -c "bash"
