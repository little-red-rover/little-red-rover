FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
	apt-get install -y --no-install-recommends curl git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

RUN mkdir -p ~/esp && \
	cd ~/esp && \
	git clone --depth=1 -b v5.2 --recursive --shallow-submodules https://github.com/espressif/esp-idf.git

RUN cd ~/esp/esp-idf && \
	./install.sh esp32,esp32c3

RUN . $HOME/esp/esp-idf/export.sh && \
	pip3 install catkin_pkg lark-parser colcon-common-extensions

RUN . $HOME/esp/esp-idf/export.sh && cd /esp32_firmware && idf.py build

RUN echo "alias get_idf='. $HOME/esp/esp-idf/export.sh'" >> /root/.bashrc
RUN echo "alias micro_ros_flash='(. $HOME/esp/esp-idf/export.sh && cd /esp32_firmware && idf.py build flash monitor)'" >> /root/.bashrc

CMD bash -c "bash"
