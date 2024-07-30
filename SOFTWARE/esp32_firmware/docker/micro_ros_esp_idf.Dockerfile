FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
	apt-get install -y --no-install-recommends curl git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 clang-format

RUN mkdir -p ~/esp && \
	cd ~/esp && \
	git clone --depth=1 -b release/v5.2 --single-branch --recursive --shallow-submodules https://github.com/espressif/esp-idf.git

RUN cd ~/esp/esp-idf && \
	./install.sh esp32s3

RUN . $HOME/esp/esp-idf/export.sh && \
	pip3 install catkin_pkg lark-parser colcon-common-extensions protobuf cryptography

RUN echo "alias get_idf='. $HOME/esp/esp-idf/export.sh'" >> /root/.bashrc
RUN echo "alias lrr_flash='(. $HOME/esp/esp-idf/export.sh && cd /esp32_firmware && idf.py build flash monitor)'" >> /root/.bashrc

WORKDIR /esp32_firmware

CMD bash -c "bash"
