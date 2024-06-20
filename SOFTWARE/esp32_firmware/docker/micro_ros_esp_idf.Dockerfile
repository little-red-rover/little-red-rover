FROM microros/esp-idf-microros:latest 
SHELL ["/bin/bash", "-c"]

RUN echo "alias get_idf='. $HOME/esp/esp-idf/export.sh'" >> /root/.bashrc
RUN echo "alias micro_ros_flash='(. $HOME/esp/esp-idf/export.sh && cd /esp32_firmware && idf.py build flash monitor)'" >> /root/.bashrc

WORKDIR /esp32_firmware

CMD bash -c "bash"
