idf_component_register(SRCS "lrr_micro_ros.c"
                    INCLUDE_DIRS ""
                    REQUIRES wifi_mgr micro_ros_mgr lidar_driver micro_ros_espidf_component
                    )