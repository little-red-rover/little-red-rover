idf_component_register(SRCS "micro_ros_mgr.c" "pub_sub_utils.c" 
                    INCLUDE_DIRS include
                    PRIV_REQUIRES micro_ros_espidf_component wifi_mgr drive_base_driver)
