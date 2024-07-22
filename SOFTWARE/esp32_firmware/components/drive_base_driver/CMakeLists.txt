idf_component_register(SRCS "drive_base_driver.c" "motor_driver.c" 
                    INCLUDE_DIRS include
                    REQUIRES micro_ros_espidf_component
                    PRIV_REQUIRES driver micro_ros_mgr pid_ctrl esp_timer
                    )
