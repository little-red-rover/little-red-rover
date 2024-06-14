idf_component_register(SRCS "LSM6DS3_imu_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES micro_ros_espidf_component
                    PRIV_REQUIRES driver micro_ros_mgr
                    )
