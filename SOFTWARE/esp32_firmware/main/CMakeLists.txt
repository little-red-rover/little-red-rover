idf_component_register(SRCS "llr_micro_ros.c"
                    INCLUDE_DIRS "")

set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../assets/front_end")
spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT)