#include <rcl/publisher.h>
#include <rcl/types.h>
#include <std_msgs/msg/detail/int32__struct.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lidar_driver.h"
#include "portmacro.h"
#include "pub_sub_utils.h"
#include "wifi_mgr.h"

#include "micro_ros_mgr.h"

#include "LSM6DS3_imu_driver.h"
#include "drive_base_driver.h"

#include <std_msgs/msg/int32.h>

#define RCCHECK(fn)                                                            \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        if ((temp_rc != RCL_RET_OK)) {                                         \
            printf("Failed status on line %d: %d. Aborting.\n",                \
                   __LINE__,                                                   \
                   (int)temp_rc);                                              \
            vTaskDelete(NULL);                                                 \
        }                                                                      \
    }
#define RCSOFTCHECK(fn)                                                        \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        if ((temp_rc != RCL_RET_OK)) {                                         \
            printf("Failed status on line %d: %d. Continuing.\n",              \
                   __LINE__,                                                   \
                   (int)temp_rc);                                              \
        }                                                                      \
    }
void app_main(void)
{

    LSM6DS3_imu_driver_init();

    // drive_base_driver_init();

    // lidar_driver_init();

    wifi_mgr_init();

    std_msgs__msg__Int32 int32_msg;
    rcl_publisher_t *int_publisher = register_publisher(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "int32_testing");
    rcl_publisher_t *int2_publisher = register_publisher(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "int32_testing2");
    rcl_publisher_t *int3_publisher = register_publisher(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "int32_testing3");
    micro_ros_mgr_init();

    while (get_uros_state() != AGENT_CONNECTED) {
    }

    int32_msg.data = 0;
    while (1) {
        ESP_LOGI(":(", ":( before");
        int32_msg.data++;
        RCCHECK(rcl_publish(int_publisher, &int32_msg, NULL));
        RCCHECK(rcl_publish(int2_publisher, &int32_msg, NULL));
        RCCHECK(rcl_publish(int3_publisher, &int32_msg, NULL));
        ESP_LOGI(":(", ":( after");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
