#include <nav_msgs/msg/detail/odometry__functions.h>
#include <nav_msgs/msg/detail/odometry__struct.h>
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

    drive_base_driver_init();

    lidar_driver_init();

    wifi_mgr_init();

    micro_ros_mgr_init();
}
