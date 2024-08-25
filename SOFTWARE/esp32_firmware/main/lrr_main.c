#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "LSM6DS3_imu_driver.h"
#include "drive_base_driver.h"
#include "lidar_driver.h"
#include "socket_mgr.h"
#include "status_led_driver.h"
#include "wifi_mgr.h"

void app_main(void)
{
    status_led_driver_init();

    set_status(eSystemGood);

    LSM6DS3_imu_driver_init();

    drive_base_driver_init();

    lidar_driver_init();

    wifi_mgr_init();

    socket_mgr_init();
}
