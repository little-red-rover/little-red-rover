// https://wiki.youyeetoo.com/en/Lidar/LD20
// https://github.com/ldrobotSensorTeam/ldlidar_sl_sdk/tree/master/ldlidar_driver/src

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pub_sub_utils.h"
#include "sdkconfig.h"
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <time.h>

#include "drive_base_driver.h"
#include "math.h"
#include "micro_ros_mgr.h"
#include "sensor_msgs/msg/laser_scan.h"
#include <rcl/rcl.h>
#include <rclc/executor_handle.h>

#define DRIVE_BASE_TASK_SIZE (2048)

#define MOTOR_ENABLE 6

#define LEFT_MOTOR_PWM_A_PIN 7
#define LEFT_MOTOR_PWM_A_CHANNEL 0
#define LEFT_MOTOR_PWM_B_PIN 8
#define LEFT_MOTOR_PWM_B_CHANNEL 1
#define RIGHT_MOTOR_PWM_A_PIN 9
#define RIGHT_MOTOR_PWM_A_CHANNEL 2
#define RIGHT_MOTOR_PWM_B_PIN 10
#define RIGHT_MOTOR_PWM_B_CHANNEL 3

#define PWM_TIMER_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_FREQ_HZ 4000

static const char *TAG = "drive_base_driver";

// geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__LaserScan cmd_vel_msg;

rcl_subscription_t *cmd_vel_subscription;

typedef struct
{
	ledc_channel_t chan_a;
	ledc_channel_t chan_b;
} motor_handle_t;

motor_handle_t left_motor_handle;
motor_handle_t right_motor_handle;

void cmd_vel_callback(const void *msgin)
{
	ESP_LOGI(TAG, "Hit!");
}

static void set_drive_base_enabled(bool enable)
{
	gpio_set_direction(MOTOR_ENABLE, GPIO_MODE_OUTPUT);

	if (enable) {
		gpio_set_level(MOTOR_ENABLE, 0);
	} else {
		gpio_set_level(MOTOR_ENABLE, 1);
	}
}

static void set_motor_speed(motor_handle_t *motor, double speed)
{
	if (speed > 0) {
		ledc_set_duty(LEDC_LOW_SPEED_MODE,
					  motor->chan_a,
					  (uint32_t)(speed * (double)(1 << PWM_TIMER_RESOLUTION)));
		ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->chan_b, 0);
	} else {
		ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->chan_a, 0);
		ledc_set_duty(LEDC_LOW_SPEED_MODE,
					  motor->chan_b,
					  (uint32_t)(-speed * (double)(1 << PWM_TIMER_RESOLUTION)));
	}
	ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->chan_a);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->chan_b);
}

static void set_diff_drive(double left, double right)
{
	set_motor_speed(&left_motor_handle, left);
	set_motor_speed(&right_motor_handle, right);
}

static void drive_base_driver_task(void *arg)
{
	while (get_uros_state() != AGENT_CONNECTED) {
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
	set_drive_base_enabled(true);
	while (1) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void configure_pwm(ledc_channel_t channel, int gpio)
{
	ledc_channel_config_t pwm_channel = { .speed_mode = LEDC_LOW_SPEED_MODE,
										  .channel = channel,
										  .timer_sel = LEDC_TIMER_0,
										  .intr_type = LEDC_INTR_DISABLE,
										  .gpio_num = gpio,
										  .duty = 0,
										  .hpoint = 0 };

	ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
}

void drive_base_driver_init()
{
	ledc_timer_config_t pwm_timer = { .speed_mode = LEDC_LOW_SPEED_MODE,
									  .duty_resolution = PWM_TIMER_RESOLUTION,
									  .timer_num = LEDC_TIMER_0,
									  .freq_hz = PWM_FREQ_HZ,
									  .clk_cfg = LEDC_AUTO_CLK };

	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

	configure_pwm(LEFT_MOTOR_PWM_A_CHANNEL, LEFT_MOTOR_PWM_A_PIN);
	configure_pwm(LEFT_MOTOR_PWM_B_CHANNEL, LEFT_MOTOR_PWM_B_PIN);
	configure_pwm(RIGHT_MOTOR_PWM_A_CHANNEL, RIGHT_MOTOR_PWM_A_PIN);
	configure_pwm(RIGHT_MOTOR_PWM_B_CHANNEL, RIGHT_MOTOR_PWM_B_PIN);

	left_motor_handle.chan_a = LEFT_MOTOR_PWM_A_CHANNEL;
	left_motor_handle.chan_b = LEFT_MOTOR_PWM_B_CHANNEL;

	right_motor_handle.chan_a = RIGHT_MOTOR_PWM_A_CHANNEL;
	right_motor_handle.chan_b = RIGHT_MOTOR_PWM_B_CHANNEL;

	// geometry_msgs__msg__Twist__init(&cmd_vel_msg);

	// cmd_vel_subscription = register_subscription(
	//   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
	//   "lrr_lidar_scan",
	//   &cmd_vel_msg,
	//   &cmd_vel_callback);

	xTaskCreate(drive_base_driver_task,
				"drive_base_driver_task",
				DRIVE_BASE_TASK_SIZE,
				NULL,
				10,
				NULL);
}