#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "hal/pcnt_types.h"
#include "pub_sub_utils.h"
#include <geometry_msgs/msg/detail/twist__functions.h>
#include <geometry_msgs/msg/detail/twist__struct.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "drive_base_driver.h"
#include "micro_ros_mgr.h"
#include <rcl/rcl.h>
#include <rclc/executor_handle.h>
#define DRIVE_BASE_TASK_SIZE (4096)

#define MOTOR_ENABLE 5

#define LEFT_MOTOR_PWM_A_PIN 6
#define LEFT_MOTOR_PWM_A_CHANNEL 0
#define LEFT_MOTOR_PWM_B_PIN 7
#define LEFT_MOTOR_PWM_B_CHANNEL 1
#define RIGHT_MOTOR_PWM_A_PIN 8
#define RIGHT_MOTOR_PWM_A_CHANNEL 2
#define RIGHT_MOTOR_PWM_B_PIN 9
#define RIGHT_MOTOR_PWM_B_CHANNEL 3

#define PWM_TIMER_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_FREQ_HZ 4000

#define LEFT_ENCODER_PIN 4
#define RIGHT_ENCODER_PIN 3

static const char *TAG = "drive_base_driver";

geometry_msgs__msg__Twist *cmd_vel_msg;

rcl_subscription_t *cmd_vel_subscription;

typedef struct
{
	pcnt_channel_handle_t channel;
	pcnt_unit_handle_t unit;
} encoder_handle_t;

typedef struct
{
	ledc_channel_t chan_a;
	ledc_channel_t chan_b;
	double speed;
	encoder_handle_t encoder;
} motor_handle_t;

static motor_handle_t left_motor_handle;
static motor_handle_t right_motor_handle;

static void set_drive_base_enabled(bool enable)
{
	gpio_set_direction(MOTOR_ENABLE, GPIO_MODE_OUTPUT);

	if (enable) {
		gpio_set_level(MOTOR_ENABLE, 1);
	} else {
		gpio_set_level(MOTOR_ENABLE, 0);
	}
}

static void set_motor_speed(motor_handle_t *motor, double speed)
{
	motor->speed = speed;

	if (speed > 0) {
		ledc_set_duty(LEDC_LOW_SPEED_MODE,
					  motor->chan_b,
					  (uint32_t)(speed * (double)(1 << PWM_TIMER_RESOLUTION)));
		ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->chan_a, 0);
		pcnt_channel_set_edge_action(motor->encoder.channel,
									 PCNT_CHANNEL_EDGE_ACTION_DECREASE,
									 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
	} else {
		ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->chan_b, 0);
		ledc_set_duty(LEDC_LOW_SPEED_MODE,
					  motor->chan_a,
					  (uint32_t)(-speed * (double)(1 << PWM_TIMER_RESOLUTION)));
		pcnt_channel_set_edge_action(motor->encoder.channel,
									 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
									 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
	}
	if (speed == 0) {
		pcnt_channel_set_edge_action(motor->encoder.channel,
									 PCNT_CHANNEL_EDGE_ACTION_HOLD,
									 PCNT_CHANNEL_EDGE_ACTION_HOLD);
	}

	ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->chan_a);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->chan_b);
}

static void set_diff_drive(double left, double right)
{
	set_motor_speed(&left_motor_handle, left);
	set_motor_speed(&right_motor_handle, right);
}
double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}

void cmd_vel_callback(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg =
	  (const geometry_msgs__msg__Twist *)msgin;
	double x = msg->angular.z;
	double y = msg->linear.x;
	set_diff_drive(clamp(y + x, -1.0, 1.0),
				   clamp(y - x, -1.0, 1.0));
}

static void drive_base_driver_task(void *arg)
{
	while (get_uros_state() != AGENT_CONNECTED) {
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
	set_drive_base_enabled(true);
	while (1) {
		// int left_pulse_cnt;
		// int right_pulse_cnt;
		// pcnt_unit_get_count(left_motor_handle.encoder.unit, &left_pulse_cnt);
		// pcnt_unit_get_count(right_motor_handle.encoder.unit,
		// &right_pulse_cnt);
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

motor_handle_t configure_motor(gpio_num_t pwm_a_pin,
							   ledc_channel_t pwm_a_chan,
							   gpio_num_t pwm_b_pin,
							   ledc_channel_t pwm_b_chan,
							   gpio_num_t encoder_pin)
{
	// PWM
	motor_handle_t motor;
	configure_pwm(pwm_a_chan, pwm_a_pin);
	configure_pwm(pwm_b_chan, pwm_b_pin);

	motor.chan_a = pwm_a_chan;
	motor.chan_b = pwm_b_chan;

	// ENCODER
	gpio_pullup_en(encoder_pin);

	pcnt_unit_config_t unit_config = { .low_limit = INT16_MIN,
									   .high_limit = INT16_MAX,
									   .flags = { .accum_count = 1 } };

	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &motor.encoder.unit));

	pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 10000 };
	ESP_ERROR_CHECK(
	  pcnt_unit_set_glitch_filter(motor.encoder.unit, &filter_config));

	pcnt_chan_config_t chan_config = {
		.edge_gpio_num = encoder_pin,
	};
	ESP_ERROR_CHECK(pcnt_new_channel(
	  motor.encoder.unit, &chan_config, &motor.encoder.channel));

	ESP_ERROR_CHECK(
	  pcnt_channel_set_edge_action(motor.encoder.channel,
								   PCNT_CHANNEL_EDGE_ACTION_HOLD,
								   PCNT_CHANNEL_EDGE_ACTION_HOLD));

	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor.encoder.unit, INT16_MAX));
	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor.encoder.unit, INT16_MIN));

	ESP_ERROR_CHECK(pcnt_unit_enable(motor.encoder.unit));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(motor.encoder.unit));
	ESP_ERROR_CHECK(pcnt_unit_start(motor.encoder.unit));

	return motor;
}

void drive_base_driver_init()
{
	// PWM SETUP
	ledc_timer_config_t pwm_timer = { .speed_mode = LEDC_LOW_SPEED_MODE,
									  .duty_resolution = PWM_TIMER_RESOLUTION,
									  .timer_num = LEDC_TIMER_0,
									  .freq_hz = PWM_FREQ_HZ,
									  .clk_cfg = LEDC_AUTO_CLK };

	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

	// MOTOR SETUP
	left_motor_handle = configure_motor(LEFT_MOTOR_PWM_A_PIN,
										LEFT_MOTOR_PWM_A_CHANNEL,
										LEFT_MOTOR_PWM_B_PIN,
										LEFT_MOTOR_PWM_B_CHANNEL,
										LEFT_ENCODER_PIN);
	right_motor_handle = configure_motor(RIGHT_MOTOR_PWM_A_PIN,
										 RIGHT_MOTOR_PWM_A_CHANNEL,
										 RIGHT_MOTOR_PWM_B_PIN,
										 RIGHT_MOTOR_PWM_B_CHANNEL,
										 RIGHT_ENCODER_PIN);

	// MICRO ROS SETUP
	cmd_vel_msg = geometry_msgs__msg__Twist__create();

	cmd_vel_subscription = register_subscription(
	  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	  "cmd_vel",
	  cmd_vel_msg,
	  &cmd_vel_callback);

	// cmd_vel_subscription = register_subscription(
	//   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	//   "lrr_teleop",
	//   cmd_vel_msg,
	//   &cmd_vel_callback);

	// START TASK
	xTaskCreate(drive_base_driver_task,
				"drive_base_driver_task",
				DRIVE_BASE_TASK_SIZE,
				NULL,
				10,
				NULL);
}
