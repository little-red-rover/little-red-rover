#include "drive_base_driver.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/executor_handle.h>

#include <geometry_msgs/msg/detail/twist__functions.h>
#include <geometry_msgs/msg/detail/twist__struct.h>
#include <geometry_msgs/msg/twist.h>

#include <sensor_msgs/msg/detail/joint_state__functions.h>
#include <sensor_msgs/msg/detail/joint_state__struct.h>
#include <sensor_msgs/msg/joint_state.h>
#include <stdio.h>
#include <time.h>

#include "micro_ros_mgr.h"
#include "motor_driver.h"
#include "pub_sub_utils.h"

#define DRIVE_BASE_TASK_SIZE (4096)

// PIN DEFINITIONS
#define MOTOR_ENABLE 5

#define LEFT_MOTOR_PWM_A_PIN 8
#define LEFT_MOTOR_PWM_A_CHANNEL 0
#define LEFT_MOTOR_PWM_B_PIN 9
#define LEFT_MOTOR_PWM_B_CHANNEL 1
#define RIGHT_MOTOR_PWM_A_PIN 6
#define RIGHT_MOTOR_PWM_A_CHANNEL 2
#define RIGHT_MOTOR_PWM_B_PIN 7
#define RIGHT_MOTOR_PWM_B_CHANNEL 3

#define LEFT_ENCODER_PIN 4
#define RIGHT_ENCODER_PIN 3

static const char *TAG = "drive_base_driver";

// SUBSCRIPTIONS
static sensor_msgs__msg__JointState *joint_state_msg;
static rcl_publisher_t *joint_state_publisher;

// PUBLISHERS
static geometry_msgs__msg__Twist *cmd_vel_msg;
static rcl_subscription_t *cmd_vel_subscription;

static motor_handle_t left_motor_handle;
static motor_handle_t right_motor_handle;

static void set_drive_base_enabled(bool enable)
{
    set_motor_enabled(&left_motor_handle, enable);
    set_motor_enabled(&right_motor_handle, enable);
}

static void set_diff_drive(float left, float right)
{
    set_motor_velocity(&left_motor_handle, left);
    set_motor_velocity(&right_motor_handle, right);
}
float clamp(float d, float min, float max)
{
    const float t = d < min ? min : d;
    return t > max ? max : t;
}

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
    double x = msg->angular.z;
    double y = msg->linear.x;
    set_diff_drive(y + x, y - x);
}

static void drive_base_driver_task(void *arg)
{
    while (get_uros_state() != AGENT_CONNECTED) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    set_drive_base_enabled(true);
    while (1) {
        int left_pulse_cnt;
        int right_pulse_cnt;
        pcnt_unit_get_count(left_motor_handle.encoder.unit, &left_pulse_cnt);
        pcnt_unit_get_count(right_motor_handle.encoder.unit, &right_pulse_cnt);
        // ESP_LOGI(TAG,
        //          "Right encoder count %d, %d, %d",
        //          right_motor_handle.encoder.count,
        //          right_pulse_cnt,
        //          gpio_get_level(RIGHT_ENCODER_PIN));
        // ESP_LOGI(TAG,
        //          "Left encoder count %d, %d, %d",
        //          left_motor_handle.encoder.count,
        //          left_pulse_cnt,
        //          gpio_get_level(LEFT_ENCODER_PIN));
        ESP_LOGI(TAG,
                 "Right %f, %f, %f, %d",
                 right_motor_handle.cmd_velocity,
                 right_motor_handle.reported_velocity,
                 right_motor_handle.cmd_power,
                 right_motor_handle.encoder.count);
        ESP_LOGI(TAG,
                 "Left %f, %f, %f, %d",
                 left_motor_handle.cmd_velocity,
                 left_motor_handle.reported_velocity,
                 left_motor_handle.cmd_power,
                 left_motor_handle.encoder.count);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void drive_base_driver_init()
{
    // PWM SETUP
    init_motor_pwm();

    // MOTOR SETUP
    configure_motor(&left_motor_handle,
                    "left_motor",
                    LEFT_MOTOR_PWM_A_PIN,
                    LEFT_MOTOR_PWM_A_CHANNEL,
                    LEFT_MOTOR_PWM_B_PIN,
                    LEFT_MOTOR_PWM_B_CHANNEL,
                    LEFT_ENCODER_PIN);
    configure_motor(&right_motor_handle,
                    "right_motor",
                    RIGHT_MOTOR_PWM_A_PIN,
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

    joint_state_msg = sensor_msgs__msg__JointState__create();
    joint_state_publisher = register_publisher(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "wheel_states");

    // START TASK
    xTaskCreate(drive_base_driver_task,
                "drive_base_driver_task",
                DRIVE_BASE_TASK_SIZE,
                NULL,
                10,
                NULL);
}
