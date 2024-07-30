#include "drive_base_driver.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <geometry_msgs/msg/detail/pose__struct.h>
#include <nav_msgs/msg/detail/odometry__functions.h>
#include <nav_msgs/msg/detail/odometry__struct.h>
#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rclc/executor_handle.h>

#include <geometry_msgs/msg/detail/twist__functions.h>
#include <geometry_msgs/msg/detail/twist__struct.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <stdio.h>
#include <time.h>

#include "micro_ros_mgr.h"
#include "motor_driver.h"
#include "pub_sub_utils.h"

#define DRIVE_BASE_TASK_SIZE (4096)

// PIN DEFINITIONS
#define MOTOR_ENABLE 5

#define LEFT_MOTOR_PWM_A_PIN 6
#define LEFT_MOTOR_PWM_A_CHANNEL 0
#define LEFT_MOTOR_PWM_B_PIN 7
#define LEFT_MOTOR_PWM_B_CHANNEL 1
#define RIGHT_MOTOR_PWM_A_PIN 8
#define RIGHT_MOTOR_PWM_A_CHANNEL 2
#define RIGHT_MOTOR_PWM_B_PIN 9
#define RIGHT_MOTOR_PWM_B_CHANNEL 3

#define LEFT_ENCODER_PIN_A 43
#define LEFT_ENCODER_PIN_B 41
#define RIGHT_ENCODER_PIN_A 12
#define RIGHT_ENCODER_PIN_B 14

// Both in meters
#define WHEEL_DIAMETER 0.060960
#define WHEEL_TRACK 0.11439

static const char *TAG = "drive_base_driver";

// PUBLISHERS
#define PUBLISHER_LOOP_PERIOD_MS 15
static nav_msgs__msg__Odometry *odom_msg;
static rcl_publisher_t *odom_publisher;

// SUBSCRIPTIONS
static geometry_msgs__msg__Twist *cmd_vel_msg;
static rcl_subscription_t *cmd_vel_subscription;

// MOTORS
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

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
    double v = msg->linear.x;
    double w = msg->angular.z;

    // https://control.ros.org/master/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot
    set_diff_drive((v - ((w * WHEEL_TRACK) / 2.0)) * (2.0 / WHEEL_DIAMETER),
                   (v + ((w * WHEEL_TRACK) / 2.0)) * (2.0 / WHEEL_DIAMETER));
}

void odom_publish_timer_callback()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    odom_msg->header.stamp.sec = ts.tv_sec;
    odom_msg->header.stamp.nanosec = ts.tv_nsec;
    odom_msg->header.frame_id.data = "base_link";
    odom_msg->header.frame_id.size = 9;
    odom_msg->header.frame_id.capacity = 9;

    odom_msg->child_frame_id.data = "base_link";
    odom_msg->child_frame_id.size = 9;
    odom_msg->child_frame_id.capacity = 9;

    odom_msg->pose.pose.position =
      (geometry_msgs__msg__Point){ .x = 0.0, .y = 0.0, .z = 0.0 };
    odom_msg->pose.pose.orientation =
      (geometry_msgs__msg__Quaternion){ .x = 0.0, .y = 0.0, .z = 0.0 };
    // odom_msg.pose.covariance  { 0.0 } };

    if (get_uros_state() == AGENT_CONNECTED) {
        rcl_ret_t ret = rcl_publish(odom_publisher, odom_msg, NULL);
        if (ret != RCL_RET_OK) {
            ESP_LOGI(TAG, "Error publishing odom: %d.", (int)ret);
        }
    }
}

static void drive_base_driver_task(void *arg)
{
    while (get_uros_state() != AGENT_CONNECTED) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    // PWM SETUP
    init_motor_pwm();

    // MOTOR SETUP
    configure_motor(&left_motor_handle,
                    "left_motor",
                    LEFT_MOTOR_PWM_A_PIN,
                    LEFT_MOTOR_PWM_A_CHANNEL,
                    LEFT_MOTOR_PWM_B_PIN,
                    LEFT_MOTOR_PWM_B_CHANNEL,
                    LEFT_ENCODER_PIN_A,
                    LEFT_ENCODER_PIN_B);
    configure_motor(&right_motor_handle,
                    "right_motor",
                    RIGHT_MOTOR_PWM_A_PIN,
                    RIGHT_MOTOR_PWM_A_CHANNEL,
                    RIGHT_MOTOR_PWM_B_PIN,
                    RIGHT_MOTOR_PWM_B_CHANNEL,
                    RIGHT_ENCODER_PIN_A,
                    RIGHT_ENCODER_PIN_B);

    esp_timer_create_args_t pub_timer_args = { .callback =
                                                 odom_publish_timer_callback,
                                               .name = "odom_pubish_timer" };
    esp_timer_handle_t pub_timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&pub_timer_args, &pub_timer_handle));
    esp_timer_start_periodic(pub_timer_handle, PUBLISHER_LOOP_PERIOD_MS * 1000);

    set_drive_base_enabled(true);

    while (1) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void drive_base_driver_init()
{
    // MICRO ROS SETUP
    cmd_vel_msg = geometry_msgs__msg__Twist__create();
    cmd_vel_subscription = register_subscription(
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel",
      cmd_vel_msg,
      &cmd_vel_callback);

    odom_msg = nav_msgs__msg__Odometry__create();
    odom_publisher = register_publisher(
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");

    // START TASK
    xTaskCreate(drive_base_driver_task,
                "drive_base_driver_task",
                DRIVE_BASE_TASK_SIZE,
                NULL,
                10,
                NULL);
}
