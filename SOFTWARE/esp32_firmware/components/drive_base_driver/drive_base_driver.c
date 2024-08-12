#include "drive_base_driver.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/publisher.h>
#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rclc/executor_handle.h>

#include "micro_ros_utilities/type_utilities.h"

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>
#include <time.h>

#include "micro_ros_mgr.h"
#include "motor_driver.h"
#include "pub_sub_utils.h"

#define DRIVE_BASE_TASK_SIZE (4096)

// PIN DEFINITIONS
#define MOTOR_ENABLE 5

#define LEFT_MOTOR_PWM_A_PIN 7
#define LEFT_MOTOR_PWM_A_CHANNEL 0
#define LEFT_MOTOR_PWM_B_PIN 6
#define LEFT_MOTOR_PWM_B_CHANNEL 1
#define RIGHT_MOTOR_PWM_A_PIN 9
#define RIGHT_MOTOR_PWM_A_CHANNEL 2
#define RIGHT_MOTOR_PWM_B_PIN 8
#define RIGHT_MOTOR_PWM_B_CHANNEL 3

#define LEFT_ENCODER_PIN_A 14
#define LEFT_ENCODER_PIN_B 13
#define RIGHT_ENCODER_PIN_A 4
#define RIGHT_ENCODER_PIN_B 3

// Both in meters
#define WHEEL_DIAMETER 0.060960
#define WHEEL_TRACK 0.11439

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

static const char *TAG = "drive_base_driver";

// PUBLISHERS
#define PUBLISHER_LOOP_PERIOD_MS 100

sensor_msgs__msg__JointState wheel_state_msg;
rcl_publisher_t *wheel_state_publisher;

// SUBSCRIPTIONS
geometry_msgs__msg__Twist *cmd_vel_msg;
rcl_subscription_t *cmd_vel_subscription;

// MOTORS
motor_handle_t left_motor_handle;
motor_handle_t right_motor_handle;

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

void wheel_state_publish_timer_callback()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    wheel_state_msg.header.stamp.sec = ts.tv_sec;
    wheel_state_msg.header.stamp.nanosec = ts.tv_nsec;

    wheel_state_msg.position.data[0] = left_motor_handle.encoder.position;
    wheel_state_msg.velocity.data[0] = left_motor_handle.encoder.velocity;
    wheel_state_msg.effort.data[0] = left_motor_handle.applied_effort;

    wheel_state_msg.position.data[1] = right_motor_handle.encoder.position;
    wheel_state_msg.velocity.data[1] = right_motor_handle.encoder.velocity;
    wheel_state_msg.effort.data[1] = right_motor_handle.applied_effort;

    if (get_uros_state() == AGENT_CONNECTED) {
        RCSOFTCHECK(rcl_publish(wheel_state_publisher, &wheel_state_msg, NULL));
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
                    LEFT_ENCODER_PIN_B,
                    false);

    configure_motor(&right_motor_handle,
                    "right_motor",
                    RIGHT_MOTOR_PWM_A_PIN,
                    RIGHT_MOTOR_PWM_A_CHANNEL,
                    RIGHT_MOTOR_PWM_B_PIN,
                    RIGHT_MOTOR_PWM_B_CHANNEL,
                    RIGHT_ENCODER_PIN_A,
                    RIGHT_ENCODER_PIN_B,
                    true);

    esp_timer_create_args_t pub_timer_args = {
        .callback = wheel_state_publish_timer_callback,
        .name = "wheel_state_publish_timer"
    };
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

    static micro_ros_utilities_memory_conf_t conf = { 0 };

    conf.max_string_capacity = 15;
    conf.max_ros2_type_sequence_capacity = 2;
    conf.max_basic_type_sequence_capacity = 2;

    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &wheel_state_msg,
      conf);

    wheel_state_msg.header.frame_id.size = 9;
    wheel_state_msg.header.frame_id.capacity = 10;
    wheel_state_msg.header.frame_id.data = "robot_body";

    wheel_state_msg.name.size = 2;
    wheel_state_msg.name.capacity = 2;

    wheel_state_msg.name.data[0].size = 10;
    wheel_state_msg.name.data[0].capacity = 11;
    wheel_state_msg.name.data[0].data = "wheel_left";

    wheel_state_msg.name.data[1].size = 11;
    wheel_state_msg.name.data[1].capacity = 12;
    wheel_state_msg.name.data[1].data = "wheel_right";

    wheel_state_msg.position.size = 2;
    wheel_state_msg.position.capacity = 2;

    wheel_state_msg.velocity.size = 2;
    wheel_state_msg.velocity.capacity = 2;

    wheel_state_msg.effort.size = 2;
    wheel_state_msg.effort.capacity = 2;

    wheel_state_publisher = register_publisher(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "joint_states");

    // START TASK
    xTaskCreate(drive_base_driver_task,
                "drive_base_driver_task",
                DRIVE_BASE_TASK_SIZE,
                NULL,
                10,
                NULL);
}
