#include "drive_base_driver.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#include "esp_timer.h"

#include <stdio.h>
#include <time.h>

#include "motor_driver.h"
#include "soc/soc.h"

#include "messages.pb.h"
#include "socket_mgr.h"

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

void cmd_vel_callback(void *cmd)
{
    TwistCmd twist_cmd = *((TwistCmd *)cmd);
    double v = twist_cmd.v;
    double w = twist_cmd.w;

    // https://control.ros.org/master/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot
    set_diff_drive((v - ((w * WHEEL_TRACK) / 2.0)) * (2.0 / WHEEL_DIAMETER),
                   (v + ((w * WHEEL_TRACK) / 2.0)) * (2.0 / WHEEL_DIAMETER));
}

void wheel_state_publish_timer_callback()
{
    // struct timespec ts;
    // clock_gettime(CLOCK_REALTIME, &ts);
    // wheel_state_msg.header.stamp.sec = ts.tv_sec;
    // wheel_state_msg.header.stamp.nanosec = ts.tv_nsec;
    //
    // wheel_state_msg.position.data[0] = left_motor_handle.encoder.position;
    // wheel_state_msg.velocity.data[0] = left_motor_handle.encoder.velocity;
    // wheel_state_msg.effort.data[0] =
    // (double)left_motor_handle.applied_effort;
    //
    // wheel_state_msg.position.data[1] = right_motor_handle.encoder.position;
    // wheel_state_msg.velocity.data[1] = right_motor_handle.encoder.velocity;
    // wheel_state_msg.effort.data[1] =
    // (double)right_motor_handle.applied_effort;
    //
    // if (get_uros_state() == AGENT_CONNECTED) {
    //     RCSOFTCHECK(rcl_publish(wheel_state_publisher, &wheel_state_msg,
    //     NULL));
    // }
}

static void drive_base_driver_task(void *arg)
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

    set_drive_base_enabled(true);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void drive_base_driver_init()
{
    // AGENT SETUP
    register_callback(cmd_vel_callback, eTwistCmd);

    // START TASK
    xTaskCreatePinnedToCore(drive_base_driver_task,
                            "drive_base_driver_task",
                            DRIVE_BASE_TASK_SIZE,
                            NULL,
                            10,
                            NULL,
                            APP_CPU_NUM);
}
