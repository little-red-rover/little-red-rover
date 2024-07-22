#pragma once

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/executor_handle.h>

#include "driver/pulse_cnt.h"
#include "hal/ledc_types.h"
#include "pid_ctrl.h"
#include "soc/gpio_num.h"

/*
 * Represents an encoder.
 */
typedef struct
{
    pcnt_channel_handle_t channel;
    pcnt_unit_handle_t unit;
    int count;
} encoder_handle_t;

/*
 * Get an encoders count.
 */
static int get_encoder_count(encoder_handle_t encoder)
{
    int ret;
    pcnt_unit_get_count(encoder.unit, &ret);
    return ret;
};

/*
 * Represents a motor.
 */
typedef struct
{
    ledc_channel_t chan_a;
    ledc_channel_t chan_b;
    gpio_num_t enable_pin;
    float cmd_velocity;
    float reported_velocity;
    float cmd_power;
    encoder_handle_t encoder;
    pid_ctrl_block_handle_t pid_controller;
    esp_timer_handle_t pid_timer;
    esp_timer_create_args_t pid_args;
} motor_handle_t;

/*
 * Enable a motor.
 */
void set_motor_enabled(motor_handle_t *motor, bool enable);

/*
 * Set a motors velocity in rad/s.
 */
void set_motor_velocity(motor_handle_t *motor, float velocity);

/*
 * Configure a GPIO pin for PWM control.
 */
void configure_pwm(ledc_channel_t channel, int gpio);

/*
 * Configure a motor for use with the given control pins, pwm channels, and
 * encoder.
 */
void configure_motor(motor_handle_t *motor,
                     char *motor_name,
                     gpio_num_t pwm_a_pin,
                     ledc_channel_t pwm_a_chan,
                     gpio_num_t pwm_b_pin,
                     ledc_channel_t pwm_b_chan,
                     gpio_num_t encoder_pin);

/*
 * Initialize the PWM peripheral for use with motors.
 */
void init_motor_pwm();
