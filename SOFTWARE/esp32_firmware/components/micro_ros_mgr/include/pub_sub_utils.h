#pragma once

#include <rcl/rcl.h>
#include <rcl/timer.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>

typedef struct
{
    const rosidl_message_type_support_t *type_support;
    const char *topic_name;
    rcl_publisher_t publisher;
} publisher_info;

typedef struct
{
    const rosidl_message_type_support_t *type_support;
    const char *topic_name;
    rcl_subscription_t subscription;
    void *msg;
    rclc_subscription_callback_t callback;

} subscription_info;

typedef struct
{
    rcl_timer_t timer;
    unsigned int timeout_ns;
    rcl_timer_callback_t callback;

} timer_info;

size_t get_num_handles();

rcl_publisher_t *register_publisher(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name);

rcl_subscription_t *register_subscription(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name,
  void *msg,
  rclc_subscription_callback_t callback);

void register_timer(rcl_timer_callback_t callback, unsigned int timeout_ns);

void create_pub_sub(rcl_node_t *node, rclc_support_t *support);

void create_callbacks(rclc_executor_t *executor);

void destroy_pub_sub(rcl_node_t *node);
