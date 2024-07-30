#pragma once

#include <rcl/rcl.h>
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

size_t get_num_subscriptions();

rcl_publisher_t *register_publisher(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name);

rcl_subscription_t *register_subscription(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name,
  void *msg,
  rclc_subscription_callback_t callback);

void create_pub_sub(rcl_node_t *node);

void create_sub_callbacks(rclc_executor_t *executor);

void destroy_pub_sub(rcl_node_t *node);
