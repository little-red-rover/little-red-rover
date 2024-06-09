#pragma once

#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>

typedef struct
{
	rosidl_message_type_support_t *type_support;
	char *topic_name;
	rcl_publisher_t publisher;
} publisher_info;

typedef struct
{
	rosidl_message_type_support_t *type_support;
	char *topic_name;
	rcl_subscription_t subscription;
} subscription_info;

void init_publisher_mem();

void init_subscriptions_mem();

rcl_publisher_t *register_publisher(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name);

rcl_publisher_t *register_subscription(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name);

void create_pub_sub(const rcl_node_t *node);

void destroy_pub_sub(const rcl_node_t *node);
