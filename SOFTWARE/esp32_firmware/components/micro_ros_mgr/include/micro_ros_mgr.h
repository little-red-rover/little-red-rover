#pragma once

#include <rcl/rcl.h>

void micro_ros_mgr_init();

rcl_publisher_t *register_publisher(
  const rosidl_message_type_support_t *type_support,
  const char *name);