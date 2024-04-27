#pragma once

#include <rcl/rcl.h>

enum states
{
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
};

void micro_ros_mgr_init();

rcl_publisher_t *register_publisher(
  const rosidl_message_type_support_t *type_support,
  const char *name);

enum states get_uros_state();