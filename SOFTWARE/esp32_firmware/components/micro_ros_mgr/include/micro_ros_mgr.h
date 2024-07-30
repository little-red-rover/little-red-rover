#pragma once

#include <rcl/rcl.h>
#include <rclc/executor_handle.h>

enum states
{
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
};

void micro_ros_mgr_init();

enum states get_uros_state();

void add_subscription_callback(rcl_subscription_t *subscription,
							   void *msg,
							   rclc_subscription_callback_t callback);
