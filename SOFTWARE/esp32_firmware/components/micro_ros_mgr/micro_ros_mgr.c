// Based on
// https://github.com/micro-ROS/micro_ros_arduino/blob/iron/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino

#include <rcl/types.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "sensor_msgs/msg/laser_scan.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#include "micro_ros_mgr.h"

#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>

#include "pub_sub_utils.h"
#include "wifi_mgr.h"

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
#define EXECUTE_EVERY_N_MS(MS, X)                                              \
	do {                                                                       \
		static volatile int64_t init = -1;                                     \
		if (init == -1) {                                                      \
			init = uxr_millis();                                               \
		}                                                                      \
		if (uxr_millis() - init > MS) {                                        \
			X;                                                                 \
			init = uxr_millis();                                               \
		}                                                                      \
	} while (0)

static const char *TAG = "micro_ros_mgr";
static char MICRO_ROS_AGENT_IP[16];

enum states state;

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rmw_init_options_t rmw_options;
rmw_context_t rmw_context;
std_msgs__msg__Int32 msg;

esp_err_t get_micro_ros_agent_ip()
{
	nvs_handle_t my_handle;
	esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} else {
		size_t l = sizeof(MICRO_ROS_AGENT_IP);
		err = nvs_get_str(my_handle, "uros_ag_ip", MICRO_ROS_AGENT_IP, &l);
		switch (err) {
			case ESP_OK:
				ESP_LOGI(TAG,
						 "Retrieved IP (%s) for micro ros agent IP.",
						 MICRO_ROS_AGENT_IP);
				break;
			case ESP_ERR_NVS_NOT_FOUND:
				ESP_LOGI(TAG, "Agent IP has not been set.");
				nvs_close(my_handle);
				return ESP_FAIL;
				break;
			default:
				ESP_LOGI(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
				nvs_close(my_handle);
				return ESP_FAIL;
		}

		nvs_close(my_handle);
	}
	return ESP_OK;
}

void init_middleware()
{
	allocator = rcl_get_default_allocator();

	init_options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&init_options, allocator));

	rmw_options = *rcl_init_options_get_rmw_init_options(&init_options);

	RCCHECK(rmw_uros_options_set_udp_address(
	  MICRO_ROS_AGENT_IP, "8001", &rmw_options));
}

rcl_ret_t create_entities()
{
	// create init_options
	if (rclc_support_init_with_options(
		  &support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
		return RCL_RET_ERROR;
	}

	// create node
	RCCHECK(rclc_node_init_default(&node, "little_red_rover", "", &support));

	// create executor
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// create publishers / subscriptions
	create_pub_sub(&node);

	return RCL_RET_OK;
}

rcl_ret_t destroy_entities()
{
	rmw_shutdown(&rmw_context);
	rmw_context = *rcl_context_get_rmw_context(&support.context);
	(void)rmw_uros_set_context_entity_destroy_session_timeout(&rmw_context, 0);

	destroy_pub_sub(&node);
	rcl_timer_fini(&timer);
	rcl_node_fini(&node);
	rcl_shutdown(&support.context);
	rclc_executor_fini(&executor);
	rclc_support_fini(&support);

	return RCL_RET_OK;
}

enum states get_uros_state()
{
	return state;
}

void micro_ros_task(void *arg)
{
	state = WAITING_AGENT;

	msg.data = 0;

	while (get_micro_ros_agent_ip() != ESP_OK) {
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
	init_middleware();

	while (true) {
		switch (state) {
			case WAITING_AGENT:
				EXECUTE_EVERY_N_MS(
				  500, rmw_context = rmw_get_zero_initialized_context();
				  rmw_init(&rmw_options, &rmw_context);
				  state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5))
							? AGENT_AVAILABLE
							: WAITING_AGENT;
				  if (state == WAITING_AGENT) {
					  ESP_LOGI(TAG, "Agent unavailable, trying again.");
					  get_micro_ros_agent_ip();
					  RCCHECK(rmw_uros_options_set_udp_address(
						MICRO_ROS_AGENT_IP, "8001", &rmw_options));
				  } else { rmw_shutdown(&rmw_context); });
				break;
			case AGENT_AVAILABLE:
				state = (RCL_RET_OK == create_entities()) ? AGENT_CONNECTED
														  : WAITING_AGENT;
				if (state == WAITING_AGENT) {
					ESP_LOGI(TAG, "Failed to create micro_ros structures.");
					destroy_entities();
				};
				break;
			case AGENT_CONNECTED:
				EXECUTE_EVERY_N_MS(200,
								   state =
									 (RMW_RET_OK == rmw_uros_ping_agent(100, 5))
									   ? AGENT_CONNECTED
									   : AGENT_DISCONNECTED;);

				if (state == AGENT_CONNECTED) {
					rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
					usleep(10000);
				}
				break;
			case AGENT_DISCONNECTED:
				destroy_entities();
				state = WAITING_AGENT;
				ESP_LOGI(TAG, "Agent disconnected, attempting to reconnect.");
				init_middleware();
				break;
			default:
				break;
		}
	}

	vTaskDelete(NULL);
}

void micro_ros_mgr_init()
{
	// pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	xTaskCreate(micro_ros_task,
				"uros_task",
				CONFIG_MICRO_ROS_APP_STACK,
				NULL,
				CONFIG_MICRO_ROS_APP_TASK_PRIO,
				NULL);
}