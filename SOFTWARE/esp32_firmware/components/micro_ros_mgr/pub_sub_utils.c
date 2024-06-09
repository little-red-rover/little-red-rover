#include "pub_sub_utils.h"

#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "micro_ros_mgr.h"

#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>

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

publisher_info *publishers;
size_t num_publishers = 0;

subscription_info *subscriptions;
size_t num_subscriptions = 0;

void init_publisher_mem()
{
	publishers = (publisher_info *)malloc(sizeof(publisher_info));
}

void init_subscriptions_mem()
{
	subscriptions = (subscription_info *)malloc(sizeof(subscription_info));
}

rcl_publisher_t *register_publisher(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name)
{
	if (num_publishers == 0)
		init_publisher_mem();
	else {
		publishers = (publisher_info *)realloc(
		  publishers, sizeof(publisher_info) * (num_publishers + 1));
	}
	publisher_info *cur_publisher = publishers + num_publishers;
	cur_publisher->type_support = type_support;
	cur_publisher->topic_name = topic_name;

	num_publishers++;
	return &(cur_publisher->publisher);
}

rcl_publisher_t *register_subscription(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name)
{
	if (num_subscriptions == 0)
		init_subscriptions_mem();
	else {
		subscriptions = (subscription_info *)realloc(
		  subscriptions, sizeof(subscription_info) * (num_subscriptions + 1));
	}
	subscription_info *cur_subscription = subscriptions + num_subscriptions;
	cur_subscription->type_support = type_support;
	cur_subscription->topic_name = topic_name;

	num_subscriptions++;
	return &(cur_subscription->subscription);
}

void create_pub_sub(const rcl_node_t *node)
{
	for (uint16_t i = 0; i < num_publishers; i++) {
		ESP_LOGI("create_pub_sub",
				 "Creating publisher %s",
				 ((publishers + i)->topic_name));
		RCCHECK(rclc_publisher_init_best_effort(&((publishers + i)->publisher),
												node,
												(publishers + i)->type_support,
												(publishers + i)->topic_name));
	}
	for (uint16_t i = 0; i < num_subscriptions; i++) {
		ESP_LOGI("create_pub_sub",
				 "Creating subscriber %s",
				 ((subscriptions + i)->topic_name));
		RCCHECK(
		  rclc_publisher_init_best_effort(&((subscriptions + i)->subscription),
										  node,
										  (publishers + i)->type_support,
										  (publishers + i)->topic_name));
	}
}

void destroy_pub_sub(const rcl_node_t *node)
{
	for (size_t i = 0; i < num_publishers; i++) {
		ESP_LOGI("destroy_pub_sub",
				 "Destroying publisher %s",
				 ((publishers + i)->topic_name));
		rcl_publisher_fini(&((publishers + i)->publisher), node);
	}
	for (size_t i = 0; i < num_subscriptions; i++) {
		ESP_LOGI("destroy_pub_sub",
				 "Destroying subscription %s",
				 ((subscriptions + i)->topic_name));
		rcl_publisher_fini(&((subscriptions + i)->subscription), node);
	}
}