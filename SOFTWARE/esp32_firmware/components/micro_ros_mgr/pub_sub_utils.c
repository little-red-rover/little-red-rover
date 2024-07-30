#include "pub_sub_utils.h"

#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "drive_base_driver.h"
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

#define MAX_PUBLISHERS 10
#define MAX_SUBSCRIBERS 10

publisher_info *publishers[MAX_PUBLISHERS];
size_t num_publishers = 0;

subscription_info *subscriptions[MAX_SUBSCRIBERS];
size_t num_subscriptions = 0;

size_t get_num_subscriptions()
{
    return num_subscriptions;
}

rcl_publisher_t *register_publisher(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name)
{
    publishers[num_publishers] = malloc(sizeof(publisher_info));
    publishers[num_publishers]->publisher =
      rcl_get_zero_initialized_publisher();
    publishers[num_publishers]->type_support = type_support;
    publishers[num_publishers]->topic_name = topic_name;

    num_publishers++;
    return &(publishers[num_publishers - 1]->publisher);
}

rcl_subscription_t *register_subscription(
  const rosidl_message_type_support_t *type_support,
  const char *topic_name,
  void *msg,
  rclc_subscription_callback_t callback)
{
    subscriptions[num_subscriptions] = malloc(sizeof(subscription_info));
    subscriptions[num_subscriptions]->subscription =
      rcl_get_zero_initialized_subscription();
    subscriptions[num_subscriptions]->type_support = type_support;
    subscriptions[num_subscriptions]->topic_name = topic_name;
    subscriptions[num_subscriptions]->msg = msg;
    subscriptions[num_subscriptions]->callback = callback;

    num_subscriptions++;
    return &(subscriptions[num_subscriptions - 1]->subscription);
}

void create_pub_sub(rcl_node_t *node)
{
    for (uint16_t i = 0; i < num_publishers; i++) {
        ESP_LOGI("create_pub_sub",
                 "Creating publisher %s",
                 (publishers[i]->topic_name));
        RCCHECK(rclc_publisher_init_best_effort(&(publishers[i]->publisher),
                                                node,
                                                publishers[i]->type_support,
                                                publishers[i]->topic_name));
    }
    for (uint16_t i = 0; i < num_subscriptions; i++) {
        ESP_LOGI("create_pub_sub",
                 "Creating subscriber %s",
                 (subscriptions[i]->topic_name));
        RCCHECK(
          rclc_subscription_init_default(&(subscriptions[i]->subscription),
                                         node,
                                         subscriptions[i]->type_support,
                                         subscriptions[i]->topic_name));
    }
}

void create_sub_callbacks(rclc_executor_t *executor)
{
    for (uint16_t i = 0; i < num_subscriptions; i++) {
        ESP_LOGI("create_sub_callbacks",
                 "Creating callback for subscriber %s",
                 subscriptions[i]->topic_name);

        RCCHECK(
          rclc_executor_add_subscription(executor,
                                         &(subscriptions[i]->subscription),
                                         (subscriptions[i]->msg),
                                         subscriptions[i]->callback,
                                         ON_NEW_DATA));
    }
}

void destroy_pub_sub(rcl_node_t *node)
{
    for (size_t i = 0; i < num_publishers; i++) {
        ESP_LOGI("destroy_pub_sub",
                 "Destroying publisher %s",
                 (publishers[i]->topic_name));
        RCCHECK(rcl_publisher_fini(&(publishers[i]->publisher), node));
    }
    for (size_t i = 0; i < num_subscriptions; i++) {
        ESP_LOGI("destroy_pub_sub",
                 "Destroying subscription %s",
                 subscriptions[i]->topic_name);
        RCCHECK(rcl_subscription_fini(&(subscriptions[i]->subscription), node));
    }
}
