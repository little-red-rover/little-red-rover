#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "esp_wifi_default.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "esp_mac.h"

#include "wifi_mgr.h"

static esp_ip4_addr_t STA_IP;

static const char *TAG = "wifi_mgr";
static void wifi_prov_event_handler(void *arg,
									esp_event_base_t event_base,
									int event_id,
									void *event_data)
{
	const char *TAG = "wifi_prov_event";
	if (event_base == WIFI_PROV_EVENT) {
		switch (event_id) {
			case WIFI_PROV_START:
				ESP_LOGI(TAG, "Provisioning started");
				break;
			case WIFI_PROV_CRED_RECV: {
				wifi_sta_config_t *wifi_sta_cfg =
				  (wifi_sta_config_t *)event_data;
				ESP_LOGI(TAG,
						 "Received Wi-Fi credentials"
						 "\n\tSSID     : %s\n\tPassword : %s",
						 (const char *)wifi_sta_cfg->ssid,
						 (const char *)wifi_sta_cfg->password);
				break;
			}
			case WIFI_PROV_CRED_FAIL: {
				wifi_prov_sta_fail_reason_t *reason =
				  (wifi_prov_sta_fail_reason_t *)event_data;
				ESP_LOGE(TAG,
						 "Provisioning failed!\n\tReason : %s"
						 "\n\tPlease reset to factory and retry provisioning",
						 (*reason == WIFI_PROV_STA_AUTH_ERROR)
						   ? "Wi-Fi station authentication failed"
						   : "Wi-Fi access-point not found");
				vTaskDelay(5000 / portTICK_PERIOD_MS);
				wifi_prov_mgr_reset_provisioning();
				// Delay so esp_prov.py has time to read reason for failure.
				esp_restart();
				break;
			}
			case WIFI_PROV_CRED_SUCCESS:
				ESP_LOGI(TAG, "Provisioning successful");
				break;
			case WIFI_PROV_END:
				/* De-initialize manager once provisioning is finished */
				// wifi_prov_mgr_deinit();
				break;
			default:
				break;
		}
	}
}

esp_err_t init_fs(void)
{
	const char *TAG = "fs_init";
	esp_vfs_spiffs_conf_t conf = { .base_path = "/www",
								   .partition_label = NULL,
								   .max_files = 5,
								   .format_if_mount_failed = false };
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(
			  TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
		}
		return ESP_FAIL;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,
				 "Failed to get SPIFFS partition information (%s)",
				 esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
	}
	return ESP_OK;
}
/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG_AP = "WiFi SoftAP";
static const char *TAG_STA = "WiFi Sta";

static int s_retry_num = 0;

/* FreeRTOS event group to signal when we are connected/disconnected */
static EventGroupHandle_t s_wifi_event_group;

static void wifi_event_handler(void *arg,
							   esp_event_base_t event_base,
							   int32_t event_id,
							   void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t *event =
		  (wifi_event_ap_staconnected_t *)event_data;
		ESP_LOGI(TAG_AP,
				 "Station " MACSTR " joined, AID=%d",
				 MAC2STR(event->mac),
				 event->aid);
	} else if (event_base == WIFI_EVENT &&
			   event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t *event =
		  (wifi_event_ap_stadisconnected_t *)event_data;
		ESP_LOGI(TAG_AP,
				 "Station " MACSTR " left, AID=%d",
				 MAC2STR(event->mac),
				 event->aid);
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
		ESP_LOGI(TAG_STA, "Station started");
	} else if (event_base == WIFI_EVENT &&
			   event_id == WIFI_EVENT_STA_DISCONNECTED) {
		wifi_err_reason_t reason =
		  (*(wifi_event_sta_disconnected_t *)event_data).reason;
		if (reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
			reason == WIFI_REASON_NO_AP_FOUND ||
			reason == WIFI_REASON_HANDSHAKE_TIMEOUT ||
			reason == WIFI_REASON_AUTH_EXPIRE ||
			reason == WIFI_REASON_AUTH_FAIL) {
			wifi_prov_mgr_reset_provisioning();
			ESP_LOGI(
			  TAG_STA,
			  "Authentication failed. Resetting provisioning and restarting.");
			esp_restart();
		}
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		STA_IP = event->ip_info.ip;
		ESP_LOGI(TAG_STA, "Got IP:" IPSTR, IP2STR(&STA_IP));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

static esp_err_t get_ip_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Got request for IP. Sending " IPSTR, IP2STR(&STA_IP));
	char *ip_str = (char *)malloc(50 * sizeof(char));
	snprintf(ip_str, 50, IPSTR, IP2STR(&STA_IP));
	httpd_resp_send(req, ip_str, HTTPD_RESP_USE_STRLEN);
	free(ip_str);
	return ESP_OK;
};

// https://github.com/espressif/esp-idf/issues/4863#issuecomment-594357432
esp_err_t get_remote_ip(httpd_req_t *req, struct sockaddr_in6 *addr_in)
{
	int s = httpd_req_to_sockfd(req);
	socklen_t addrlen = sizeof(*addr_in);
	if (lwip_getpeername(s, (struct sockaddr *)addr_in, &addrlen) != -1) {
		return ESP_OK;
	} else {
		ESP_LOGE(TAG, "Error getting peer's IP/port");
		return ESP_FAIL;
	}
}

static esp_err_t get_agent_ip_handler(httpd_req_t *req)
{
	char *micro_ros_agent_ip;
	struct sockaddr_in6 addr_in;
	if (get_remote_ip(req, &addr_in) == ESP_OK) {
		micro_ros_agent_ip = inet_ntoa(addr_in.sin6_addr.un.u32_addr[3]);
		ESP_LOGI(TAG, "Remote IP is %s", micro_ros_agent_ip);
	} else {
		return ESP_FAIL;
	}

	httpd_resp_sendstr(req, micro_ros_agent_ip);

	nvs_handle_t my_handle;
	esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} else {
		err = nvs_set_str(my_handle, "uros_ag_ip", micro_ros_agent_ip);
		if (err != ESP_OK) {
			ESP_LOGE(TAG,
					 "Error (%s) writing uros agent ip to flash.",
					 esp_err_to_name(err));

			return ESP_FAIL;
		}
		err = nvs_commit(my_handle);
		if (err != ESP_OK) {
			ESP_LOGE(
			  TAG, "Error (%s) commiting flash memory.", esp_err_to_name(err));

			return ESP_FAIL;
		}

		nvs_close(my_handle);
		ESP_LOGI(TAG, "Saved agent IP to flash.");
	}

	return ESP_OK;
}

/* Save the server handle here */
static int *_server_context = NULL;
static httpd_handle_t _server = NULL;

static esp_err_t start_webserver()
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.uri_match_fn = httpd_uri_match_wildcard;

	ESP_LOGI(TAG, "Starting HTTP Server");
	ESP_ERROR_CHECK(httpd_start(&_server, &config));

	/* URI handler for getting web server files */
	httpd_uri_t common_get_uri = { .uri = "/get-ip",
								   .method = HTTP_GET,
								   .handler = get_ip_handler,
								   .user_ctx = "" };
	httpd_uri_t agent_ip_get_uri = { .uri = "/set-agent-ip",
									 .method = HTTP_GET,
									 .handler = get_agent_ip_handler,
									 .user_ctx = "" };
	httpd_register_uri_handler(_server, &common_get_uri);
	httpd_register_uri_handler(_server, &agent_ip_get_uri);
	httpd_register_err_handler(_server, HTTPD_404_NOT_FOUND, get_ip_handler);

	return ESP_OK;
}

void wifi_mgr_init()
{
	/* NVS INIT */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
		ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}

	/* EVENT LOOP INIT */
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	/* WIFI INIT */
	ESP_ERROR_CHECK(esp_netif_init());

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

	wifi_config_t wifi_config = {
        .ap = {
            .ssid = "little_red_rover",
            .ssid_len = 0,
            .channel = 0,
            .password = "",
            .max_connection = 14,
            .authmode = WIFI_AUTH_OPEN,
            .pmf_cfg = {
                    .required = true,
            },
        },
    };

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

	ESP_LOGI("AP", "ESP_WIFI_MODE_AP");
	esp_netif_t *esp_netif_ap = esp_netif_create_default_wifi_ap();

	ESP_LOGI("STA", "ESP_WIFI_MODE_STA");
	esp_netif_t *esp_netif_sta = esp_netif_create_default_wifi_sta();

	ESP_ERROR_CHECK(start_webserver());
	//    printf("webserver handle: %d \n", *(unsigned int
	//    *)_server_handle);
	// start_webserver();

	/* PROVISIONING INIT */
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_event_handler_register(
	  WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_prov_event_handler, NULL));

	wifi_prov_mgr_config_t config = { .scheme = wifi_prov_scheme_softap,
									  .scheme_event_handler =
										WIFI_PROV_EVENT_HANDLER_NONE };
	ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
	wifi_prov_scheme_softap_set_httpd_handle((void *)(&_server));

	// wifi_prov_mgr_reset_provisioning();

	// This will be tied to a button
	ESP_ERROR_CHECK(wifi_prov_mgr_disable_auto_stop(0));

	/*  */
	bool provisioned = false;
	ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

	if (!provisioned) {
		ESP_ERROR_CHECK(esp_event_handler_register(
		  IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

		ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
		  WIFI_PROV_SECURITY_1, NULL, "little_red_rover", NULL));
	} else {
		ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

		ESP_ERROR_CHECK(esp_event_handler_register(
		  WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
		ESP_ERROR_CHECK(esp_event_handler_register(
		  IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

		ESP_ERROR_CHECK(esp_wifi_start());
	}

	/* Wait for Wi-Fi connection */
	xEventGroupWaitBits(
	  s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
	ESP_LOGI(TAG, "Device is provisioned and connected.");
}