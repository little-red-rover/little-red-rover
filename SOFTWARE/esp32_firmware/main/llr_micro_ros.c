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
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_wifi_default.h"

#include "rest_server.h"
#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>

static void wifi_prov_event_handler(void* arg, esp_event_base_t event_base,
                          int event_id, void* event_data)
{
	static const char *TAG = "wifi_prov_event";
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG, "Received Wi-Fi credentials"
                         "\n\tSSID     : %s\n\tPassword : %s",
                         (const char *) wifi_sta_cfg->ssid,
                         (const char *) wifi_sta_cfg->password);
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                         "\n\tPlease reset to factory and retry provisioning",
                         (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                         "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning successful");
                break;
            case WIFI_PROV_END:
                /* De-initialize manager once provisioning is finished */
                wifi_prov_mgr_deinit();
                break;
            default:
                break;
        }
    }
}

esp_err_t init_fs(void)
{
	static const char *TAG = "fs_init";
    esp_vfs_spiffs_conf_t conf = {.base_path = "/www",
                                  .partition_label = NULL,
                                  .max_files = 5,
                                  .format_if_mount_failed = false};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

void app_main(void)
{
    /* NVS INIT */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

	/* FILE SYSTEM INIT */
    ESP_ERROR_CHECK(init_fs());

	/* EVENT LOOP INIT */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

	/* WEBSERVER INIT */
    ESP_ERROR_CHECK(esp_netif_init());

	esp_netif_create_default_wifi_ap();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(rest_server_start("/www"));

	/* PROVISIONING INIT */
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_prov_event_handler, NULL));

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE
    };
	ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

	wifi_prov_scheme_softap_set_httpd_handle((void*)rest_server_get_httpd_handle());

	ESP_ERROR_CHECK( wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1, NULL, "hello_world", "hello_world") );
}