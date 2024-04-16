idf_component_register(SRCS "wifi_mgr.c" 
                    INCLUDE_DIRS include
                    PRIV_REQUIRES rest_server esp_netif spiffs fatfs nvs_flash esp_wifi wifi_provisioning dns_server)
