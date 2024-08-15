#include "include/socket_mgr.h"

#include "freertos/idf_additions.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "lwip/sockets.h"
#include "nvs.h"

#include "msgpack.h"

#define SOCKET_TX_TASK_STACK_SIZE 4096
#define SOCKET_RX_TASK_STACK_SIZE 4096
#define MAX_RX_QUEUES 2

static const char *TAG = "SOCKET_MGR";

static char AGENT_IP[16];
#define PORT 8001

esp_err_t get_agent_ip()
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        size_t l = sizeof(AGENT_IP);
        err = nvs_get_str(my_handle, "uros_ag_ip", AGENT_IP, &l);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(
                  TAG, "Retrieved IP (%s) for micro ros agent IP.", AGENT_IP);
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

static int socket_id;

typedef enum UDP_TX_PACKET_TYPES
{
    LIDAR_TX,
    IMU_TX,
} eUdpTxPacket_t;

typedef enum UDP_RX_PACKET_TYPES
{
    CMD_VEL_RX,
} eUdpRxPacket_t;

typedef struct UDP_COMMANDS
{
    eUdpTxPacket_t event_id;
    void *msg_data;
    size_t msg_size;
} UdpCmd_t;

QueueHandle_t tx_queue;

static void socket_tx_task(void *arg)
{
    // int err = sendto(sock,
    //                  payload,
    //                  strlen(payload),
    //                  0,
    //                  (struct sockaddr *)&dest_addr,
    //                  sizeof(dest_addr));
    // if (err < 0) {
    //     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    //     break;
    // }
    // ESP_LOGI(TAG, "Message sent");

    // UdpCmd_t tx_packet;
    // while (1) {
    //     // xQueueReceive(tx_queue, &tx_packet, portMAX_DELAY);
    //
    //     switch (tx_packet.event_id) {
    //         case LIDAR_TX:
    //             // Transmit over socket
    //             ESP_LOGI(TAG, "GOT TX PACKET");
    //             break;
    //         default:
    //             break;
    //     }
    // }

    while (1) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

size_t num_rx_queues;
QueueHandle_t rx_queues[MAX_RX_QUEUES];
static char rx_buffer[1500];

static void socket_rx_task(void *arg)
{
    // Recv from socket
    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    while (1) {
        int len = recvfrom(socket_id,
                           rx_buffer,
                           sizeof(rx_buffer) - 1,
                           0,
                           (struct sockaddr *)&source_addr,
                           &socklen);

        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            return;
        }
        // Data received
        else {
            rx_buffer[len] = 0; // Null-terminate whatever we received and
                                // treat like a string
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, AGENT_IP);
            ESP_LOGI(TAG, "%s", rx_buffer);
            if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                ESP_LOGI(TAG, "Received expected message, reconnecting");
            }
        }

        UdpCmd_t rx_packet;

        // Place onto relevant queue
        // switch (rx_packet.event_id) {
        //     case CMD_VEL_RX:
        //         ESP_LOGI(TAG, "GOT RX PACKET");
        //         break;
        //     default:
        //         break;
        // }
    }
}

void socket_mgr_init()
{
    while (get_agent_ip() != ESP_OK) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    struct sockaddr_in6 dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);

    socket_id = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (socket_id < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }

    int err = bind(socket_id, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    ESP_LOGI(TAG, "Socket created, communicating with %s:%d", AGENT_IP, PORT);

    xTaskCreatePinnedToCore(socket_tx_task,
                            "socket_tx_task",
                            SOCKET_TX_TASK_STACK_SIZE,
                            NULL,
                            10,
                            NULL,
                            APP_CPU_NUM);
    xTaskCreatePinnedToCore(socket_rx_task,
                            "socket_rx_task",
                            SOCKET_TX_TASK_STACK_SIZE,
                            NULL,
                            10,
                            NULL,
                            APP_CPU_NUM);
}
