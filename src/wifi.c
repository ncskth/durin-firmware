#include <mdns.h>
#include <sys/socket.h>
#include <netdb.h>
#include <esp_netif.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/gpio_hal.h>
#include <esp_wifi.h>

#include "pt.h"
#include "durin.h"

#define WIFI_DELAY_NO_CLIENT 1000
#define WIFI_DELAY_NO_CONNECTION 5000
#define WIFI_DELAY_READ 50

int8_t tcp_server_socket_id;
int8_t tcp_client_socket_id;
int8_t udp_client_socket_id;

static void tcp_server_task(void *arg) {
    struct sockaddr_in tcpServerAddr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(1337)
    };
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    struct sockaddr_in remote_addr;
	unsigned int socklen;
	socklen = sizeof(remote_addr);
    uint8_t rx_buf[32], tx_buf[32];

    int socket_id = socket(AF_INET, SOCK_STREAM, 0);
    bind(socket_id, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr));
    listen(socket_id, 2);
    mdns_init();
    mdns_hostname_set("durin");
    fcntl(socket_id, F_SETFL, O_NONBLOCK);
    // accept client loop
    while (1) {

        int client_socket = accept(socket_id,(struct sockaddr *)&remote_addr, &socklen);
        if (client_socket < 0) {
            vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
            continue;
        }

        fcntl(client_socket, F_SETFL, O_NONBLOCK);

        printf("new client\n");
        // handle client loop
        while (1) {
            int bytes_received = recv(client_socket, rx_buf, sizeof(rx_buf), 1);
            //no bytes available
            if (bytes_received == ERR_WOULDBLOCK) {
                vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
                continue;
            }
            if (bytes_received < 0 ) {
                printf("error %d errno %d\n", bytes_received, errno);
                close(client_socket);
                break;
            }
            if (bytes_received > 0) {
                printf("%d\n", bytes_received);
                write(client_socket, rx_buf, bytes_received);
            }
        }// handle client loop
    }// accept client loop
    vTaskDelete(NULL);
}

// void tcp_server_task() {
//     struct sockaddr_in remote_addr;
// 	unsigned int socklen;
// 	socklen = sizeof(remote_addr);
//     uint8_t rx_buf[32], tx_buf[32];

//     // accept client loop
//     while (1) {        
//         if (!durin.info.wifi_connected) {
//             printf("no wifi\n");
//             vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
//             continue;
//         }
//         vTaskDelay(5000 / portTICK_PERIOD_MS);
//         tcp_client_socket_id = accept(tcp_server_socket_id, (struct sockaddr *)&remote_addr, &socklen);
//         if (tcp_client_socket_id < 0) {
//             printf("shit client %d\n", tcp_client_socket_id);
//             vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
//             continue;
//         }
//         fcntl(tcp_client_socket_id, F_SETFL, O_NONBLOCK);

//         // handle client loop
//         while (1) {
//             int bytes_received = recv(tcp_client_socket_id, rx_buf, sizeof(rx_buf), 1);
//             //no bytes available
//             if (bytes_received == ERR_WOULDBLOCK) {
//                 vTaskDelay(WIFI_DELAY_READ / portTICK_PERIOD_MS);
//             }
//             if (bytes_received < 0 ) {
//                 close(tcp_client_socket_id);
//                 break;
//             }
//             if (bytes_received > 0) {
//                 write(tcp_client_socket_id, rx_buf, bytes_received);
//             }
//         }// handle client loop
//     }// accept client loop
// }

void wifi_disconnected_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    if (durin.info.wifi_connected) {
        close(tcp_server_socket_id);
    }
    durin.info.wifi_connected = 0;
    esp_wifi_connect(); // a failed connect sends a disconnected event
}

void got_ip_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    struct sockaddr_in tcpServerAddr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(1337)
    };
    struct sockaddr_in remote_addr;
	unsigned int socklen;
	socklen = sizeof(remote_addr);
    uint8_t rx_buf[32], tx_buf[32];

    int server_socket_id = socket(AF_INET, SOCK_STREAM, 0);
    bind(server_socket_id, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr));
    listen(server_socket_id, 2);
    fcntl(server_socket_id, F_SETFL, O_NONBLOCK);

    durin.info.wifi_connected = 1;
}

void init_wifi() {
    durin.info.wifi_connected = 0;
    // WiFi
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_init_config.wifi_task_core_id = 1;
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "OnePlus8",
            .password = "halloj1337",
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .bssid_set = 0,
            .channel = 0,
            .listen_interval = 0,
            .pmf_cfg = {
                .required = 0
            },
            .rm_enabled = 1,
            .btm_enabled = 1,
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_country_code("SE", true));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    // esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, wifi_connected_handler, NULL, NULL);
    // esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_disconnected_handler, NULL, NULL);
    // esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_handler, NULL, NULL);    

    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 2048, NULL, 4, NULL, 1);
}

//send stuff here
void update_wifi(struct pt *pt) {

}


void parse_byte(struct pt *pt) {

}