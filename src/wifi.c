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
#include "protocol.h"

#define PORT 1337
#define SSID "OnePlus8"
#define PASSWORD "halloj1337"

#define WIFI_DELAY_NO_CLIENT 1000
#define WIFI_DELAY_NO_BYTES 200
#define WIFI_DELAY_NO_CONNECTION 5000

int8_t tcp_server_socket_id;
int8_t udp_client_socket_id;

// static void tcp_server_task(void *arg) {
//     struct sockaddr_in tcpServerAddr = {
//         .sin_addr.s_addr = htonl(INADDR_ANY),
//         .sin_family = AF_INET,
//         .sin_port = htons(1337)
//     };
//     vTaskDelay(10000 / portTICK_PERIOD_MS);
//     struct sockaddr_in remote_addr;
// 	unsigned int socklen;
// 	socklen = sizeof(remote_addr);
//     uint8_t rx_buf[32], tx_buf[32];

//     int socket_id = socket(AF_INET, SOCK_STREAM, 0);
//     bind(socket_id, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr));
//     listen(socket_id, 2);
//     fcntl(socket_id, F_SETFL, O_NONBLOCK);
//     // accept client loop
//     while (1) {
//         int client_socket = accept(socket_id,(struct sockaddr *)&remote_addr, &socklen);
//         if (client_socket < 0) {
//             vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
//             continue;
//         }

//         printf("new client\n");
//         // handle client loop
//         while (1) {
//             int bytes_received = recv(client_socket, rx_buf, sizeof(rx_buf), 0);
//             //no bytes available
//             if (bytes_received < 0 ) {
//                 if (errno == ERR_WOULDBLOCK) {
//                     vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
//                     continue;
//                 }
//                 if (errno == ENOTCONN) {
//                     break;
//                 }
//             }
//             if (bytes_received > 0) {
//                 printf("%d\n", bytes_received);
//                 write(client_socket, rx_buf, bytes_received);
//             }
//         }// handle client loop
//     }// accept client loop
//     vTaskDelete(NULL);
// }



void tcp_server_task() {
    struct protocol_state protocol_state;
    struct sockaddr_in remote_addr;
	unsigned int socklen;
	socklen = sizeof(remote_addr);
    uint8_t rx_buf[64];

    while (1) {
        if (!durin.info.wifi_connected) {
            printf("no wifi\n");
            vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
            continue;
        }

        int client_socket = accept(tcp_server_socket_id,(struct sockaddr *)&remote_addr, &socklen);
        if (client_socket < 0) {
            vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
            continue;
        }

        printf("new client\n");
        // handle client loop
        protocol_state.state = 0;
        while (1) {
            int bytes_received = recv(client_socket, rx_buf, sizeof(rx_buf), 0);
            //no bytes available
            if (bytes_received < 0 ) {
                if (errno == ERR_WOULDBLOCK) {
                    vTaskDelay(WIFI_DELAY_NO_BYTES / portTICK_PERIOD_MS);
                    continue;
                }
                if (errno == ENOTCONN) {
                    break;
                }
            }
            if (bytes_received > 0) {
                for (uint8_t i = 0; i < bytes_received; i++) {
                    protocol_parse_byte(&protocol_state, rx_buf[i]);
                }
            }
        }// handle client loop
    }// accept client loop
}

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
        .sin_port = htons(PORT)
    };

    tcp_server_socket_id = socket(AF_INET, SOCK_STREAM, 0);
    bind(tcp_server_socket_id, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr));
    listen(tcp_server_socket_id, 2);
    fcntl(tcp_server_socket_id, F_SETFL, O_NONBLOCK);

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
            .ssid = SSID,
            .password = PASSWORD,
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
    esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_disconnected_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_handler, NULL, NULL);    

    // udp_client_socket_id = open(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    // fcntl(udp_client_socket_id, F_SETFL, O_NONBLOCK);

    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 2048, NULL, 4, NULL, 1);
}

void update_wifi(struct pt *pt) {
    PT_BEGIN(pt);
    static uint64_t last_send; 
    last_send = esp_timer_get_time();
    while(1) {
        PT_YIELD(pt);
        if (!durin.info.telemetry_udp_enabled) {
            continue;
        }
        if (esp_timer_get_time() - last_send < durin.info.telemetry_udp_rate * 1000) {
            continue;
        }
        //send stuff

        last_send = esp_timer_get_time();
    }

    PT_END(pt);
}