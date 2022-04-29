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
#define SSID "NCSpeople"
#define PASSWORD "peopleNCS"

#define WIFI_DELAY_NO_CLIENT 1000
#define WIFI_DELAY_NO_BYTES 200
#define WIFI_DELAY_NO_CONNECTION 5000

int8_t tcp_server_socket_id;
int8_t udp_client_socket_id;

uint8_t volt_to_percent(float volt) {
    // 8.4 100
    // 8.2 90
    // 8.05 80
    // 7.91 70
    // 7.75 60
    // 7.67 50
    // 7.59 40
    // 7.53 30
    // 7.45 20
    // 7.37 10

    if (volt < 7.37)
        return 0;
    if (volt < 7.45)
        return 10;
    if (volt < 7.53)
        return 20;
    if (volt < 7.59)
        return 30;
    if (volt < 7.67)
        return 40;
    if (volt < 7.75)
        return 50;
    if (volt < 7.91)
        return 60;
    if (volt < 8.05)
        return 70;  
    if (volt < 8.2)
        return 80;        
    if (volt < 8.4)
        return 90;
    return 100;
}

uint16_t package_misc(uint8_t *buf) {
    buf[0] = MISC_UDP;
    struct misc_package *misc_data = (buf + 1);
    misc_data->battery_voltage = durin.telemetry.battery_voltage * 1000;
    misc_data->charge_percent = volt_to_percent(durin.telemetry.battery_voltage);
    misc_data->ax = durin.telemetry.ax;
    misc_data->ay = durin.telemetry.ay;
    misc_data->az = durin.telemetry.az;
    misc_data->gx = durin.telemetry.gx;
    misc_data->gy = durin.telemetry.gy;
    misc_data->gz = durin.telemetry.gz;
    misc_data->mx = durin.telemetry.mx;
    misc_data->my = durin.telemetry.my;
    misc_data->mz = durin.telemetry.mz;
    printf("sent acceleration %d\n", misc_data->ax);
    return sizeof(*misc_data) + 1;
}

uint16_t package_tof(uint8_t *buf, uint8_t package) {
    uint8_t i = 2 * (package - 1);
    buf[0] = TOF_UDP_1 + package - 1;
    memcpy(buf + 1, durin.telemetry.ranging_data[i], 128);
    memcpy(buf + 129, durin.telemetry.ranging_data[i + 1], 128);
    printf("send tof %d %d %d %d %d %d\n", buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    return 257;
}

void tcp_server_task() {
    struct protocol_state protocol_state;
    struct sockaddr_in remote_addr;
	unsigned int socklen;
	socklen = sizeof(remote_addr);
    uint8_t rx_buf[512];
    uint8_t tx_buf[512];

    while (1) {
        vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
        if (!durin.info.wifi_connected) {
            printf("no wifi\n");
            vTaskDelay(WIFI_DELAY_NO_CLIENT / portTICK_PERIOD_MS);
            continue;
        }

        int client_socket = accept(tcp_server_socket_id,(struct sockaddr *)&remote_addr, &socklen);
        if (client_socket < 0) {
            printf("no client\n");            
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

                vTaskDelay(WIFI_DELAY_NO_BYTES / portTICK_PERIOD_MS);
                continue;
            }
            if (bytes_received > 0) {
                printf("WIFI got %d bytes\n", bytes_received);
                for (uint8_t i = 0; i < bytes_received; i++) {
                    uint8_t response;
                    uint8_t should_respond = protocol_parse_byte(&protocol_state, rx_buf[i], &response);
                    if (!should_respond) {
                        continue;
                    }
                    if (response == ACKNOWLEDGE || response == POLL_ALL) {
                        tx_buf[0] = 0;
                        send(client_socket, tx_buf, 1, 0);
                    }
                    if (response == MISC_UDP || response == POLL_ALL) {
                        uint16_t len = package_misc(tx_buf);
                        send(client_socket, tx_buf, len, 0);
                    }
                    if (response == TOF_UDP_1 || response == POLL_ALL) {
                        uint16_t len = package_tof(tx_buf, 1);
                        send(client_socket, tx_buf, len, 0);
                    }
                    if (response == TOF_UDP_2 || response == POLL_ALL) {
                        uint16_t len = package_tof(tx_buf, 2);
                        send(client_socket, tx_buf, len, 0);
                    }
                    if (response == TOF_UDP_3 || response == POLL_ALL) {
                        uint16_t len = package_tof(tx_buf, 3);
                        send(client_socket, tx_buf, len, 0);
                    }
                    if (response == TOF_UDP_4 || response == POLL_ALL) {
                        uint16_t len = package_tof(tx_buf, 4);
                        send(client_socket, tx_buf, len, 0);
                    }
                    if (response == UWB_UDP || response == POLL_ALL) {
                        tx_buf[0] = UWB_UDP;
                        tx_buf[1] = 0;
                        send(client_socket, tx_buf, 2, 0);
                    }
                }
            }
            vTaskDelay(WIFI_DELAY_NO_BYTES / portTICK_PERIOD_MS);
        }// handle client loop
    }// accept client loop
}

void wifi_disconnected_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    if (durin.info.wifi_connected) {
        close(tcp_server_socket_id);
        close(udp_client_socket_id);
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

    udp_client_socket_id = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    fcntl(udp_client_socket_id, F_SETFL, O_NONBLOCK);

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

    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 4096, NULL, 4, NULL, 1);
}

void update_wifi(struct pt *pt) {
    PT_BEGIN(pt);
    static uint64_t last_send; 
    last_send = esp_timer_get_time();
    while(1) {
        PT_YIELD(pt);
        if (!durin.info.wifi_connected) {
            // printf("UDP wifi not active\n");
            continue;
        }
        if (!durin.info.telemetry_udp_enabled) {
            // printf("UDP not enabled\n");
            continue;
        }
        if (durin.info.telemetry_udp_rate == 0) {
            continue;
        }
        if (esp_timer_get_time() - last_send < durin.info.telemetry_udp_rate * 1000) {
            // printf("UDP waiting %llu %d\n", esp_timer_get_time() - last_send, durin.info.telemetry_udp_rate * 1000);
            continue;
        }
        last_send = esp_timer_get_time();
        //send stuff
        struct sockaddr_in dest_addr;
        // dest_addr.sin_addr.s_addr = htonl(durin.info.telemetry_udp_address);
        dest_addr.sin_addr.s_addr = durin.info.telemetry_udp_address;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(durin.info.telemetry_udp_port);

        uint8_t buf[512];
        int16_t len;

        int e;

        len = package_misc(buf);
        e = sendto(udp_client_socket_id, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        // printf("udp return %d\n", e);

        len = package_tof(buf, 1);
        e = sendto(udp_client_socket_id, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        // printf("udp return %d\n", e);

        len = package_tof(buf, 2);
        e = sendto(udp_client_socket_id, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        // printf("udp return %d\n", e);

        len = package_tof(buf, 3);
        e = sendto(udp_client_socket_id, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        // printf("udp return %d\n", e);

        len = package_tof(buf, 4);
        e = sendto(udp_client_socket_id, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        // printf("udp return %d\n", e);
    }

    PT_END(pt);
}