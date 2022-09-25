#include <mdns.h>
#include <sys/socket.h>
#include <netdb.h>
#include <esp_netif.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/gpio_hal.h>
#include <esp_wifi.h>

#include "hardware.h"
#include "pt.h"
#include "durin.h"
#include "protocol.h"
#include "wifi.h"

#define PORT 1337

#define WIFI_DELAY_NO_CLIENT 1000
#define WIFI_DELAY_NO_BYTES 1
#define WIFI_DELAY_NO_CONNECTION 5000

void send_tcp(uint8_t *buf, uint16_t len);
void send_udp(uint8_t *buf, uint16_t len);

int8_t tcp_server_socket_id;
int8_t tcp_client_socket_id;
int8_t udp_client_socket_id;

wifi_config_t wifi_config = {
    .sta = {
        .ssid = DEFAULT_SSID,
        .password = DEFAULT_PASSWORD,
        .scan_method = WIFI_ALL_CHANNEL_SCAN,
        .bssid_set = 0,
        .channel = 0,
        .listen_interval = 0,
        .pmf_cfg = {
            .required = 0
        },
        .rm_enabled = 1,
        .btm_enabled = 1,
    },
};

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

void send_tcp(uint8_t *buf, uint16_t len) {
    if (tcp_client_socket_id < 0) {
        return;
    }
    send(tcp_client_socket_id, buf, len, MSG_DONTWAIT);
}

void send_udp(uint8_t *buf, uint16_t len) {
    if (!durin.info.wifi_connected) {
        return;
    }
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = durin.info.telemetry_udp_address;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(durin.info.telemetry_udp_port);
    sendto(udp_client_socket_id, buf, len, MSG_DONTWAIT, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

void wifi_disconnected_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    if (durin.info.wifi_connected) {
        close(tcp_server_socket_id);
        close(udp_client_socket_id);
        mdns_free();
    }
    durin.info.wifi_connected = 0;

    static uint8_t ping = 0;
    if (ping) {
        memcpy(wifi_config.sta.ssid, durin_persistent.main_ssid, sizeof(durin_persistent.main_ssid));
        memcpy(wifi_config.sta.password, durin_persistent.main_password, sizeof(durin_persistent.main_password));
        ping = 0;
    } else {
        memcpy(wifi_config.sta.ssid, DEFAULT_SSID, sizeof(DEFAULT_SSID));
        memcpy(wifi_config.sta.password, DEFAULT_PASSWORD, sizeof(DEFAULT_PASSWORD));
        ping = 1;
    }
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect(); // a failed connect sends a disconnected event
}

void got_ip_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    struct sockaddr_in tcpServerAddr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(PORT)
    };
    mdns_init();
    char hostname[16];
    sprintf(hostname, "durin%d", durin_persistent.node_id);
    mdns_hostname_set(hostname);

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

    memcpy(wifi_config.sta.ssid, durin_persistent.main_ssid, sizeof(durin_persistent.main_ssid));
    memcpy(wifi_config.sta.password, durin_persistent.main_password, sizeof(durin_persistent.main_password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_country_code("SE", true));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    esp_wifi_set_ps(WIFI_PS_NONE);
    // esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, wifi_connected_handler, NULL, NULL);
    esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_disconnected_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_handler, NULL, NULL);    

    //xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 6096, NULL, 4, NULL, TRASH_CORE);
}

void update_wifi(struct pt *pt) {
    PT_BEGIN(pt);
    while(1) {
        PT_YIELD(pt);
        
        // struct sockaddr_in dest_addr;
        // dest_addr.sin_addr.s_addr = durin.info.telemetry_udp_address;
        // dest_addr.sin_family = AF_INET;
        // dest_addr.sin_port = htons(durin.info.telemetry_udp_port);
        // sendto(udp_client_socket_id, buf, 2, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }
    PT_END(pt);
}

void update_tcp_server(struct pt *pt) {
    PT_BEGIN(pt);

    static struct protocol_state protocol_state = {0};
    static struct sockaddr_in remote_addr;
	static unsigned int socklen;
	socklen = sizeof(remote_addr);
    static uint8_t rx_buf[1024];
    static uint8_t tx_buf[1024];
    protocol_state.channel = CHANNEL_TCP;

    while (1) {
        PT_YIELD(pt);
        if (!durin.info.wifi_connected) {
            // printf("no wifi\n");
            PT_YIELD(pt);
            continue;
        }
        tcp_client_socket_id = accept(tcp_server_socket_id,(struct sockaddr *)&remote_addr, &socklen);
        
        if (tcp_client_socket_id < 0) {
            // printf("no client\n");            
            PT_YIELD(pt);
            continue;
        }
        fcntl(tcp_client_socket_id, F_SETFL, O_NONBLOCK); // this doesn't work lmao, gotta use MSG_DONTWAIT

        // printf("new client\n");
        // handle client loop
        protocol_state.state = 0;
        while (1) {
            int bytes_received = recv(tcp_client_socket_id, rx_buf, sizeof(rx_buf) - 1, MSG_DONTWAIT);

            //no bytes available
            if (bytes_received < 0 ) {
                if (errno == ERR_WOULDBLOCK) {
                    PT_YIELD(pt);
                    continue;
                }
                if (errno == ENOTCONN) {
                    close(tcp_client_socket_id);
                    tcp_client_socket_id = -1;
                    break;
                }

                PT_YIELD(pt);
                continue;
            }
            if (bytes_received > 0) {
                // printf("WIFI got %d bytes\n", bytes_received);
                for (uint16_t i = 0; i < bytes_received; i++) {
                    protocol_parse_byte(&protocol_state, rx_buf[i]);
                }
            }
            PT_YIELD(pt);
        }// handle client loop
    }// accept client loop

    PT_END(pt);
}