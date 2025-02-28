/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "esp_hosted_api.h"
#include "esp_wifi.h"

#include <stdio.h>
#include <string.h>
#include <hardware/clocks.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t wifi_event_queue;

void wifi_event(void* pvParameters)
{
    wifi_event_msg_t event_msg;
    while (1)
    {
        if (xQueueReceive(wifi_event_queue, &event_msg, portMAX_DELAY) == pdPASS) {
            // 处理接收到的事件
            if (event_msg.event_id == WIFI_EVENT_SCAN_DONE)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            if(event_msg.event_data != NULL)
              vPortFree(event_msg.event_data); // 处理完后释放内存        // 从队列中接收事件消息
        }
    }
}

uint16_t number = 20;
wifi_ap_record_t ap_info[20];
uint16_t ap_count = 0;

#include "lwip/tcpip.h"
#include "lwip/opt.h"
#include "lwip/dhcp.h"
#include "ethernetif.h"
#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/ip_addr.h"
#include "lwip/icmp.h"
#include "lwip/raw.h"
#include "lwip/inet.h"

struct netif gnetif;

#define MAX_DHCP_TRIES  4

#define DHCP_OFF                   (uint8_t) 0
#define DHCP_START                 (uint8_t) 1
#define DHCP_WAIT_ADDRESS          (uint8_t) 2
#define DHCP_ADDRESS_ASSIGNED      (uint8_t) 3
#define DHCP_TIMEOUT               (uint8_t) 4
#define DHCP_LINK_DOWN             (uint8_t) 5

volatile uint8_t DHCP_state = DHCP_START;

void ethernet_dhcp_thread(void * pvParameters)
{
    struct netif *netif = (struct netif *) pvParameters;
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;
    struct dhcp *dhcp;

    for (;;)
    {
        switch (DHCP_state)
        {
        case DHCP_START:
            {
                // ip_addr_set_zero_ip4(&netif->ip_addr);
                // ip_addr_set_zero_ip4(&netif->netmask);
                // ip_addr_set_zero_ip4(&netif->gw);

                IP4_ADDR(&netif->ip_addr, 192, 168, 31, 7);  // 例如：192.168.1.100
                IP4_ADDR(&netif->netmask, 255, 255, 255, 0); // 子网掩码：255.255.255.0
                IP4_ADDR(&netif->gw, 192, 168, 31, 1);       // 网关：192.168.1.1
                DHCP_state = DHCP_WAIT_ADDRESS;
                dhcp_start(netif);
            }
            break;

        case DHCP_WAIT_ADDRESS:
            {
                if (dhcp_supplied_address(netif))
                {
                    DHCP_state = DHCP_ADDRESS_ASSIGNED;
                }
                else
                {
                    dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

                    /* DHCP timeout */
                    if (dhcp->tries > MAX_DHCP_TRIES)
                    {
                        DHCP_state = DHCP_START;
                    }
                }
            }
            break;
        case DHCP_LINK_DOWN:
            {
                DHCP_state = DHCP_OFF;
            }
            break;
        default: break;
        }

        /* wait 500 ms */
        vTaskDelay(500);
    }
}

static void netif_config(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

    ip_addr_set_zero_ip4(&ipaddr);
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);

    /* add the network interface */
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    /*  Registers the default network interface. */
    netif_set_default(&gnetif);
    vTaskDelay(3000);
    netif_set_up(&gnetif);
    netif_set_link_up(&gnetif);
    xTaskCreate(ethernet_dhcp_thread, "EthDHCP", configMINIMAL_STACK_SIZE * 2, &gnetif, 1, NULL);
}

void wifi_app(void* pvParameters)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_hosted_host_init();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_event_queue = xQueueCreate(10, sizeof(wifi_event_msg_t)); // 创建一个长度为10的队列
    if (wifi_event_queue == NULL) {
        printf("Failed to create event queue");
    }

    xTaskCreate(wifi_event, "wifi_event", (1 * 1024), NULL, tskIDLE_PRIORITY + 2UL, NULL);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // WiFi 扫描(非必须)
    // esp_wifi_scan_start(NULL, true);
    // ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    // ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Xiaomi_F5F6",
            .password = "TaterLi1024"
        },
    };

    // 这里代码需要后期修改,如果连接失败,则延迟一会重复连接直到成功.
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        printf("WiFi connect failed! ret:%x", ret);
        while (1)
        {
            vTaskSuspend(NULL);
        }
    }

    // esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);

    tcpip_init(NULL, NULL);
    netif_config();

    while (1) {
        // 数据发送
        // esp_wifi_internal_tx(WIFI_IF_STA,buf,sizeof(buf));

        vTaskDelay(1000);
    }
}

void main_app(__unused void *params) {
    xTaskCreate(wifi_app, "wifi_thread", (10 * 1024), NULL, tskIDLE_PRIORITY + 2UL, NULL);
    while(true) {
        vTaskDelete(NULL);
    }
}

int main( void )
{
    stdio_init_all();

    xTaskCreate(main_app, "MainThread", (1 * 1024), NULL, tskIDLE_PRIORITY , NULL);
    vTaskStartScheduler();
    for(;;)
    {
        // 无论怎样都不应该执行到这里.
    }
}