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
            vPortFree(event_msg.event_data); // 处理完后释放内存        // 从队列中接收事件消息
        }
    }
}

uint16_t number = 20;
wifi_ap_record_t ap_info[20];
uint16_t ap_count = 0;

char buf[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x48, 0x31, 0xb7, 0x3e, 0x55, 0x40, 0x8, 0x0, 0x45, 0x0, 0x1, 0x50, 0x0, 0x0, 0x0, 0x0, 0x40, 0x11, 0x79, 0x9e, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0x0, 0x44, 0x0, 0x43, 0x1, 0x3c, 0x21, 0x66, 0x1, 0x1, 0x6, 0x0, 0x68, 0xb8, 0x3c, 0x71, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x48, 0x31, 0xb7, 0x3e, 0x55, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x63, 0x82, 0x53, 0x63, 0x35, 0x1, 0x1, 0x39, 0x2, 0x5, 0xdc, 0xc, 0x9, 0x65, 0x73, 0x70, 0x72, 0x65, 0x73, 0x73, 0x69, 0x66, 0x37, 0x4, 0x1, 0x3, 0x1c, 0x6, 0x3d, 0x7, 0x1, 0x48, 0x31, 0xb7, 0x3e, 0x55, 0x40, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// 数据接受的函数
esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
    if (((uint8_t *)buffer)[0] == 0xFF)
    {
        vTaskDelay(1);
    }
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

    esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);

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