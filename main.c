#include "esp_hosted_api.h"
#include "esp_wifi.h"

#include <stdio.h>
#include <string.h>
#include <hardware/clocks.h>

#include "pico/stdlib.h"

#include "lwip/tcpip.h"
#include "lwip/opt.h"
#include "lwip/dhcp.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/apps/lwiperf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define MAX_DHCP_TRIES  4

#define DHCP_START                 (uint8_t) 1
#define DHCP_WAIT_ADDRESS          (uint8_t) 2
#define DHCP_ADDRESS_ASSIGNED      (uint8_t) 3

struct netif gnetif;

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

void EthBoot(void * pvParameters)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;
    struct dhcp *dhcp;

    static uint8_t state = DHCP_START;

    ip_addr_set_zero_ip4(&ipaddr);
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);
    
    /* add the network interface */
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    /*  Registers the default network interface. */
    netif_set_default(&gnetif);
    netif_set_up(&gnetif);
    netif_set_link_up(&gnetif);

    for (;;)
    {
        switch (state)
        {
        case DHCP_START:
            {
                ip_addr_set_zero_ip4(&gnetif.ip_addr);
                ip_addr_set_zero_ip4(&gnetif.netmask);
                ip_addr_set_zero_ip4(&gnetif.gw);
                state = DHCP_WAIT_ADDRESS;
                dhcp_start(&gnetif);
            }
            break;

        case DHCP_WAIT_ADDRESS:
            {
                if (dhcp_supplied_address(&gnetif))
                {
                    state = DHCP_ADDRESS_ASSIGNED;
                    // 启动内置的iperf2测速服务器
                    lwiperf_start_tcp_server_default(NULL,NULL);
                }
                else
                {
                    dhcp = (struct dhcp *)netif_get_client_data(&gnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

                    /* DHCP timeout */
                    if (dhcp->tries > MAX_DHCP_TRIES)
                    {
                        state = DHCP_START;
                    }
                }
            }
            break;
        default: break;
        }

        /* wait 500 ms */
        vTaskDelay(500);
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

    tcpip_init(NULL, NULL);
    xTaskCreate(EthBoot, "EthBoot", configMINIMAL_STACK_SIZE * 2, &gnetif, 1, NULL);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
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