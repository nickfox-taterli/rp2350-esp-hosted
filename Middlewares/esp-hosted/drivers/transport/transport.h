//
// Created by TaterLi on 25-2-8.
//

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include "esp_wrapper.h"

// 收发队列大小
#define TO_SLAVE_QUEUE_SIZE               20
#define FROM_SLAVE_QUEUE_SIZE             20

#define MAX_TRANSPORT_BUFFER_SIZE         1600

#define H_BUFF_NO_ZEROCOPY                0
#define H_BUFF_ZEROCOPY                   1

#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED (0xff)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        (0x0)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      (0x2)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      (0x5)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      (0x9)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      (0xC)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C6      (0xD)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C5      (0x17)

esp_err_t transport_drv_init(void(*esp_hosted_up_cb)(void));
esp_err_t transport_drv_deinit(void);
esp_err_t transport_drv_reconfigure(void);
transport_channel_t *transport_drv_add_channel(void *api_chan,
        esp_hosted_if_type_t if_type, uint8_t secure,
        transport_channel_tx_fn_t *tx, const transport_channel_rx_fn_t rx);
esp_err_t transport_drv_remove_channel(transport_channel_t *channel);

int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
        uint8_t * wbuffer, uint16_t wlen, uint8_t buff_zcopy, void (*free_wbuf_fun)(void* ptr));

#endif //TRANSPORT_H
