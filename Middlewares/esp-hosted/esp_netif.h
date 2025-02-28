//
// Created by TaterLi on 25-2-8.
//

#ifndef ESP_NETIF_H
#define ESP_NETIF_H

#include "esp_wifi_types_generic.h"

/**
 * @brief Definition of ESP-NETIF based errors
 */
#define ESP_ERR_ESP_NETIF_BASE                  0x5000
#define ESP_ERR_ESP_NETIF_INVALID_PARAMS        ESP_ERR_ESP_NETIF_BASE + 0x01
#define ESP_ERR_ESP_NETIF_IF_NOT_READY          ESP_ERR_ESP_NETIF_BASE + 0x02
#define ESP_ERR_ESP_NETIF_DHCPC_START_FAILED    ESP_ERR_ESP_NETIF_BASE + 0x03
#define ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED  ESP_ERR_ESP_NETIF_BASE + 0x04
#define ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED  ESP_ERR_ESP_NETIF_BASE + 0x05
#define ESP_ERR_ESP_NETIF_NO_MEM                ESP_ERR_ESP_NETIF_BASE + 0x06
#define ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED      ESP_ERR_ESP_NETIF_BASE + 0x07
#define ESP_ERR_ESP_NETIF_DRIVER_ATTACH_FAILED  ESP_ERR_ESP_NETIF_BASE + 0x08
#define ESP_ERR_ESP_NETIF_INIT_FAILED           ESP_ERR_ESP_NETIF_BASE + 0x09
#define ESP_ERR_ESP_NETIF_DNS_NOT_CONFIGURED    ESP_ERR_ESP_NETIF_BASE + 0x0A
#define ESP_ERR_ESP_NETIF_MLD6_FAILED           ESP_ERR_ESP_NETIF_BASE + 0x0B
#define ESP_ERR_ESP_NETIF_IP6_ADDR_FAILED       ESP_ERR_ESP_NETIF_BASE + 0x0C
#define ESP_ERR_ESP_NETIF_DHCPS_START_FAILED    ESP_ERR_ESP_NETIF_BASE + 0x0D
#define ESP_ERR_ESP_NETIF_TX_FAILED             ESP_ERR_ESP_NETIF_BASE + 0x0E

/* Set additional WiFi features and capabilities */
#define WIFI_FEATURE_CAPS 0

#define WIFI_INIT_CONFIG_MAGIC    0x1F2F3F4F

#define WIFI_INIT_CONFIG_DEFAULT() { \
    .static_rx_buf_num = 16,\
    .dynamic_rx_buf_num = 64,\
    .tx_buf_type = 1,\
    .static_tx_buf_num = 0,\
    .dynamic_tx_buf_num = 64,\
    .rx_mgmt_buf_type = 0,\
    .rx_mgmt_buf_num = 5,\
    .cache_tx_buf_num = 0,\
    .csi_enable = 0,\
    .ampdu_rx_enable = 1,\
    .ampdu_tx_enable = 1,\
    .amsdu_tx_enable = 0,\
    .nvs_enable = 1,\
    .nano_enable = 0,\
    .rx_ba_win = 32,\
    .wifi_task_core_id = 0,\
    .beacon_max_len = 752, \
    .mgmt_sbuf_num = 32, \
    .feature_caps = WIFI_FEATURE_CAPS, \
    .sta_disconnected_pm = true,  \
    .espnow_max_encrypt_num = 7, \
    .tx_hetb_queue_num = 1, \
    .dump_hesigb_enable = false, \
    .magic = WIFI_INIT_CONFIG_MAGIC\
}

typedef struct {
    int                    static_rx_buf_num;      /**< WiFi static RX buffer number */
    int                    dynamic_rx_buf_num;     /**< WiFi dynamic RX buffer number */
    int                    tx_buf_type;            /**< WiFi TX buffer type */
    int                    static_tx_buf_num;      /**< WiFi static TX buffer number */
    int                    dynamic_tx_buf_num;     /**< WiFi dynamic TX buffer number */
    int                    rx_mgmt_buf_type;       /**< WiFi RX MGMT buffer type */
    int                    rx_mgmt_buf_num;        /**< WiFi RX MGMT buffer number */
    int                    cache_tx_buf_num;       /**< WiFi TX cache buffer number */
    int                    csi_enable;             /**< WiFi channel state information enable flag */
    int                    ampdu_rx_enable;        /**< WiFi AMPDU RX feature enable flag */
    int                    ampdu_tx_enable;        /**< WiFi AMPDU TX feature enable flag */
    int                    amsdu_tx_enable;        /**< WiFi AMSDU TX feature enable flag */
    int                    nvs_enable;             /**< WiFi NVS flash enable flag */
    int                    nano_enable;            /**< Nano option for printf/scan family enable flag */
    int                    rx_ba_win;              /**< WiFi Block Ack RX window size */
    int                    wifi_task_core_id;      /**< WiFi Task Core ID */
    int                    beacon_max_len;         /**< WiFi softAP maximum length of the beacon */
    int                    mgmt_sbuf_num;          /**< WiFi management short buffer number, the minimum value is 6, the maximum value is 32 */
    uint64_t               feature_caps;           /**< Enables additional WiFi features and capabilities */
    bool                   sta_disconnected_pm;    /**< WiFi Power Management for station at disconnected status */
    int                    espnow_max_encrypt_num; /**< Maximum encrypt number of peers supported by espnow */
    int                    tx_hetb_queue_num;      /**< WiFi TX HE TB QUEUE number for STA HE TB PPDU transmission */
    bool                   dump_hesigb_enable;     /**< enable dump sigb field */
    int                    magic;                  /**< WiFi init magic number, it should be the last field */
} wifi_init_config_t;

#define ESP_WIFI_MAX_CONN_NUM  (10)       /**< max number of stations which can connect to ESP32/ESP32S3/ESP32S2 soft-AP */

/** @brief List of stations associated with the Soft-AP */
typedef struct wifi_sta_list_t {
    wifi_sta_info_t sta[ESP_WIFI_MAX_CONN_NUM]; /**< station list */
    int       num; /**< number of stations in the list (other entries are invalid) */
} wifi_sta_list_t;

/**
 * @brief Forward declare wifi_sta_list_t. The definition depends on the target device
 * that implements esp_wifi
 */
typedef struct wifi_sta_list_t wifi_sta_list_t;

/**
  * @brief     The WiFi RX callback function
  *
  *            Each time the WiFi need to forward the packets to high layer, the callback function will be called
  */
typedef esp_err_t (*wifi_rxcb_t)(void *buffer, uint16_t len, void *eb);

#endif //ESP_NETIF_H
