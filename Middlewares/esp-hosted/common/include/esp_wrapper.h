//
// Created by TaterLi on 25-2-6.
//

#ifndef ESP_WRAPPER_H
#define ESP_WRAPPER_H

#include <stdio.h>

#include "adapter.h"

/* 错误定义 */

typedef int esp_err_t;

/* 错误码常量定义 */
#define ESP_OK          0       /*!< esp_err_t值，表示成功（无错误） */
#define ESP_FAIL        -1      /*!< 通用esp_err_t错误码，表示失败 */

#define ESP_ERR_NO_MEM              0x101   /*!< 内存不足 */
#define ESP_ERR_INVALID_ARG         0x102   /*!< 无效参数 */
#define ESP_ERR_INVALID_STATE       0x103   /*!< 无效状态 */
#define ESP_ERR_INVALID_SIZE        0x104   /*!< 无效大小 */
#define ESP_ERR_NOT_FOUND           0x105   /*!< 未找到请求的资源 */
#define ESP_ERR_NOT_SUPPORTED       0x106   /*!< 不支持的操作或功能 */
#define ESP_ERR_TIMEOUT             0x107   /*!< 操作超时 */
#define ESP_ERR_INVALID_RESPONSE    0x108   /*!< 接收到的响应无效 */
#define ESP_ERR_INVALID_CRC         0x109   /*!< CRC或校验和无效 */
#define ESP_ERR_INVALID_VERSION     0x10A   /*!< 版本无效 */
#define ESP_ERR_INVALID_MAC         0x10B   /*!< MAC地址无效 */
#define ESP_ERR_NOT_FINISHED        0x10C   /*!< 操作未完全完成 */
#define ESP_ERR_NOT_ALLOWED         0x10D   /*!< 不允许的操作 */

#define ESP_ERR_WIFI_BASE           0x3000  /*!< WiFi错误码的起始编号 */
#define ESP_ERR_MESH_BASE           0x4000  /*!< MESH错误码的起始编号 */
#define ESP_ERR_FLASH_BASE          0x6000  /*!< 闪存错误码的起始编号 */
#define ESP_ERR_HW_CRYPTO_BASE      0xc000  /*!< 硬件加密模块错误码的起始编号 */
#define ESP_ERR_MEMPROT_BASE        0xd000  /*!< 内存保护API错误码的起始编号 */

#define RET_OK                                       0
#define RET_FAIL                                     -1
#define RET_INVALID                                  -2
#define RET_FAIL_MEM                                 -3
#define RET_FAIL4                                    -4
#define RET_FAIL_TIMEOUT                             -5

#define likely(x)      (x)
#define unlikely(x)    (x)

/*
 * Host to big endian, host to little endian, big endian to host, and little
 * endian to host byte order functions as detailed in byteorder(9).
 */
#if _BYTE_ORDER == _LITTLE_ENDIAN
#define htobe16(x)  bswap16((x))
#define htobe32(x)  bswap32((x))
#define htobe64(x)  bswap64((x))
#define htole16(x)  ((uint16_t)(x))
#define htole32(x)  ((uint32_t)(x))
#define htole64(x)  ((uint64_t)(x))

#define be16toh(x)  bswap16((x))
#define be32toh(x)  bswap32((x))
#define be64toh(x)  bswap64((x))
#define le16toh(x)  ((uint16_t)(x))
#define le32toh(x)  ((uint32_t)(x))
#define le64toh(x)  ((uint64_t)(x))
#else /* _BYTE_ORDER != _LITTLE_ENDIAN */
#define htobe16(x)  ((uint16_t)(x))
#define htobe32(x)  ((uint32_t)(x))
#define htobe64(x)  ((uint64_t)(x))
#define htole16(x)  bswap16((x))
#define htole32(x)  bswap32((x))
#define htole64(x)  bswap64((x))

#define be16toh(x)  ((uint16_t)(x))
#define be32toh(x)  ((uint32_t)(x))
#define be64toh(x)  ((uint64_t)(x))
#define le16toh(x)  bswap16((x))
#define le32toh(x)  bswap32((x))
#define le64toh(x)  bswap64((x))
#endif /* _BYTE_ORDER == _LITTLE_ENDIAN */

/* 日志定义 */

// 定义日志级别枚举
typedef enum
{
    ESP_LOG_NONE = 0, /*!< 无日志输出 */
    ESP_LOG_ERROR = 1, /*!< 严重错误，软件模块无法自行恢复 */
    ESP_LOG_WARN = 2, /*!< 错误条件，但已采取恢复措施 */
    ESP_LOG_INFO = 3, /*!< 描述正常事件流的信息消息 */
    ESP_LOG_DEBUG = 4, /*!< 正常使用中不必要的额外信息（值、指针、大小等） */
    ESP_LOG_VERBOSE = 5, /*!< 更大的调试信息块，或可能淹没输出的频繁消息 */
    ESP_LOG_MAX = 6, /*!< 支持的日志级别数量 */
} esp_log_level_t;

typedef enum {
    ESP_IF_WIFI_STA = 0,     /**< Station interface */
    ESP_IF_WIFI_AP,          /**< Soft-AP interface */
    ESP_IF_WIFI_NAN,         /**< NAN interface */
    ESP_IF_ETH,              /**< Ethernet interface */
    ESP_IF_MAX
} esp_interface_t;

// 定义日志输出宏
#define ESP_LOGE(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)

// 日志输出开关
#define ESP_LOG_ENABLE 1

// 日志级别输出宏
#define ESP_LOG_LEVEL_LOCAL(level, tag, format, ...) do {               \
        if (ESP_LOG_ENABLE) ESP_LOG_LEVEL(level, tag, format, ##__VA_ARGS__); \
    } while(0)

// 日志输出实现
#define ESP_LOG_LEVEL(level, tag, format, ...) do {                     \
        if (level == ESP_LOG_ERROR) {                                   \
            printf("[E][%s] " format "\n", tag, ##__VA_ARGS__);         \
        } else if (level == ESP_LOG_WARN) {                             \
            printf("[W][%s] " format "\n", tag, ##__VA_ARGS__);         \
        } else if (level == ESP_LOG_DEBUG) {                            \
            printf("[D][%s] " format "\n", tag, ##__VA_ARGS__);         \
        } else if (level == ESP_LOG_VERBOSE) {                          \
            printf("[V][%s] " format "\n", tag, ##__VA_ARGS__);         \
        } else {                                                        \
            printf("[I][%s] " format "\n", tag, ##__VA_ARGS__);         \
        }                                                               \
    } while(0)

/* -------- 创建句柄 ------- */
#define HOSTED_CREATE_HANDLE(tYPE, hANDLE) {                                   \
hANDLE = (tYPE *)pvPortMalloc(sizeof(tYPE));                               \
if (!hANDLE) {                                                             \
printf("%s:%u 内存分配失败，无法创建句柄\n", __func__,__LINE__);         \
return NULL;                                                           \
}                                                                          \
}

/* -------- 释放句柄 ------- */
#define HOSTED_FREE(buff) if (buff) { vPortFree(buff); buff = NULL; }

/* -------- 分配并清零内存 ------- */
#define HOSTED_CALLOC(struct_name, buff, nbytes, gotosym) do {                 \
    buff = (struct_name *)pvPortCalloc(1, nbytes);                            \
    if (!buff) {                                                              \
        printf("%s, Malloc Failed\n", __func__);                               \
        goto gotosym;                                                         \
    }                                                                         \
} while(0);

/* -------- 分配内存 ------- */
#define HOSTED_MALLOC(struct_name, buff, nbytes, gotosym) do {                \
    buff = (struct_name *)pvPortMalloc(nbytes);                               \
    if (!buff) {                                                              \
        printf("%s, Malloc Failed\n", __func__);                               \
        goto gotosym;                                                         \
    }                                                                         \
} while(0);

#define ESP_ERROR_CHECK(x) do {                                         \
    esp_err_t err_rc_ = (x);                                        \
    if (unlikely(err_rc_ != ESP_OK)) {                              \
        printf("Error in file: %s, line: %d\n", __FILE__, __LINE__); \
        while (1) {}                                                \
    }                                                               \
} while(0)

// 日志输出宏
void log_buffer_hexdump_internal(const char *tag, const void *buffer, uint16_t buff_len, int log_level);
#define LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level) \
    do { if (ESP_LOG_ENABLE) {log_buffer_hexdump_internal(tag, buffer, buff_len, level);} } while(0)

#define ESP_PRIV_HEXDUMP(tag1, tag2, buff, len, curr_level)                   \
    printf("[%s] %s: len[%d]\n", tag1, tag2, (int)len);     \
    LOG_BUFFER_HEXDUMP(tag2, buff, len, curr_level);

#define ESP_HEXLOGE(tag2, buff, len) ESP_PRIV_HEXDUMP(TAG, tag2, buff, len, ESP_LOG_ERROR)
#define ESP_HEXLOGW(tag2, buff, len) ESP_PRIV_HEXDUMP(TAG, tag2, buff, len, ESP_LOG_WARN)
#define ESP_HEXLOGI(tag2, buff, len) ESP_PRIV_HEXDUMP(TAG, tag2, buff, len, ESP_LOG_INFO)
#define ESP_HEXLOGD(tag2, buff, len) ESP_PRIV_HEXDUMP(TAG, tag2, buff, len, ESP_LOG_DEBUG)
#define ESP_HEXLOGV(tag2, buff, len) ESP_PRIV_HEXDUMP(TAG, tag2, buff, len, ESP_LOG_VERBOSE)

// 定义传输通道的发送函数类型
// h: 传输通道的句柄
// buffer: 要发送的数据缓冲区
// len: 数据长度
// 返回: ESP_OK 表示成功，其他值表示错误
typedef esp_err_t (*transport_channel_tx_fn_t)(void* h, void* buffer, size_t len);

// 定义传输通道的接收函数类型
// h: 传输通道的句柄
// buffer: 接收数据的缓冲区
// buff_to_free: 需要释放的缓冲区（如果有）
// len: 数据长度
// 返回: ESP_OK 表示成功，其他值表示错误
typedef esp_err_t (*transport_channel_rx_fn_t)(void* h, void* buffer, void* buff_to_free, size_t len);

typedef struct esp_remote_channel *esp_remote_channel_t;

/**
 * @brief Remote channel Rx function pointer
 */
typedef esp_err_t (*esp_remote_channel_rx_fn_t)(void *h, void *buffer, void *buff_to_free, size_t len);

/**
 * @brief Remote channel Tx function pointer
 */
typedef esp_err_t (*esp_remote_channel_tx_fn_t)(void *h, void *buffer, size_t len);

/**
 * @brief Remote channel configuration
 */
typedef struct esp_remote_channel_config *esp_remote_channel_config_t;

// 定义传输通道的结构体
typedef struct
{
    void* api_chan; // 指向API通道的指针，用于与底层硬件或协议栈交互
    esp_hosted_if_type_t if_type; // 接口类型，表示使用的通信接口（如WiFi、以太网等）
    uint8_t secure; // 安全标志，表示是否启用加密或安全通信
    transport_channel_tx_fn_t tx; // 发送函数指针，用于发送数据
    transport_channel_rx_fn_t rx; // 接收函数指针，用于接收数据
    void* memp; // 内存池指针，用于管理内存分配（通常用于避免动态内存分配）
} transport_channel_t;

typedef struct {
    int32_t event_id;
    void* event_data;
    size_t event_data_size;
} wifi_event_msg_t;

/* 函数导出 */
esp_err_t transport_drv_init(void (*esp_hosted_up_cb)(void));
esp_err_t transport_drv_deinit(void);
esp_err_t transport_drv_reconfigure(void);

int hosted_wifi_event_post(int32_t event_id,
        void* event_data, size_t event_data_size, uint32_t ticks_to_wait);

#endif //ESP_WRAPPER_H
