//
// Created by TaterLi on 25-2-8.
//

#include "esp_wrapper.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

// 定义每行显示的字节数
#define BYTES_PER_LINE 16

// 判断字符是否可打印
#define IS_CHAR_PRINTABLE(c) ((c) >= 0x20 && (c) <= 0x7E)

static const char TAG[] = "wrapper";

static int cvt_hex(uint8_t value, int width, char *output_str);
static void log_buffer_hexdump_line(uintptr_t orig_buff, const void *ptr_line, char *output_str, int buff_len);

QueueHandle_t wifi_event_queue;

// 打印十六进制数据块
void log_buffer_hexdump_internal(const char *tag, const void *buffer, uint16_t buff_len, int log_level)
{
    char output_str[(2 + sizeof(void *) * 2) + 3 + (BYTES_PER_LINE * 3) + 2 + (1 + BYTES_PER_LINE + 1) + 1];
    uintptr_t addr = (uintptr_t)buffer;
    const uint8_t *ptr = (const uint8_t *)buffer;

    while (buff_len > 0) {
        int line_len = (buff_len > BYTES_PER_LINE) ? BYTES_PER_LINE : buff_len;
        log_buffer_hexdump_line(addr, ptr, output_str, line_len);

        // 这里替换为你的实际日志输出函数
        printf("[%s] %s\n", tag, output_str);

        addr += line_len;
        ptr += line_len;
        buff_len -= line_len;
    }
}

int hosted_wifi_event_post(int32_t event_id,
        void* event_data, size_t event_data_size, uint32_t ticks_to_wait)
{
    ESP_LOGV(TAG, "event %ld recvd --> event_data:%p event_data_size: %u\n", event_id, event_data, event_data_size);

    // 检查队列是否有效
    if (wifi_event_queue == NULL) {
        ESP_LOGE(TAG, "Event queue is not initialized");
        return -1; // 返回错误
    }

    // 创建一个事件消息结构体
    wifi_event_msg_t event_msg;
    event_msg.event_id = event_id;

    // 复制事件数据到结构体中
    event_msg.event_data = pvPortMalloc(event_data_size); // 动态分配内存
    if (event_msg.event_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for event data");
        return -1; // 返回错误
    }
    memcpy(event_msg.event_data, event_data, event_data_size);
    event_msg.event_data_size = event_data_size;

    // 将事件消息发送到队列
    if (xQueueSend(wifi_event_queue, &event_msg, ticks_to_wait) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send event to queue");
        vPortFree(event_msg.event_data); // 释放内存
        return -1; // 返回错误
    }

    return 0; // 成功
}

// 将数值转换为十六进制字符串
static int cvt_hex(uint8_t value, int width, char *output_str)
{
    static const char hex_digits[] = "0123456789abcdef";
    int len = 0;

    while (width--) {
        output_str[len++] = hex_digits[(value >> (width * 4)) & 0xF];
    }
    return len;
}

// 打印一行十六进制数据
static void log_buffer_hexdump_line(uintptr_t orig_buff, const void *ptr_line, char *output_str, int buff_len)
{
    const unsigned char *ptr = (unsigned char *)ptr_line;
    *output_str++ = '0';
    *output_str++ = 'x';
    output_str += cvt_hex(orig_buff, sizeof(uintptr_t) * 2, output_str);
    *output_str++ = ' ';

    for (int i = 0; i < BYTES_PER_LINE; i++) {
        if ((i & 7) == 0) {
            *output_str++ = ' ';
        }
        *output_str++ = ' ';
        if (i < buff_len) {
            output_str += cvt_hex(ptr[i], 2, output_str);
        } else {
            *output_str++ = ' ';
            *output_str++ = ' ';
        }
    }

    *output_str++ = ' ';
    *output_str++ = ' ';
    *output_str++ = '|';
    for (int i = 0; i < buff_len; i++) {
        *output_str++ = IS_CHAR_PRINTABLE(ptr[i]) ? ptr[i] : '.';
    }
    *output_str++ = '|';
    *output_str = 0;
}


