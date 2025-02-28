// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

/** Includes **/
#include "string.h"
#include "serial_ll_if.h"
#include "transport.h"
#include "adapter.h"

static const char TAG[] = "serial_ll";

/** Macros / Constants **/
#define MAX_SERIAL_INTF                   2
#define TO_SERIAL_INFT_QUEUE_SIZE         100

typedef enum {
	INIT,
	ACTIVE,
	DESTROY
} serial_ll_state_e;

static struct rx_data {
	int len;
	uint8_t *data;
} r;

/* data structures needed for serial driver */
static QueueHandle_t to_serial_ll_intf_queue[MAX_SERIAL_INTF];
static serial_ll_handle_t * interface_handle_g[MAX_SERIAL_INTF] = {NULL};
static uint8_t conn_num = 0;

/** Function Declarations **/
static int       serial_ll_open    (serial_ll_handle_t *serial_ll_hdl);
static uint8_t * serial_ll_read    (const serial_ll_handle_t * serial_ll_hdl,
		uint16_t * rlen);
static int       serial_ll_write   (const serial_ll_handle_t * serial_ll_hdl,
		uint8_t * wbuffer, const uint16_t wlen);
static int       serial_ll_close   (serial_ll_handle_t * serial_ll_hdl);


/* define serial interface */
static struct serial_ll_operations serial_ll_fops = {
	.open    = serial_ll_open,
	.read    = serial_ll_read,
	.write   = serial_ll_write,
	.close   = serial_ll_close,
};

/** function definition **/

/** Local Functions **/

/**
  * @brief 打开新的串行接口
  * @param  serial_ll_hdl - 串行接口的句柄
  * @retval 0 成功, -1 失败
  */
static int serial_ll_open(serial_ll_handle_t *serial_ll_hdl)
{
	if (!serial_ll_hdl) {
		ESP_LOGE(TAG, "串行接口句柄无效");
		return STM_FAIL;
	}

	if (serial_ll_hdl->queue) {
		/* 清理之前的队列 */
		vQueueDelete(serial_ll_hdl->queue); // 使用FreeRTOS的vQueueDelete函数删除队列
	}

	/* 创建串行接收队列 */
	serial_ll_hdl->queue = xQueueCreate(TO_SERIAL_INFT_QUEUE_SIZE, sizeof(interface_buffer_handle_t)); // 使用FreeRTOS的xQueueCreate函数创建队列

	if (!serial_ll_hdl->queue) {
		serial_ll_close(serial_ll_hdl);
		ESP_LOGE(TAG, "无法创建串行接收队列");
		return STM_FAIL;
	}

	serial_ll_hdl->state = ACTIVE;
	return STM_OK;
}

/**
  * @brief Get serial handle for iface_num
  * @param  iface_num - serial connection number
  * @retval serial_ll_hdl - output handle of serial interface
  */
static serial_ll_handle_t * get_serial_ll_handle(const uint8_t iface_num)
{
	if ((iface_num < MAX_SERIAL_INTF) &&
		(interface_handle_g[iface_num]) &&
		(interface_handle_g[iface_num]->state == ACTIVE)) {

		return interface_handle_g[iface_num];
	}
	return NULL;
}

/**
  * @brief 关闭串行接口
  * @param  serial_ll_hdl - 串行接口句柄
  * @retval rbuffer - 从串行接口读取的缓冲区
  */
static int serial_ll_close(serial_ll_handle_t * serial_ll_hdl)
{
	// 设置状态为销毁
	serial_ll_hdl->state = DESTROY;

	// 如果队列存在，则销毁队列
	if (serial_ll_hdl->queue) {
		vQueueDelete(serial_ll_hdl->queue);  // 使用FreeRTOS的vQueueDelete函数销毁队列
		serial_ll_hdl->queue = NULL;
	}

	// 重置连接
	if (conn_num > 0) {
		interface_handle_g[--conn_num] = NULL;
	}

	// 如果句柄存在，则释放内存
	if (serial_ll_hdl) {
		vPortFree(serial_ll_hdl);  // 使用FreeRTOS的vPortFree函数释放内存
		serial_ll_hdl = NULL;
	}
	return STM_OK;
}

/**
  * @brief  串口非阻塞读取函数（基于FreeRTOS）
  * @param  serial_ll_hdl - 串口句柄
  *         rlen - 输出参数，读取的字节数
  * @retval rbuffer - 从串口读取的缓冲区
  */
static uint8_t * serial_ll_read(const serial_ll_handle_t * serial_ll_hdl,
							 uint16_t * rlen)
{
	interface_buffer_handle_t buf_handle = {0};

	/* 初始化输出参数 */
	*rlen = 0;

	/* 检查串口接口是否有效 */
	if ((!serial_ll_hdl) || (serial_ll_hdl->state != ACTIVE)) {
		ESP_LOGE(TAG, "无效的串口接口");
		return NULL;
	}

	/* 使用FreeRTOS队列进行非阻塞读取
	 * 设置等待时间为0，即立即返回
	 * 如果队列为空，返回pdFAIL
	 */
	if (xQueueReceive(serial_ll_hdl->queue, &buf_handle, 0) != pdPASS) {
		/* 队列为空，没有数据可读 */
		return NULL;
	}

	/* 检查有效负载和长度是否有效 */
	if (!buf_handle.payload || !buf_handle.payload_len) {
		return NULL;
	}

	/* 返回读取的数据长度 */
	*rlen = buf_handle.payload_len;

	return buf_handle.payload;
}

/**
  * @brief Serial interface write
  * @param  serial_ll_hdl - handle
  *         wlen - number of bytes to write
  *         wbuffer - buffer to send
  * @retval STM_FAIL/STM_OK
  */
static int serial_ll_write(const serial_ll_handle_t * serial_ll_hdl,
	uint8_t * wbuffer, const uint16_t wlen)
{

	if ((! serial_ll_hdl) || (serial_ll_hdl->state != ACTIVE)) {
		ESP_LOGE(TAG, "serial invalid interface for write");
		return STM_FAIL;
	}

	return esp_hosted_tx(serial_ll_hdl->if_type,
		serial_ll_hdl->if_num, wbuffer, wlen, H_BUFF_NO_ZEROCOPY, vPortFree);
}

/**
  * @brief 串口接收处理函数，当SPI驱动接收到数据时调用
  * @param  if_num - 接口实例
  *         rxbuff - SPI驱动提供的缓冲区
  *         rx_len - 缓冲区大小
  *         seq_num - 串口序列号
  *         flag_more_frags - 分片标志
  * @retval 0 成功，其他值失败
  */
stm_ret_t serial_ll_rx_handler(interface_buffer_handle_t * buf_handle)
{
#define SERIAL_ALLOC_REALLOC_RDATA() \
	do { \
		if(!r.data) { \
			r.data = (uint8_t *)pvPortMalloc(buf_handle->payload_len); \
		} else { \
			uint8_t *temp = (uint8_t *)pvPortMalloc(r.len + buf_handle->payload_len); \
			if (temp) { \
				memcpy(temp, r.data, r.len); \
				vPortFree(r.data); \
				r.data = temp; \
			} else { \
				ESP_LOGE(TAG, "Failed to allocate serial data"); \
				goto serial_buff_cleanup; \
			} \
		} \
		if (!r.data) { \
			ESP_LOGE(TAG, "Failed to allocate serial data"); \
			goto serial_buff_cleanup; \
		} \
	} while(0);

	serial_ll_handle_t * serial_ll_hdl = NULL;
	uint8_t *serial_buf = NULL;
	interface_buffer_handle_t new_buf_handle = {0};

	/* 检查句柄和长度是否有效 */
	if (!buf_handle || !buf_handle->payload_len) {
		ESP_LOGE(TAG, "%s:%u Invalid parameters", __func__, __LINE__);
		goto serial_buff_cleanup;
	}

	serial_ll_hdl = get_serial_ll_handle(buf_handle->if_num);

	/* 检查串口接口是否已启动 */
	if ((! serial_ll_hdl) || (serial_ll_hdl->state != ACTIVE)) {
		ESP_LOGE(TAG, "Serial interface not registered yet");
		goto serial_buff_cleanup;
	}

	/* 处理分片数据 */
	if (buf_handle->flag & MORE_FRAGMENT) {
		ESP_LOGD(TAG, "Fragment!!!");
		SERIAL_ALLOC_REALLOC_RDATA();

		memcpy((r.data + r.len), buf_handle->payload, buf_handle->payload_len);
		r.len += buf_handle->payload_len;
		return STM_OK;
	}

	SERIAL_ALLOC_REALLOC_RDATA();

	/* 处理非分片或最后一个分片数据 */
	memcpy((r.data + r.len), buf_handle->payload, buf_handle->payload_len);
	r.len += buf_handle->payload_len;

	serial_buf = (uint8_t *)pvPortMalloc(r.len);
	if(!serial_buf) {
		ESP_LOGE(TAG, "Malloc failed, drop pkt");
		goto serial_buff_cleanup;
	}
	memcpy(serial_buf, r.data, r.len);

	/* 为串口消息处理创建新的缓冲区句柄 */
	new_buf_handle.if_type = ESP_SERIAL_IF;
	new_buf_handle.if_num = buf_handle->if_num;
	new_buf_handle.payload_len = r.len;
	new_buf_handle.payload = serial_buf;
	new_buf_handle.priv_buffer_handle = serial_buf;
	new_buf_handle.free_buf_handle = vPortFree;

	/* 清理旧缓冲区 */
	r.len = 0;
	vPortFree(r.data);
	r.data = NULL;

	/* 将数据发送到串口队列 */
	if (xQueueSend(serial_ll_hdl->queue, &new_buf_handle, portMAX_DELAY) != pdPASS) {
		ESP_LOGE(TAG, "Failed send serialif queue[%u]", new_buf_handle.if_num);
		goto serial_buff_cleanup;
	}

	/* 通知上层数据已准备好 */
	if (serial_ll_hdl->serial_rx_callback) {
		(*serial_ll_hdl->serial_rx_callback) ();
	} else {
		goto serial_buff_cleanup;
	}

	return STM_OK;

serial_buff_cleanup:

	vPortFree(buf_handle->priv_buffer_handle);

	r.len = 0;

	vPortFree(new_buf_handle.priv_buffer_handle);

	vPortFree(r.data);
	return STM_FAIL;
}

/** Exported Functions **/

/**
  * @brief create and return new serial interface
  * @param  serial_rx_callback - callback to be invoked on rx data
  * @retval serial_ll_hdl - output handle of serial interface
  */
serial_ll_handle_t * serial_ll_init(void(*serial_rx_callback)(void))
{
	serial_ll_handle_t  * serial_ll_hdl = NULL;

	/* Check if more serial interfaces be created */
	if ((conn_num+1) < MAX_SERIAL_INTF) {

		serial_ll_hdl = (serial_ll_handle_t *)pvPortMalloc(sizeof(serial_ll_handle_t));
		if (! serial_ll_hdl) {
			ESP_LOGE(TAG, "Serial interface - malloc failed");
			return NULL;
		}

		serial_ll_hdl->if_type = ESP_SERIAL_IF;
		serial_ll_hdl->if_num  = conn_num;
		serial_ll_hdl->queue   = to_serial_ll_intf_queue[conn_num];
		serial_ll_hdl->state   = INIT;
		serial_ll_hdl->fops    = &serial_ll_fops;
		serial_ll_hdl->serial_rx_callback   = serial_rx_callback;
		interface_handle_g[conn_num] = serial_ll_hdl;
		conn_num++;

	} else {
		ESP_LOGE(TAG, "Number of serial interface connections overflow");
		return NULL;
	}

	return serial_ll_hdl;
}
