//
// Created by TaterLi on 25-2-8.
//

#include "transport.h"

#include <esp_hosted_api.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "adapter.h"
#include "mempool.h"
#include "errno.h"
#include "common.h"
#include "inttypes.h"
#include "serial_ll_if.h"

#include "esp_netif.h"
#include "esp_config.h"

static const char TAG[] = "transport";

// 引脚配置
#define GPIO_HANDSHAKE 6
#define GPIO_DATAREADY 5
#define GPIO_RESET 7

#define GPIO_SPI_MOSI 3
#define GPIO_SPI_MISO 4
#define GPIO_SPI_SCK 2
#define GPIO_SPI_CS 8

// 任务相关的一些设置
#define RPC_TASK_STACK_SIZE                          (5*1024)
#define RPC_TASK_PRIO                                23
#define DFLT_TASK_STACK_SIZE                         (5*1024)
#define DFLT_TASK_PRIO                               23

#define MAX_RETRY_TRANSPORT_ACTIVE                   1000

static SemaphoreHandle_t spi_trans_ready_sem;

static QueueHandle_t to_slave_queue[MAX_PRIORITY_QUEUES];  // 使用FreeRTOS的QueueHandle_t定义发送队列
static SemaphoreHandle_t sem_to_slave_queue;              // 使用FreeRTOS的SemaphoreHandle_t定义发送信号量
static QueueHandle_t from_slave_queue[MAX_PRIORITY_QUEUES]; // 使用FreeRTOS的QueueHandle_t定义接收队列
static SemaphoreHandle_t sem_from_slave_queue;             // 使用FreeRTOS的SemaphoreHandle_t定义接收信号量

static SemaphoreHandle_t spi_bus_lock;  // 使用FreeRTOS的SemaphoreHandle_t定义SPI总线锁

static TaskHandle_t spi_transaction_thread;
static TaskHandle_t spi_rx_thread;

static uint8_t transport_state = TRANSPORT_INACTIVE;

void(*transport_esp_hosted_up_cb)(void) = NULL;

static void spi_transaction_task(void * pvParameters);
static void spi_process_rx_task(void * pvParameters);

static void cs_select(void);
static void cs_deselect(void);

static uint8_t is_transport_rx_ready(void);
static uint8_t is_transport_tx_ready(void);

static void reset_slave(void);

static void transport_driver_event_handler(uint8_t event);

static void transport_sta_free_cb(void *buf);
static void transport_ap_free_cb(void *buf);
static void transport_serial_free_cb(void *buf);

static esp_err_t transport_drv_sta_tx(void *h, void *buffer, size_t len);
static esp_err_t transport_drv_ap_tx(void *h, void *buffer, size_t len);

static void process_event(uint8_t *evt_buf, uint16_t len);
static uint32_t process_ext_capabilities(uint8_t * ptr);
static int process_init_event(uint8_t *evt_buf, uint16_t len);

static void print_capabilities(uint32_t cap);

static int check_and_execute_spi_transaction(void);

transport_channel_t *chan_arr[ESP_MAX_IF];
static char chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
volatile uint8_t wifi_tx_throttling;

static uint8_t schedule_dummy_rx = 0;

static struct mempool * buf_mp_g;

/**
  * @brief  传输层初始化
  * @param  transport_evt_handler_fp - 事件处理函数
  * @retval esp_err_t
  */
esp_err_t transport_drv_init(void(*esp_hosted_up_cb)(void))
{
    uint8_t prio_q_idx;

    // 使用FreeRTOS API创建互斥锁
    spi_bus_lock = xSemaphoreCreateMutex();
    configASSERT(spi_bus_lock != NULL);  // 使用FreeRTOS风格的assert

    // 使用FreeRTOS API创建信号量
    sem_to_slave_queue = xSemaphoreCreateCounting(TO_SLAVE_QUEUE_SIZE * MAX_PRIORITY_QUEUES, 0);
    configASSERT(sem_to_slave_queue != NULL);
    sem_from_slave_queue = xSemaphoreCreateCounting(FROM_SLAVE_QUEUE_SIZE * MAX_PRIORITY_QUEUES, 0);
    configASSERT(sem_from_slave_queue != NULL);

    // 初始化优先级队列
    for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
        // 创建接收队列
        from_slave_queue[prio_q_idx] = xQueueCreate(FROM_SLAVE_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
        configASSERT(from_slave_queue[prio_q_idx] != NULL);

        // 创建发送队列
        to_slave_queue[prio_q_idx] = xQueueCreate(TO_SLAVE_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
        configASSERT(to_slave_queue[prio_q_idx] != NULL);
    }

    // 创建并初始化SPI传输就绪信号量
    spi_trans_ready_sem = xSemaphoreCreateBinary();
    configASSERT(spi_trans_ready_sem != NULL);
    xSemaphoreGive(spi_trans_ready_sem);  // 初始化为可用状态

    // 初始化SPI总线
    spi_init(spi0, 4000 * 1000);
    gpio_set_function(GPIO_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI_SCK, GPIO_FUNC_SPI);

    gpio_init(GPIO_SPI_CS);
    gpio_set_dir(GPIO_SPI_CS, GPIO_OUT);

    // 复位信号
    gpio_init(GPIO_RESET);
    gpio_set_dir(GPIO_RESET, GPIO_OUT);

    // 创建SPI传输任务
    xTaskCreate(spi_transaction_task, "spi_trans", DFLT_TASK_STACK_SIZE, NULL, DFLT_TASK_PRIO, &spi_transaction_thread);
    configASSERT(spi_transaction_thread != NULL);

    // 创建SPI接收处理任务
    xTaskCreate(spi_process_rx_task, "spi_rx", DFLT_TASK_STACK_SIZE, NULL, DFLT_TASK_PRIO, &spi_rx_thread);
    configASSERT(spi_rx_thread != NULL);

    return ESP_OK;
}

esp_err_t transport_drv_deinit(void)
{
    transport_state = TRANSPORT_INACTIVE;
    return ESP_OK;
}

esp_err_t transport_drv_reconfigure(void)
{
    static int retry_slave_connection = 0;

    ESP_LOGI(TAG, "Attempt connection with slave: retry[%u]",retry_slave_connection);
    if (!is_transport_tx_ready()) {
        reset_slave();
        transport_state = TRANSPORT_RX_ACTIVE;

        while (!is_transport_tx_ready()) {
            if (retry_slave_connection < MAX_RETRY_TRANSPORT_ACTIVE) {
                retry_slave_connection++;
                if (retry_slave_connection%10==0) {
                    ESP_LOGE(TAG, "Not able to connect with ESP-Hosted slave device");
                    reset_slave();
                }
            } else {
                ESP_LOGE(TAG, "Failed to get ESP_Hosted slave transport up");
                return ESP_FAIL;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    } else {
        ESP_LOGI(TAG, "Transport is already up");
    }
    retry_slave_connection = 0;
    return ESP_OK;
}

esp_err_t transport_drv_remove_channel(transport_channel_t *channel)
{
    if (!channel)
        return ESP_FAIL;

    switch (channel->if_type) {
    case ESP_AP_IF:
    case ESP_STA_IF:
        //Should we additionally do:
        esp_wifi_internal_reg_rxcb(channel->if_type, NULL);
        break;
    case ESP_SERIAL_IF:
        /* TODO */
            break;
    default:
        break;
    }

    assert(chan_arr[channel->if_type] == channel);

    mempool_destroy(channel->memp);
    chan_arr[channel->if_type] = NULL;
    HOSTED_FREE(channel);

    return ESP_OK;
}


esp_err_t transport_drv_serial_tx(void *h, void *buffer, size_t len)
{
    /* TODO */
    // 断言检查传入的句柄h是否有效，并且与通道数组中的ESP_SERIAL_IF通道匹配
    assert(h && h==chan_arr[ESP_SERIAL_IF]->api_chan);
    // 调用esp_hosted_tx函数进行串口数据传输，使用非零拷贝模式，并指定传输完成后的回调函数
    return esp_hosted_tx(ESP_SERIAL_IF, 0, buffer, len, H_BUFF_NO_ZEROCOPY, transport_serial_free_cb);
}

transport_channel_t *transport_drv_add_channel(void *api_chan,
        esp_hosted_if_type_t if_type, uint8_t secure,
        transport_channel_tx_fn_t *tx, const transport_channel_rx_fn_t rx)
{
    transport_channel_t *channel = NULL;

    // 检查接口类型是否有效，如果无效则触发错误
    ESP_ERROR_CHECK(if_type >= ESP_MAX_IF);

    // 检查传输和接收回调函数是否为空，如果为空则记录错误并返回NULL
    if (!tx || !rx) {
        ESP_LOGE(TAG, "%s fail for IF[%u]: tx or rx is NULL", __func__, if_type );
        return NULL;
    }

    // 检查通道数组中的对应接口类型是否已经存在，如果存在则释放原有通道并记录警告
    if (chan_arr[if_type]) {
        /* Channel config already existed */
        ESP_LOGW(TAG, "Channel [%u] already created, replace with new callbacks", if_type);
        HOSTED_FREE(chan_arr[if_type]);
    }

    // 为新的通道分配内存，并检查分配是否成功
    chan_arr[if_type] = pvPortCalloc(sizeof(transport_channel_t), 1);
    assert(chan_arr[if_type]);
    channel = chan_arr[if_type];

    // 根据接口类型设置对应的传输回调函数
    switch (if_type) {

    case ESP_STA_IF:
        *tx = transport_drv_sta_tx;
        break;

    case ESP_AP_IF:
        *tx = transport_drv_ap_tx;
        break;

    case ESP_SERIAL_IF:
        *tx = transport_drv_serial_tx;
        break;

    default:
        //*tx = transport_drv_tx;
            ESP_LOGW(TAG, "Not yet suppported ESP_Hosted interface for if_type[%u]", if_type);
        return NULL;
    }

    // 初始化通道结构体的各个字段
    channel->api_chan = api_chan;
    channel->if_type = if_type;
    channel->secure = secure;
    channel->tx = *tx;
    channel->rx = rx;

    // 创建内存池用于传输缓冲区，并检查内存池是否创建成功
    /* Need to change size wrt transport */
    channel->memp = mempool_create(MAX_TRANSPORT_BUFFER_SIZE);
#ifdef H_USE_MEMPOOL
    assert(channel->memp);
#endif

    // 记录通道添加成功的日志信息
    ESP_LOGI(TAG, "Add ESP-Hosted channel IF[%u]: S[%u] Tx[%p] Rx[%p]",
            if_type, secure, *tx, rx);

    return channel;
}

/**
  * @brief  调度SPI事务，如果满足以下条件：
  * a. SPI主机（STM）有有效的TX缓冲区
  * b. SPI外设（ESP）有有效的TX缓冲区
  * c. 预期从SPI外设（ESP）接收到虚拟事务
  * @param  argument: 未使用
  * @retval None
  */
static int process_spi_rx_buf(uint8_t * rxbuff)
{
    struct esp_payload_header *payload_header;
    uint16_t rx_checksum = 0, checksum = 0;
    interface_buffer_handle_t buf_handle = {0};
    uint16_t len, offset;
    int ret = 0;
    uint8_t pkt_prio = PRIO_Q_OTHERS;

    if (!rxbuff)
        return -1;

    ESP_HEXLOGV("h_spi_rx", rxbuff, 32);

    /* 创建缓冲区接收句柄，用于处理 */
    payload_header = (struct esp_payload_header *) rxbuff;

    /* 从负载头中获取长度和偏移量 */
    len = le16toh(payload_header->len);
    offset = le16toh(payload_header->offset);

    if (ESP_MAX_IF == payload_header->if_type)
        schedule_dummy_rx = 0;

    if (!len) {
        wifi_tx_throttling = payload_header->throttle_cmd;
        ret = -5;
        goto done;
    }

    if ((len > MAX_TRANSPORT_BUFFER_SIZE) ||
        (offset != sizeof(struct esp_payload_header))) {
        ESP_LOGI(TAG, "rx packet ignored: len [%u], rcvd_offset[%u], exp_offset[%u]\n",
                len, offset, sizeof(struct esp_payload_header));

        /* 1. 没有有效负载需要处理
         * 2. 输入数据包大小超过驱动容量
         * 3. 负载头大小不匹配，可能是错误的头/位打包？
         */
        ret = -2;
        goto done;

    } else {
        rx_checksum = le16toh(payload_header->checksum);
        payload_header->checksum = 0;

        checksum = compute_checksum(rxbuff, len+offset);
        //TODO: checksum needs to be configurable from menuconfig
        if (checksum == rx_checksum) {
            buf_handle.priv_buffer_handle = rxbuff;
            buf_handle.free_buf_handle = vPortFree;
            buf_handle.payload_len = len;
            buf_handle.if_type     = payload_header->if_type;
            buf_handle.if_num      = payload_header->if_num;
            buf_handle.payload     = rxbuff + offset;
            buf_handle.seq_num     = le16toh(payload_header->seq_num);
            buf_handle.flag        = payload_header->flags;
            wifi_tx_throttling     = payload_header->throttle_cmd;
            if (buf_handle.if_type == ESP_SERIAL_IF)
                pkt_prio = PRIO_Q_SERIAL;
            else if (buf_handle.if_type == ESP_HCI_IF)
                pkt_prio = PRIO_Q_BT;
            /* 默认情况下为OTHERS */

            /* 使用FreeRTOS的队列API将数据包放入队列 */
            if (xQueueSend(from_slave_queue[pkt_prio], &buf_handle, HOSTED_BLOCK_MAX) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send item to queue");
                ret = -6;
                goto done;
            }

            /* 使用FreeRTOS的信号量API通知任务有新数据 */
            if (xSemaphoreGive(sem_from_slave_queue) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to give semaphore");
                ret = -7;
                goto done;
            }

        } else {
            ESP_LOGI(TAG, "rcvd_crc[%u] != exp_crc[%u], drop pkt\n",checksum, rx_checksum);
            ret = -4;
            goto done;
        }
    }

    return ret;

done:
    /* 错误情况，中止 */
    if (rxbuff) {
        vPortFree(rxbuff);
        rxbuff = NULL;
    }

    return ret;
}

static void process_capabilities(uint8_t cap)
{
    ESP_LOGI(TAG, "capabilities: 0x%x",cap);
}

static void process_priv_communication(interface_buffer_handle_t *buf_handle)
{
    if (!buf_handle || !buf_handle->payload || !buf_handle->payload_len)
        return;

    process_event(buf_handle->payload, buf_handle->payload_len);
}

static void print_capabilities(uint32_t cap)
{
    ESP_LOGI(TAG, "Features supported are:");
    if (cap & ESP_WLAN_SDIO_SUPPORT)
        ESP_LOGI(TAG, "\t * WLAN");
    if (cap & ESP_BT_UART_SUPPORT)
        ESP_LOGI(TAG, "\t   - HCI over UART");
    if (cap & ESP_BT_SDIO_SUPPORT)
        ESP_LOGI(TAG, "\t   - HCI over SDIO");
    if (cap & ESP_BT_SPI_SUPPORT)
        ESP_LOGI(TAG, "\t   - HCI over SPI");
    if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
        ESP_LOGI(TAG, "\t   - BT/BLE dual mode");
    else if (cap & ESP_BLE_ONLY_SUPPORT)
        ESP_LOGI(TAG, "\t   - BLE only");
    else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
        ESP_LOGI(TAG, "\t   - BR EDR only");
}

static uint32_t process_ext_capabilities(uint8_t * ptr)
{
    // ptr address may be not be 32-bit aligned
    uint32_t cap;

    cap = (uint32_t)ptr[0] +
        ((uint32_t)ptr[1] << 8) +
        ((uint32_t)ptr[2] << 16) +
        ((uint32_t)ptr[3] << 24);
    ESP_LOGI(TAG, "extended capabilities: 0x%"PRIx32,cap);

    return cap;
}

static esp_err_t get_chip_str_from_id(int chip_id, char* chip_str)
{
    int ret = ESP_OK;
    assert(chip_str);

    switch(chip_id) {
    case ESP_PRIV_FIRMWARE_CHIP_ESP32:
        strcpy(chip_str, "esp32");
        break;
    case ESP_PRIV_FIRMWARE_CHIP_ESP32C2:
        strcpy(chip_str, "esp32c2");
        break;
    case ESP_PRIV_FIRMWARE_CHIP_ESP32C3:
        strcpy(chip_str, "esp32c3");
        break;
    case ESP_PRIV_FIRMWARE_CHIP_ESP32C6:
        strcpy(chip_str, "esp32c6");
        break;
    case ESP_PRIV_FIRMWARE_CHIP_ESP32S2:
        strcpy(chip_str, "esp32s2");
        break;
    case ESP_PRIV_FIRMWARE_CHIP_ESP32S3:
        strcpy(chip_str, "esp32s3");
        break;
    case ESP_PRIV_FIRMWARE_CHIP_ESP32C5:
        strcpy(chip_str, "esp32c5");
        break;
    default:
        ESP_LOGW(TAG, "Unsupported chip id: %u", chip_id);
        strcpy(chip_str, "unsupported");
        ret = ESP_FAIL;
        break;
    }
    return ret;
}

static void verify_host_config_for_slave(uint8_t chip_type)
{
    uint8_t exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32C3;
    if (chip_type!=exp_chip_id) {
        char slave_str[20], exp_str[20];

        memset(slave_str, '\0', 20);
        memset(exp_str, '\0', 20);

        get_chip_str_from_id(chip_type, slave_str);
        get_chip_str_from_id(exp_chip_id, exp_str);
        ESP_LOGE(TAG, "Identified slave [%s] != Expected [%s]\n\t\trun 'idf.py menuconfig' at host to reselect the slave?\n\t\tAborting.. ", slave_str, exp_str);
        vTaskDelay(pdMS_TO_TICKS(10));
        assert(0!=0);
    }
}

static esp_err_t send_slave_config(uint8_t host_cap, uint8_t firmware_chip_id,
        uint8_t raw_tp_direction, uint8_t low_thr_thesh, uint8_t high_thr_thesh)
{
#define LENGTH_1_BYTE 1
    struct esp_priv_event *event = NULL;
    uint8_t *pos = NULL;
    uint16_t len = 0;
    uint8_t *sendbuf = NULL;

    sendbuf = pvPortMalloc(512);
    assert(sendbuf);

    /* Populate event data */
    //event = (struct esp_priv_event *) (sendbuf + sizeof(struct esp_payload_header)); //ZeroCopy
    event = (struct esp_priv_event *) (sendbuf);

    event->event_type = ESP_PRIV_EVENT_INIT;

    /* Populate TLVs for event */
    pos = event->event_data;

    /* TLVs start */

    /* TLV - Board type */
    ESP_LOGI(TAG, "Slave chip Id[%x]", ESP_PRIV_FIRMWARE_CHIP_ID);
    *pos = HOST_CAPABILITIES;                          pos++;len++;
    *pos = LENGTH_1_BYTE;                              pos++;len++;
    *pos = host_cap;                                   pos++;len++;

    /* TLV - Capability */
    *pos = RCVD_ESP_FIRMWARE_CHIP_ID;                  pos++;len++;
    *pos = LENGTH_1_BYTE;                              pos++;len++;
    *pos = firmware_chip_id;                           pos++;len++;

    *pos = SLV_CONFIG_TEST_RAW_TP;                     pos++;len++;
    *pos = LENGTH_1_BYTE;                              pos++;len++;
    *pos = raw_tp_direction;                           pos++;len++;

    *pos = SLV_CONFIG_THROTTLE_HIGH_THRESHOLD;         pos++;len++;
    *pos = LENGTH_1_BYTE;                              pos++;len++;
    *pos = high_thr_thesh;                             pos++;len++;

    *pos = SLV_CONFIG_THROTTLE_LOW_THRESHOLD;          pos++;len++;
    *pos = LENGTH_1_BYTE;                              pos++;len++;
    *pos = low_thr_thesh;                              pos++;len++;

    /* TLVs end */

    event->event_len = len;

    /* payload len = Event len + sizeof(event type) + sizeof(event len) */
    len += 2;

    return esp_hosted_tx(ESP_PRIV_IF, 0, sendbuf, len, H_BUFF_NO_ZEROCOPY, vPortFree);
}

static int process_init_event(uint8_t *evt_buf, uint16_t len)
{
	uint8_t len_left = len, tag_len;
	uint8_t *pos;
	uint8_t raw_tp_config = ESP_TEST_RAW_TP_NONE;
	uint32_t ext_cap = 0;

	if (!evt_buf)
		return ESP_FAIL;

	pos = evt_buf;
	ESP_LOGD(TAG, "Init event length: %u", len);
	if (len > 64) {
		ESP_LOGE(TAG, "Init event length: %u", len);
#if H_TRANSPORT_IN_USE == H_TRANSPORT_SPI
		ESP_LOGE(TAG, "Seems incompatible SPI mode try changing SPI mode. Asserting for now.");
#endif
		assert(len < 64);
	}

	while (len_left) {
		tag_len = *(pos + 1);

		if (*pos == ESP_PRIV_CAPABILITY) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			process_capabilities(*(pos + 2));
			print_capabilities(*(pos + 2));
		} else if (*pos == ESP_PRIV_CAP_EXT) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			ext_cap = process_ext_capabilities(pos + 2);
		} else if (*pos == ESP_PRIV_FIRMWARE_CHIP_ID) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			chip_type = *(pos+2);
			verify_host_config_for_slave(chip_type);
		} else if (*pos == ESP_PRIV_TEST_RAW_TP) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
#if TEST_RAW_TP
			process_test_capabilities(*(pos + 2));
#else
			if (*(pos + 2))
				ESP_LOGW(TAG, "Slave enabled Raw Throughput Testing, but not enabled on Host");
#endif
		} else if (*pos == ESP_PRIV_RX_Q_SIZE) {
			ESP_LOGD(TAG, "slave rx queue size: %u", *(pos + 2));
		} else if (*pos == ESP_PRIV_TX_Q_SIZE) {
			ESP_LOGD(TAG, "slave tx queue size: %u", *(pos + 2));
		} else {
			ESP_LOGD(TAG, "Unsupported EVENT: %2x", *pos);
		}
		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	if ((chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S2) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S3) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C2) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C3) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C6) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C5)) {
		ESP_LOGI(TAG, "ESP board type is not mentioned, ignoring [%d]\n\r", chip_type);
		chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
		return -1;
	} else {
		ESP_LOGI(TAG, "ESP board type is : %d \n\r", chip_type);
	}

	transport_driver_event_handler(TRANSPORT_TX_ACTIVE);
	return send_slave_config(0, chip_type, raw_tp_config,
		H_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD,
		H_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD);
}

int serial_rx_handler(interface_buffer_handle_t * buf_handle)
{
	return serial_ll_rx_handler(buf_handle);
}

static void process_event(uint8_t *evt_buf, uint16_t len)
{
    int ret = 0;
    struct esp_priv_event *event;

    if (!evt_buf || !len)
        return;

    event = (struct esp_priv_event *) evt_buf;

    if (event->event_type == ESP_PRIV_EVENT_INIT) {

        ESP_LOGI(TAG, "Received INIT event from ESP32 peripheral");
        ESP_HEXLOGD("Slave_init_evt", event->event_data, event->event_len);

        ret = process_init_event(event->event_data, event->event_len);
        if (ret) {
            ESP_LOGE(TAG, "failed to init event\n\r");
        }
    } else {
        ESP_LOGW(TAG, "Drop unknown event\n\r");
    }
}

// 中断内的printf容易死锁,可能是中断优先级?
static void gpio_callback(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    switch (gpio)
    {
        case GPIO_HANDSHAKE:
            xSemaphoreGiveFromISR(spi_trans_ready_sem,&xHigherPriorityTaskWoken );
            // ESP_LOGV(TAG, "%s", "GPIO_HANDSHAKE");
            break;
        case GPIO_DATAREADY:
            xSemaphoreGiveFromISR(spi_trans_ready_sem,&xHigherPriorityTaskWoken );
            // ESP_LOGV(TAG, "%s", "GPIO_DATAREADY");
            break;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void spi_transaction_task(void* pvParameters)
{
    // 使用FreeRTOS的日志函数替代ESP_LOGD
    ESP_LOGD(TAG, "Staring SPI task");

    // 配置GPIO中断
    gpio_init(GPIO_HANDSHAKE);
    gpio_init(GPIO_DATAREADY);

    gpio_set_dir(GPIO_HANDSHAKE, GPIO_IN);
    gpio_set_dir(GPIO_DATAREADY, GPIO_IN);

    gpio_set_irq_enabled_with_callback(GPIO_HANDSHAKE,
                                     GPIO_IRQ_EDGE_RISE,
                                     true,
                                     gpio_callback);

    gpio_set_irq_enabled_with_callback(GPIO_DATAREADY,
                                     GPIO_IRQ_EDGE_RISE,
                                     true,
                                     gpio_callback);

    ESP_LOGD(TAG, "SPI GPIOs configured");

    vTaskDelay(1000);

    for (;;) {
        // 如果信号量都还没创建,那么任务等一会信号量.
        if (!spi_trans_ready_sem) {
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        // 使用FreeRTOS信号量获取函数
        if (xSemaphoreTake(spi_trans_ready_sem, portMAX_DELAY) == pdTRUE) {
            check_and_execute_spi_transaction();
        }
    }
 }

/**
  * @brief  获取下一个SPI事务的TX缓冲区
  * @param  is_valid_tx_buf: 用于指示是否有有效数据的指针
  * @param  free_func: 用于释放缓冲区的函数指针
  * @retval sendbuf - 返回的Tx缓冲区指针
  */
static uint8_t * get_next_tx_buffer(uint8_t *is_valid_tx_buf, void (**free_func)(void* ptr))
{
    struct  esp_payload_header *payload_header;
    uint8_t *sendbuf = NULL;
    uint8_t *payload = NULL;
    uint16_t len = 0;
    interface_buffer_handle_t buf_handle = {0};
    uint8_t tx_needed = 1;

    assert(is_valid_tx_buf);
    assert(free_func);

    *is_valid_tx_buf = 0;

    /* 检查高层是否有数据需要传输，非阻塞方式。
     * 如果没有数据需要发送，队列接收将失败。
     * 在这种情况下，只发送带有零负载长度的负载头。
     */

    if (xSemaphoreTake(sem_to_slave_queue, 0) == pdTRUE) {

        /* 根据信号量判断是否有Tx消息 */
        if (xQueueReceive(to_slave_queue[PRIO_Q_SERIAL], &buf_handle, 0) == pdFALSE)
            if (xQueueReceive(to_slave_queue[PRIO_Q_BT], &buf_handle, 0) == pdFALSE)
                if (xQueueReceive(to_slave_queue[PRIO_Q_OTHERS], &buf_handle, 0) == pdFALSE) {
                    tx_needed = 0; /* 没有Tx消息 */
                }

        if (tx_needed)
            len = buf_handle.payload_len;
    }

    if (len) {
        ESP_HEXLOGD("h_spi_tx", buf_handle.payload, 16);

        if (!buf_handle.payload_zcopy) {
            sendbuf = pvPortMalloc(MAX_SPI_BUFFER_SIZE);
            assert(sendbuf);
            *free_func = vPortFree;
        } else {
            sendbuf = buf_handle.payload;
            *free_func = buf_handle.free_buf_handle;
        }

        if (!sendbuf) {
            ESP_LOGE(TAG, "spi buff malloc failed");
            *free_func = NULL;
            goto done;
        }

        *is_valid_tx_buf = 1;

        /* 构造Tx头 */
        payload_header = (struct esp_payload_header *) sendbuf;
        payload = sendbuf + sizeof(struct esp_payload_header);
        payload_header->len     = htole16(len);
        payload_header->offset  = htole16(sizeof(struct esp_payload_header));
        payload_header->if_type = buf_handle.if_type;
        payload_header->if_num  = buf_handle.if_num;

        if (payload_header->if_type == ESP_HCI_IF) {
            // 对HCI的特殊处理
            if (!buf_handle.payload_zcopy) {
                // 将负载的第一个字节复制到头部
                payload_header->hci_pkt_type = buf_handle.payload[0];
                // 调整实际负载长度
                len -= 1;
                payload_header->len = htole16(len);
                memcpy(payload, &buf_handle.payload[1], len);
            }
        } else
        if (!buf_handle.payload_zcopy)
            memcpy(payload, buf_handle.payload, min(len, MAX_TRANSPORT_BUFFER_SIZE));

        payload_header->checksum = htole16(compute_checksum(sendbuf,
                sizeof(struct esp_payload_header)+len));
    }

done:
    if (len && !buf_handle.payload_zcopy) {
        /* 释放分配的缓冲区，仅在未请求零拷贝时 */
        if (buf_handle.free_buf_handle)
            buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
    }

    return sendbuf;
}

static int check_and_execute_spi_transaction(void)
{
    uint8_t *txbuff = NULL;
    uint8_t *rxbuff = NULL;
    uint8_t is_valid_tx_buf = 0;
    void (*tx_buff_free_func)(void* ptr) = NULL;
    struct esp_payload_header *h = NULL;
    static uint8_t schedule_dummy_tx = 0;

    uint32_t ret = 0;
    int gpio_handshake = 0;
    int gpio_rx_data_ready = 0;

    // 使用FreeRTOS的互斥锁来保护SPI总线访问
    xSemaphoreTake(spi_bus_lock, portMAX_DELAY);

    /* 检查握手信号线，判断从设备是否准备好进行下一次传输 */
    gpio_handshake = gpio_get(GPIO_HANDSHAKE);

    /* 检查数据就绪信号线，判断从设备是否有数据要发送 */
    gpio_rx_data_ready = gpio_get(GPIO_DATAREADY);

    if (gpio_handshake) {

        /* 获取下一个要发送的TX缓冲区 */
        txbuff = get_next_tx_buffer(&is_valid_tx_buf, &tx_buff_free_func);

        if ( (gpio_rx_data_ready) ||
                (is_valid_tx_buf) || schedule_dummy_tx || schedule_dummy_rx ) {

            if (!txbuff) {
                /* 即使没有数据要发送，也需要一个有效的TX缓冲区供SPI驱动使用 */
                txbuff = pvPortMalloc(MAX_SPI_BUFFER_SIZE);
                configASSERT(txbuff);

                h = (struct esp_payload_header *) txbuff;
                h->if_type = ESP_MAX_IF;
                tx_buff_free_func = vPortFree;
                schedule_dummy_tx = 0;
            } else {
                schedule_dummy_tx = 1;
                ESP_HEXLOGV("h_spi_tx", txbuff, 16);
            }

            ESP_LOGV(TAG, "dr %u tx_valid %u\n", gpio_rx_data_ready, is_valid_tx_buf);
            /* 分配RX缓冲区 */
            rxbuff = pvPortMalloc(MAX_SPI_BUFFER_SIZE);
            configASSERT(rxbuff);

            /* 只有在以下任一条件成立时才执行传输：
             * a. 有有效的TX缓冲区要发送给从设备
             * b. 从设备有数据要发送（主机接收）
             */

            cs_select();
            ret = spi_write_read_blocking(spi0,txbuff, rxbuff, MAX_SPI_BUFFER_SIZE);
            cs_deselect();

             if (MAX_SPI_BUFFER_SIZE == ret)
                process_spi_rx_buf(rxbuff);
        }

        if (txbuff && tx_buff_free_func) {
            tx_buff_free_func(txbuff);
        }
    }
    if ((!gpio_handshake) || schedule_dummy_tx || schedule_dummy_rx)
        xSemaphoreGive(spi_trans_ready_sem);

    // 释放SPI总线互斥锁
    xSemaphoreGive(spi_bus_lock);

    return ret;
}


/**
  * @brief  RX处理任务
  * @param  pvParameters: FreeRTOS任务参数（未使用）
  * @retval None
  */
static void spi_process_rx_task(void * pvParameters)
{
	interface_buffer_handle_t buf_handle_l = {0};
	interface_buffer_handle_t *buf_handle = NULL;
	int ret = 0;

	while (1) {
		// 使用FreeRTOS信号量替代原有信号量机制
		if (xSemaphoreTake(sem_from_slave_queue, portMAX_DELAY) != pdTRUE) {
			ESP_LOGE(TAG, "Failed to take semaphore");
			continue;
		}

		// 使用FreeRTOS队列机制替代原有队列机制
		if (xQueueReceive(from_slave_queue[PRIO_Q_SERIAL], &buf_handle_l, 0) != pdTRUE)
			if (xQueueReceive(from_slave_queue[PRIO_Q_BT], &buf_handle_l, 0) != pdTRUE)
				if (xQueueReceive(from_slave_queue[PRIO_Q_OTHERS], &buf_handle_l, 0) != pdTRUE) {
				    ESP_LOGI(TAG, "No element in any queue found");
					continue;
				}

		buf_handle = &buf_handle_l;

		struct esp_priv_event *event = NULL;

		/* 处理接收到的缓冲区，针对所有可能的接口类型 */
		if (buf_handle->if_type == ESP_SERIAL_IF) {
			/* 串口接口路径 */
			serial_rx_handler(buf_handle);

		} else if((buf_handle->if_type == ESP_STA_IF) ||
				(buf_handle->if_type == ESP_AP_IF)) {
			schedule_dummy_rx = 1;
			if (chan_arr[buf_handle->if_type] && chan_arr[buf_handle->if_type]->rx) {
				/* 使用FreeRTOS内存管理替代原有内存分配 */
				uint8_t * copy_payload = (uint8_t *)pvPortMalloc(buf_handle->payload_len);
				assert(copy_payload);
				memcpy(copy_payload, buf_handle->payload, buf_handle->payload_len);
				H_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle, buf_handle->priv_buffer_handle);

				ret = chan_arr[buf_handle->if_type]->rx(chan_arr[buf_handle->if_type]->api_chan,
						copy_payload, copy_payload, buf_handle->payload_len);
				if (unlikely(ret))
					vPortFree(copy_payload);
			}
		} else if (buf_handle->if_type == ESP_PRIV_IF) {
			process_priv_communication(buf_handle);
			// hci_drv_show_configuration();
			/* 接收到私有事务 */
		    ESP_LOGI(TAG, "Received INIT event");

			event = (struct esp_priv_event *) (buf_handle->payload);
			if (event->event_type != ESP_PRIV_EVENT_INIT) {
				/* 用户可以重用此类事务 */
			}
		} else if (buf_handle->if_type == ESP_HCI_IF) {
		    // 蓝牙未实现
		    // hci_rx_handler(buf_handle);
		} else if (buf_handle->if_type == ESP_TEST_IF) {
#if TEST_RAW_TP
			update_test_raw_tp_rx_len(buf_handle->payload_len+H_ESP_PAYLOAD_HEADER_OFFSET);
#endif
		} else {
		    ESP_LOGW(TAG, "unknown type %d ", buf_handle->if_type);
		}
		/* 释放缓冲区句柄 */
		/* 当缓冲区被卸载到其他模块时，该模块负责释放缓冲区。
		 * 如果未卸载或卸载失败，应在此处释放缓冲区。
		 */
		if (!buf_handle->payload_zcopy) {
			H_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,buf_handle->priv_buffer_handle);
		}
	}
}


static void cs_select(void) {
    for (uint8_t i = 0;i<5;i++)
    {
        gpio_put(GPIO_SPI_CS, 0);
    }
}

static void cs_deselect(void) {
    for (uint8_t i = 0;i<5;i++)
    {
        gpio_put(GPIO_SPI_CS, 1     );
    }
}

static uint8_t is_transport_rx_ready(void)
{
    return (transport_state >= TRANSPORT_RX_ACTIVE);
}

static uint8_t is_transport_tx_ready(void)
{
    return (transport_state >= TRANSPORT_TX_ACTIVE);
}

static void reset_slave(void)
{
    ESP_LOGI(TAG, "Reset slave using GPIO");

    gpio_put(GPIO_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_put(GPIO_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

static void transport_driver_event_handler(uint8_t event)
{
    switch(event)
    {
    case TRANSPORT_TX_ACTIVE:
        {
            /* Initiate control path now */
            ESP_LOGI(TAG, "Base transport is set-up\n\r");
            if (transport_esp_hosted_up_cb)
                transport_esp_hosted_up_cb();
            transport_state = TRANSPORT_TX_ACTIVE;
            break;
        }

    default:
        break;
    }
}

static void transport_sta_free_cb(void *buf)
{
    mempool_free(chan_arr[ESP_STA_IF]->memp, buf);
}

static void transport_ap_free_cb(void *buf)
{
    mempool_free(chan_arr[ESP_AP_IF]->memp, buf);
}

static void transport_serial_free_cb(void *buf)
{
    mempool_free(chan_arr[ESP_SERIAL_IF]->memp, buf);
}

static esp_err_t transport_drv_sta_tx(void *h, void *buffer, size_t len)
{
    void * copy_buff = NULL;

    if (!buffer || !len)
        return ESP_OK;

    if (unlikely(wifi_tx_throttling)) {
        errno = -ENOBUFS;
        //return ESP_ERR_NO_BUFFS;
#if defined(ESP_ERR_ESP_NETIF_TX_FAILED)
        return ESP_ERR_ESP_NETIF_TX_FAILED;
#else
        return ESP_ERR_ESP_NETIF_NO_MEM;
#endif
    }

    assert(h && h==chan_arr[ESP_STA_IF]->api_chan);

    /*  Prepare transport buffer directly consumable */
    copy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_STA_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
    assert(copy_buff);
    memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);

    return esp_hosted_tx(ESP_STA_IF, 0, copy_buff, len, H_BUFF_ZEROCOPY, transport_sta_free_cb);
}

static esp_err_t transport_drv_ap_tx(void *h, void *buffer, size_t len)
{
    void * copy_buff = NULL;

    if (!buffer || !len)
        return ESP_OK;

    assert(h && h==chan_arr[ESP_AP_IF]->api_chan);

    /*  Prepare transport buffer directly consumable */
    copy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_AP_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
    assert(copy_buff);
    memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);

    return esp_hosted_tx(ESP_AP_IF, 0, copy_buff, len, H_BUFF_ZEROCOPY, transport_ap_free_cb);
}

/**
  * @brief  通过SPI向从设备发送数据
  * @param  iface_type - 接口类型
  *         iface_num - 接口编号
  *         wbuffer - 发送缓冲区
  *         wlen - 发送缓冲区大小
  *         buff_zcopy - 是否使用零拷贝
  *         free_wbuf_fun - 释放缓冲区的回调函数
  * @retval sendbuf - 发送缓冲区
  */
int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
        uint8_t * wbuffer, uint16_t wlen, uint8_t buff_zcopy, void (*free_wbuf_fun)(void* ptr))
{
    interface_buffer_handle_t buf_handle = {0};
    void (*free_func)(void* ptr) = NULL;
    uint8_t transport_up = is_transport_tx_ready();
    uint8_t pkt_prio = PRIO_Q_OTHERS;

    // 如果提供了释放缓冲区的回调函数，则赋值给free_func
    if (free_wbuf_fun)
        free_func = free_wbuf_fun;

    // 检查输入参数的有效性
    if (!wbuffer || !wlen || (wlen > MAX_TRANSPORT_BUFFER_SIZE) || !transport_up) {
        ESP_LOGE(TAG, "write fail: trans_ready[%u] buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?",
                transport_up, wbuffer, wlen, MAX_TRANSPORT_BUFFER_SIZE);
        H_FREE_PTR_WITH_FUNC(free_func, wbuffer);
        return STM_FAIL;
    }

    // 初始化缓冲区句柄
    buf_handle.payload_zcopy = buff_zcopy;
    buf_handle.if_type = iface_type;
    buf_handle.if_num = iface_num;
    buf_handle.payload_len = wlen;
    buf_handle.payload = wbuffer;
    buf_handle.priv_buffer_handle = wbuffer;
    buf_handle.free_buf_handle = free_func;

    ESP_LOGV(TAG, "ifype: %u wbuff:%p, free: %p wlen:%u ", iface_type, wbuffer, free_func, wlen);

    // 根据接口类型设置数据包优先级
    if (buf_handle.if_type == ESP_SERIAL_IF)
        pkt_prio = PRIO_Q_SERIAL;
    else if (buf_handle.if_type == ESP_HCI_IF)
        pkt_prio = PRIO_Q_BT;
    /* 其他情况默认为PRIO_Q_OTHERS */

    // 将数据包加入发送队列
    xQueueSend(to_slave_queue[pkt_prio], &buf_handle, portMAX_DELAY);
    xSemaphoreGive(sem_to_slave_queue);

    // 通知SPI传输准备就绪
    xSemaphoreGive(spi_trans_ready_sem);

    return STM_OK;
}
