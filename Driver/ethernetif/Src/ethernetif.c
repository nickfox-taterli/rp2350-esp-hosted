#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/tcpip.h"
#include "ethernetif.h"

#include <string.h>

/* Define those to better describe your network interface. */

#include "lwip/opt.h"
#include "lwip/def.h"
#include "arch/sys_arch.h"

#include "esp_hosted_api.h"

#define IFNAME0 'e'
#define IFNAME1 'n'

#ifdef TIME_STAMPING
#define DEFAULT_ADDNED    0x1E70C600
#define DEFAULT_INC    0xD7
#endif

struct netif *_netif;

static uint8_t txbuf[1600];
//static uint8_t txbuf[350] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x48, 0x31, 0xb7, 0x3e, 0x55, 0x40, 0x8, 0x0, 0x45, 0x0, 0x1, 0x50, 0x0, 0x0, 0x0, 0x0, 0x40, 0x11, 0x79, 0x9e, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0x0, 0x44, 0x0, 0x43, 0x1, 0x3c, 0x21, 0x66, 0x1, 0x1, 0x6, 0x0, 0x68, 0xb8, 0x3c, 0x71, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x48, 0x31, 0xb7, 0x3e, 0x55, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x63, 0x82, 0x53, 0x63, 0x35, 0x1, 0x1, 0x39, 0x2, 0x5, 0xdc, 0xc, 0x9, 0x65, 0x73, 0x70, 0x72, 0x65, 0x73, 0x73, 0x69, 0x66, 0x37, 0x4, 0x1, 0x3, 0x1c, 0x6, 0x3d, 0x7, 0x1, 0x48, 0x31, 0xb7, 0x3e, 0x55, 0x40, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// 数据接受的函数

static void low_level_init(struct netif *netif) {
    /* 设置MAC硬件地址长度 */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* 设置MAC硬件地址 */
    netif->hwaddr[0] = ETH_MAC_ADDR0;
    netif->hwaddr[1] = ETH_MAC_ADDR1;
    netif->hwaddr[2] = ETH_MAC_ADDR2;
    netif->hwaddr[3] = ETH_MAC_ADDR3;
    netif->hwaddr[4] = ETH_MAC_ADDR4;
    netif->hwaddr[5] = ETH_MAC_ADDR5;

    /* 设置最大传输单元（MTU） */
    netif->mtu = 1500;

    /* 设置设备能力标志 */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
#ifdef LWIP_IGMP
    netif->flags |= NETIF_FLAG_IGMP;
#endif
}

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    uint16_t len = 0;

#if ETH_PAD_SIZE
    // 如果定义了填充大小，移除填充字节
    pbuf_header(p, -ETH_PAD_SIZE);
#endif

    // 遍历 pbuf 链表，将数据复制到发送缓冲区,因为LWIP在这个函数后就释放掉pbuf了,所以要复制出来.
    for (q = p; q != NULL; q = q->next) {
        memcpy((uint8_t *)&txbuf[len], q->payload, q->len);
        if ((len + q->len) > 1600)
            return ERR_MEM;
        len += q->len;
    }

    // 内部会复制缓冲区,并不担心冲刷
    esp_wifi_internal_tx(WIFI_IF_STA,txbuf,len);
    // esp_wifi_internal_tx(WIFI_IF_STA,txbuf,350);

#if ETH_PAD_SIZE
    // 如果定义了填充大小，恢复填充字节
    pbuf_header(p, ETH_PAD_SIZE);
#endif

    // 更新链路统计信息
    LINK_STATS_INC(link.xmit);

    return ERR_OK;  // 返回成功
}

static struct pbuf * low_level_input(struct netif *netif, uint16_t len, uint8_t *buf)
{
    struct pbuf *p, *q;

#if ETH_PAD_SIZE
    len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

    /* We allocate a pbuf chain of pbufs from the pool. */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (p != NULL)
    {

#if ETH_PAD_SIZE
        pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

        len = 0;
        /* We iterate over the pbuf chain until we have read the entire
        * packet into the pbuf. */
        for(q = p; q != NULL; q = q->next)
        {
            memcpy((uint8_t*)q->payload, (uint8_t*)&buf[len], q->len);
            len = len + q->len;
        }


#if ETH_PAD_SIZE
        pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

        LINK_STATS_INC(link.recv);
    }
    else
    {
        // do nothing. drop the packet
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
    }

    return p;
}

#include <string.h>  // 用于 memcmp 函数
#include <stdint.h> // 用于 uint8_t 类型

static esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
    // 定义目标字节序列
    const uint8_t target_bytes[12] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x70, 0xa8, 0xd3, 0xfc, 0x5d, 0x82};

    // 检查 buffer 的前 12 个字节是否匹配目标字节序列 (验证Only)
    if (len >= 12 && memcmp(buffer, target_bytes, 12) == 0)
    {
        // 触发 ARM 断点
        // __asm__ volatile ("bkpt #0");
    }

    struct pbuf* p = NULL;
    p = low_level_input(_netif, len, buffer);
    if (p != NULL)
    {
        if (_netif->input(p, _netif) != ERR_OK)
        {
            pbuf_free(p);
        }
    }

    return ESP_OK;  // 假设函数返回 ESP_OK
}
/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
    struct ethernetif *ethernetif;

    LWIP_ASSERT("netif != NULL", (netif != NULL));

    _netif = netif;

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "pico";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
     * Initialize the snmp variables and counters inside the struct netif.
     * The last argument should be replaced with your link speed, in units
     * of bits per second.
     */
    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}
