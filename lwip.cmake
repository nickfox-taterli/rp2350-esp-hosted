set(LWIP_DIR "Middlewares/LwIP")
set(LWIP_INCS "Middlewares/LwIP/src/include")

set(LWIP_SRCS
        ${LWIP_DIR}/src/core/init.c
        ${LWIP_DIR}/src/core/def.c
        ${LWIP_DIR}/src/core/dns.c
        ${LWIP_DIR}/src/core/inet_chksum.c
        ${LWIP_DIR}/src/core/ip.c
        ${LWIP_DIR}/src/core/mem.c
        ${LWIP_DIR}/src/core/memp.c
        ${LWIP_DIR}/src/core/netif.c
        ${LWIP_DIR}/src/core/pbuf.c
        ${LWIP_DIR}/src/core/raw.c
        ${LWIP_DIR}/src/core/stats.c
        ${LWIP_DIR}/src/core/sys.c
        ${LWIP_DIR}/src/core/altcp.c
        ${LWIP_DIR}/src/core/altcp_alloc.c
        ${LWIP_DIR}/src/core/altcp_tcp.c
        ${LWIP_DIR}/src/core/tcp.c
        ${LWIP_DIR}/src/core/tcp_in.c
        ${LWIP_DIR}/src/core/tcp_out.c
        ${LWIP_DIR}/src/core/timeouts.c
        ${LWIP_DIR}/src/core/udp.c

        ${LWIP_DIR}/src/core/ipv4/acd.c
        ${LWIP_DIR}/src/core/ipv4/autoip.c
        ${LWIP_DIR}/src/core/ipv4/dhcp.c
        ${LWIP_DIR}/src/core/ipv4/etharp.c
        ${LWIP_DIR}/src/core/ipv4/icmp.c
        ${LWIP_DIR}/src/core/ipv4/igmp.c
        ${LWIP_DIR}/src/core/ipv4/ip4_frag.c
        ${LWIP_DIR}/src/core/ipv4/ip4.c
        ${LWIP_DIR}/src/core/ipv4/ip4_addr.c

        ${LWIP_DIR}/src/core/ipv6/dhcp6.c
        ${LWIP_DIR}/src/core/ipv6/ethip6.c
        ${LWIP_DIR}/src/core/ipv6/icmp6.c
        ${LWIP_DIR}/src/core/ipv6/inet6.c
        ${LWIP_DIR}/src/core/ipv6/ip6.c
        ${LWIP_DIR}/src/core/ipv6/ip6_addr.c
        ${LWIP_DIR}/src/core/ipv6/ip6_frag.c
        ${LWIP_DIR}/src/core/ipv6/mld6.c
        ${LWIP_DIR}/src/core/ipv6/nd6.c

        ${LWIP_DIR}/src/api/api_lib.c
        ${LWIP_DIR}/src/api/api_msg.c
        ${LWIP_DIR}/src/api/err.c
        ${LWIP_DIR}/src/api/if_api.c
        ${LWIP_DIR}/src/api/netbuf.c
        ${LWIP_DIR}/src/api/netdb.c
        ${LWIP_DIR}/src/api/netifapi.c
        ${LWIP_DIR}/src/api/sockets.c
        ${LWIP_DIR}/src/api/tcpip.c

        ${LWIP_DIR}/src/netif/ethernet.c
        ${LWIP_DIR}/src/netif/bridgeif.c
        ${LWIP_DIR}/src/netif/bridgeif_fdb.c

        ${LWIP_DIR}/src/apps/lwiperf/lwiperf.c
)


set(LWIP_IF_INCS "Driver/ethernetif/Inc")

file(GLOB_RECURSE LWIP_IF_SRCS
        "Driver/ethernetif/*.*"
)

