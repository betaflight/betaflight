/*
 * lwip options for betaflight phone-config mode (usb cdc-ncm network gadget). no_sys (bare-metal, no
 * rtos), ipv4 only, tcp and udp, with the fc acting as a tiny dhcp server.
 */

#pragma once

#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0

#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        0
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define LWIP_DHCP                       0   /* the fc is the dhcp server, not a client */
#define LWIP_DNS                        0
#define LWIP_AUTOIP                     0

/* accept the dhcp discover (udp:67) that arrives from 0.0.0.0 before the client has an address, else
 * lwip drops it. PP_NTOHS is expanded later in ip4.c, where it is defined. */
#define LWIP_IP_ACCEPT_UDP_PORT(port)   ((port) == PP_NTOHS(67))

#define LWIP_SINGLE_NETIF               1
#define LWIP_NETIF_TX_SINGLE_PBUF       1
#define LWIP_NETIF_STATUS_CALLBACK      0
#define LWIP_NETIF_LINK_CALLBACK        0
#define LWIP_NETIF_HOSTNAME             0

#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (10 * 1024)
#define MEMP_NUM_PBUF                   16
#define MEMP_NUM_UDP_PCB                4
#define MEMP_NUM_TCP_PCB                4
#define MEMP_NUM_TCP_PCB_LISTEN         2
#define MEMP_NUM_TCP_SEG                16
#define PBUF_POOL_SIZE                  16
#define PBUF_POOL_BUFSIZE               1536

#define TCP_MSS                         1460
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * TCP_SND_BUF) / TCP_MSS)
#define TCP_WND                         (4 * TCP_MSS)

#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0

/* compute all checksums in software, the otg-fs has no checksum offload */
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1

#define LWIP_NETCONN_FULLDUPLEX         0
#define LWIP_PROVIDE_ERRNO              1
