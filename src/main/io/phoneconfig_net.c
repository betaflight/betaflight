/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * usb-ncm + lwip link for phone-config. the fc is 192.168.7.1 with a tiny dhcp server (the host gets
 * 192.168.7.2) and an msp-over-tcp server on 5761 that the betaflight configurator connects to.
 */

#include <string.h>

#include "platform.h"

#ifdef USE_PHONE_CONFIG

#include "drivers/time.h"
#include "io/phoneconfig.h"
#include "drivers/serial.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "pg/pg.h"

#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "netif/ethernet.h"

#include "io/phoneconfig_net.h"
#include "io/phoneflash.h"

#define NET_FC_IP        LWIP_MAKEU32(192, 168, 7, 1)
#define NET_HOST_IP      LWIP_MAKEU32(192, 168, 7, 2)
#define NET_NETMASK      LWIP_MAKEU32(255, 255, 255, 0)

static struct netif phoneNetif;
static struct udp_pcb *dhcpPcb;
static uint8_t macAddr[6];
static bool netInitDone = false;

#define NET_MSP_PORT  5761
#define MSP_RX_RING   8192U    // must exceed TCP_WND (5840) so a full receive window always fits, see mspRecv
#define MSP_TX_RING   8192U

typedef struct {
    serialPort_t port;          // must be first, cast to/from serialPort_t*
    uint8_t rxBuf[MSP_RX_RING];
    uint8_t txBuf[MSP_TX_RING];
    volatile uint16_t rxHead, rxTail;
    volatile uint16_t txHead, txTail;
} mspSerial_t;

static mspSerial_t mspSerial;
static struct tcp_pcb *mspListenPcb;
static struct tcp_pcb *mspConnPcb;        // one connection at a time

static void mspFlushTx(void);

u32_t sys_now(void)
{
    return millis();
}

static err_t phoneNetifLinkOutput(struct netif *netif, struct pbuf *p)
{
    (void)netif;
    static uint8_t txBuf[1600];
    if (p->tot_len > sizeof(txBuf)) {
        return ERR_BUF;
    }
    pbuf_copy_partial(p, txBuf, p->tot_len, 0);
    if (!phoneConfigUsbTransmitFrame(txBuf, p->tot_len)) {
        return ERR_IF;   // usb tx busy, frame dropped (tcp retransmits)
    }
    return ERR_OK;
}

static err_t phoneNetifInit(struct netif *netif)
{
    netif->name[0] = 'n';
    netif->name[1] = 'c';
    netif->output = etharp_output;
    netif->linkoutput = phoneNetifLinkOutput;
    netif->mtu = 1500;
    netif->hwaddr_len = 6;
    memcpy(netif->hwaddr, macAddr, 6);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
    return ERR_OK;
}

// called by the usb-ncm class on each frame from the host
void phoneConfigNetRxFrame(const uint8_t *frame, uint32_t length)
{
    if (!netInitDone || length == 0) {
        return;
    }
    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)length, PBUF_POOL);
    if (p == NULL) {
        return;
    }
    pbuf_take(p, frame, (u16_t)length);
    if (phoneNetif.input(p, &phoneNetif) != ERR_OK) {
        pbuf_free(p);
    }
}

void phoneConfigNetLinkChange(uint8_t up)
{
    if (!netInitDone) {
        return;
    }
    if (up) {
        netif_set_link_up(&phoneNetif);
    } else {
        netif_set_link_down(&phoneNetif);
    }
}

// minimal dhcp server
#define DHCP_BOOTREQUEST   1
#define DHCP_BOOTREPLY     2
#define DHCP_DISCOVER      1
#define DHCP_OFFER         2
#define DHCP_REQUEST       3
#define DHCP_ACK           5
#define DHCP_MAGIC_COOKIE  0x63825363U

static uint8_t dhcpFindMsgType(const uint8_t *opt, uint16_t len)
{
    uint16_t i = 0;
    while (i + 1 < len) {
        const uint8_t code = opt[i];
        if (code == 0xFF) break;     // end
        if (code == 0x00) { i++; continue; }  // pad
        const uint8_t l = opt[i + 1];
        if (code == 53 && l >= 1 && i + 2 < len) {
            return opt[i + 2];
        }
        i += 2 + l;
    }
    return 0;
}

static void dhcpRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)addr; (void)port;

    // bootp/dhcp fixed header is 240 bytes incl. the magic cookie, options follow
    if (p->tot_len < 240) {
        pbuf_free(p);
        return;
    }

    static uint8_t in[548];
    const uint16_t inLen = (p->tot_len > sizeof(in)) ? sizeof(in) : p->tot_len;
    pbuf_copy_partial(p, in, inLen, 0);
    pbuf_free(p);

    const uint8_t msgType = dhcpFindMsgType(in + 240, inLen - 240);
    if (msgType != DHCP_DISCOVER && msgType != DHCP_REQUEST) {
        return;
    }

    static uint8_t out[300];
    memset(out, 0, sizeof(out));
    out[0] = DHCP_BOOTREPLY;
    out[1] = 1;        // htype ethernet
    out[2] = 6;        // hlen
    memcpy(out + 4, in + 4, 4);     // xid
    memcpy(out + 10, in + 10, 2);   // flags
    const uint32_t y = lwip_htonl(NET_HOST_IP);
    memcpy(out + 16, &y, 4);        // yiaddr = host ip
    const uint32_t s = lwip_htonl(NET_FC_IP);
    memcpy(out + 20, &s, 4);        // siaddr = server ip
    memcpy(out + 28, in + 28, 16);  // chaddr
    uint32_t cookie = lwip_htonl(DHCP_MAGIC_COOKIE);
    memcpy(out + 236, &cookie, 4);

    uint16_t o = 240;
    out[o++] = 53; out[o++] = 1; out[o++] = (msgType == DHCP_DISCOVER) ? DHCP_OFFER : DHCP_ACK;
    out[o++] = 54; out[o++] = 4; memcpy(out + o, &s, 4); o += 4;   // server id
    const uint32_t mask = lwip_htonl(NET_NETMASK);
    out[o++] = 1;  out[o++] = 4; memcpy(out + o, &mask, 4); o += 4; // subnet mask
    out[o++] = 3;  out[o++] = 4; memcpy(out + o, &s, 4); o += 4;    // router
    out[o++] = 51; out[o++] = 4;
    out[o++] = 0x00; out[o++] = 0x01; out[o++] = 0x51; out[o++] = 0x80; // lease 86400s
    out[o++] = 0xFF;  // end

    struct pbuf *resp = pbuf_alloc(PBUF_TRANSPORT, o, PBUF_RAM);
    if (resp == NULL) {
        return;
    }
    memcpy(resp->payload, out, o);
    ip_addr_t bcast;
    IP_ADDR4(&bcast, 255, 255, 255, 255);
    udp_sendto(pcb, resp, &bcast, 68);
    pbuf_free(resp);
}

// virtual serial port backed by the rx/tx rings
static void mspPumpTx(void)
{
    phoneConfigUsbLock();
    phoneConfigUsbProcess();
    sys_check_timeouts();
    mspFlushTx();
    phoneConfigUsbUnlock();
}

static void mspVWrite(serialPort_t *p, uint8_t ch)
{
    mspSerial_t *s = (mspSerial_t *)p;

    uint16_t next = (uint16_t)((s->txHead + 1) & (MSP_TX_RING - 1));
    while (next == s->txTail) {
        mspPumpTx();
        next = (uint16_t)((s->txHead + 1) & (MSP_TX_RING - 1));
    }

    s->txBuf[s->txHead] = ch;
    s->txHead = next;
}

static void mspVWriteBuf(serialPort_t *p, const void *data, int count)
{
    const uint8_t *bytes = data;

    while (count-- > 0) {
        mspVWrite(p, *bytes++);
    }
}

static uint32_t mspVRxWaiting(const serialPort_t *p)
{
    const mspSerial_t *s = (const mspSerial_t *)p;
    return (uint32_t)((uint16_t)(s->rxHead - s->rxTail) & (MSP_RX_RING - 1));
}

static uint32_t mspVTxFree(const serialPort_t *p)
{
    const mspSerial_t *s = (const mspSerial_t *)p;
    const uint16_t used = (uint16_t)(s->txHead - s->txTail) & (MSP_TX_RING - 1);
    return (uint32_t)(MSP_TX_RING - 1 - used);
}

static uint8_t mspVRead(serialPort_t *p)
{
    mspSerial_t *s = (mspSerial_t *)p;
    const uint8_t c = s->rxBuf[s->rxTail];
    s->rxTail = (uint16_t)((s->rxTail + 1) & (MSP_RX_RING - 1));
    return c;
}

static bool mspVTxEmpty(const serialPort_t *p)
{
    const mspSerial_t *s = (const mspSerial_t *)p;
    return s->txHead == s->txTail;
}

static const struct serialPortVTable mspVTable = {
    .serialWrite                 = mspVWrite,
    .serialTotalRxWaiting        = mspVRxWaiting,
    .serialTotalTxFree           = mspVTxFree,
    .serialRead                  = mspVRead,
    .serialSetBaudRate           = NULL,
    .isSerialTransmitBufferEmpty = mspVTxEmpty,
    .setMode                     = NULL,
    .setCtrlLineStateCb          = NULL,
    .setBaudRateCb               = NULL,
    .writeBuf                    = mspVWriteBuf,
    .beginWrite                  = NULL,
    .endWrite                    = NULL,
};

// one tcp segment per call, only when the usb in endpoint is idle: the ncm tx is single-buffered
static void mspFlushTx(void)
{
    if (!mspConnPcb) {
        mspSerial.txTail = mspSerial.txHead;   // no client, discard
        return;
    }
    if (mspSerial.txTail == mspSerial.txHead) {
        return;
    }
    if (phoneConfigUsbTxBusy()) {
        return;
    }
    const uint16_t snd = tcp_sndbuf(mspConnPcb);
    if (snd == 0) {
        tcp_output(mspConnPcb);   // window full, push pending ack
        return;
    }
    const uint16_t head = mspSerial.txHead;
    uint16_t contig = (head > mspSerial.txTail)
                    ? (uint16_t)(head - mspSerial.txTail)
                    : (uint16_t)(MSP_TX_RING - mspSerial.txTail);
    if (contig > snd)     { contig = snd; }
    if (contig > TCP_MSS) { contig = TCP_MSS; }   // one segment per usb frame
    if (tcp_write(mspConnPcb, &mspSerial.txBuf[mspSerial.txTail], contig, TCP_WRITE_FLAG_COPY) != ERR_OK) {
        return;
    }
    mspSerial.txTail = (uint16_t)((mspSerial.txTail + contig) & (MSP_TX_RING - 1));
    tcp_output(mspConnPcb);
}

static void mspConnErr(void *arg, err_t err)
{
    (void)arg; (void)err;
    mspConnPcb = NULL;   // pcb already freed by lwip
}

static err_t mspRecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    (void)arg;
    if (err != ERR_OK || p == NULL) {
        if (mspConnPcb == pcb) {
            mspConnPcb = NULL;
        }
        tcp_close(pcb);
        return ERR_OK;
    }
    // accept the segment only if all of it fits the ring, else refuse it whole and let lwip redeliver
    // once the msp task drains room. acking bytes we couldn't store (the old behaviour) desynced the
    // stream with no retransmit. the ring is sized >= TCP_WND so a full window can never deadlock here.
    const uint16_t used = (uint16_t)(mspSerial.rxHead - mspSerial.rxTail) & (MSP_RX_RING - 1);
    if (p->tot_len > (uint16_t)(MSP_RX_RING - 1 - used)) {
        return ERR_MEM;
    }
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        const uint8_t *d = (const uint8_t *)q->payload;
        for (uint16_t i = 0; i < q->len; i++) {
            mspSerial.rxBuf[mspSerial.rxHead] = d[i];
            mspSerial.rxHead = (uint16_t)((mspSerial.rxHead + 1) & (MSP_RX_RING - 1));
        }
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t mspAccept(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg;
    if (err != ERR_OK || pcb == NULL) {
        return ERR_VAL;
    }
    if (mspConnPcb) {
        tcp_abort(mspConnPcb);
        mspConnPcb = NULL;
    }
    mspConnPcb = pcb;
    mspSerial.rxHead = mspSerial.rxTail = 0;
    mspSerial.txHead = mspSerial.txTail = 0;
    tcp_recv(pcb, mspRecv);
    tcp_err(pcb, mspConnErr);
    return ERR_OK;
}

// boot already ran mspInit(), just register the tcp-backed port
static void mspBridgeInit(void)
{
    memset(&mspSerial, 0, sizeof(mspSerial));
    mspSerial.port.vTable       = &mspVTable;
    mspSerial.port.mode         = MODE_RXTX;
    mspSerial.port.baudRate     = 115200;
    // usb-vcp id (the real vcp is suppressed) so it differs from the msp displayport's none-sentinel,
    // otherwise mspSerialProcess treats this as the vtx port and never evaluates '#' to enter the cli
    mspSerial.port.identifier   = SERIAL_PORT_USB_VCP;
    mspSerial.port.rxBuffer     = mspSerial.rxBuf;
    mspSerial.port.txBuffer     = mspSerial.txBuf;
    mspSerial.port.rxBufferSize = MSP_RX_RING;
    mspSerial.port.txBufferSize = MSP_TX_RING;

    mspSerialRegisterPort(&mspSerial.port);
}

static struct udp_pcb *flashPcb;

// a PF_HELLO datagram means the host wants to reflash; note it, the phone-config task hands off
static void flashRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)pcb;
    (void)addr;
    (void)port;

    if (p) {
        uint8_t buf[16];
        uint16_t n = pbuf_copy_partial(p, buf, sizeof(buf), 0);
        phoneFlashNoteHello(buf, n);
        pbuf_free(p);
    }
}

void phoneConfigNetInit(const uint8_t *mac)
{
    memcpy(macAddr, mac, 6);

    lwip_init();

    ip4_addr_t ip, mask, gw;
    ip4_addr_set_u32(&ip, lwip_htonl(NET_FC_IP));
    ip4_addr_set_u32(&mask, lwip_htonl(NET_NETMASK));
    ip4_addr_set_u32(&gw, lwip_htonl(NET_FC_IP));

    netif_add(&phoneNetif, &ip, &mask, &gw, NULL, phoneNetifInit, ethernet_input);
    netif_set_default(&phoneNetif);
    netif_set_up(&phoneNetif);
    netif_set_link_up(&phoneNetif);

    dhcpPcb = udp_new();
    if (dhcpPcb) {
        udp_bind(dhcpPcb, IP_ADDR_ANY, 67);
        udp_recv(dhcpPcb, dhcpRecv, NULL);
    }

    mspListenPcb = tcp_new();
    if (mspListenPcb) {
        tcp_bind(mspListenPcb, IP_ADDR_ANY, NET_MSP_PORT);
        mspListenPcb = tcp_listen(mspListenPcb);
        tcp_accept(mspListenPcb, mspAccept);
    }

    flashPcb = udp_new();
    if (flashPcb) {
        udp_bind(flashPcb, IP_ADDR_ANY, PHONE_FLASH_UDP_PORT);
        udp_recv(flashPcb, flashRecv, NULL);
    }

    mspBridgeInit();

    netInitDone = true;
}

void phoneConfigNetPoll(void)
{
    if (netInitDone) {
        sys_check_timeouts();   // lwip timers
        mspFlushTx();
    }
}

#endif // USE_PHONE_CONFIG
