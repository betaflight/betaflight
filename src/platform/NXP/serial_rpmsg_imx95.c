/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * VCP of the i.MX95 M7: a tty to Linux (/dev/ttyBF) over rpmsg in the uart_rpmsg protocol.
 * Each WRITE from Linux is answered with the count taken; Linux holds the rest until a WAKEUP.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RPMSG

#include <openamp/rpmsg.h>

#include "common/maths.h"
#include "common/utils.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/time.h"
#include "io/serial.h"

#include "rpmsg_imx95.h"

#define RPMSG_TTY_NAME          "rpmsg-ttyBF"
#define RPMSG_TTY_WRITE         0
#define RPMSG_TTY_WAKEUP        1
#define RPMSG_TTY_RESPONSE      (1u << 31)
#define RPMSG_TTY_BUFFER_SIZE   2048
#define RPMSG_TTY_STALL_MS      50
// Data in one WRITE: Linux's 512 B rpmsg buffer less the rpmsg and tty headers.
#define RPMSG_TTY_DATA_MAX      472

typedef struct __attribute__((packed)) {
    uint32_t command;           // RPMSG_TTY_RESPONSE marks an answer
    int32_t result;
    uint64_t cookie;
} rpmsgTtyHeader_t;

typedef struct __attribute__((packed)) {
    rpmsgTtyHeader_t header;
    uint32_t count;
    uint32_t resolved;
    uint8_t data[];
} rpmsgTtyWrite_t;

typedef struct {
    serialPort_t port;
    struct rpmsg_endpoint ept;
    bool buffering;
    bool replyOwed;
    bool wakeupOwed;
    timeMs_t sentMs;            // last send, or when output found the ring empty
    timeMs_t nextPollMs;
    rpmsgTtyWrite_t reply;
} rpmsgTtyPort_t;

static rpmsgTtyPort_t rpmsgTty;
static uint8_t rpmsgTtyRxBuffer[RPMSG_TTY_BUFFER_SIZE];
static uint8_t rpmsgTtyTxBuffer[RPMSG_TTY_BUFFER_SIZE];

static uint32_t rpmsgTtyRingUsed(uint32_t head, uint32_t tail, uint32_t size)
{
    return (head >= tail) ? head - tail : size + head - tail;
}

static uint32_t rpmsgTtyRxFree(const rpmsgTtyPort_t *tty)
{
    return tty->port.rxBufferSize - 1
           - rpmsgTtyRingUsed(tty->port.rxBufferHead, tty->port.rxBufferTail, tty->port.rxBufferSize);
}

static uint32_t rpmsgTtyTxUsed(const rpmsgTtyPort_t *tty)
{
    return rpmsgTtyRingUsed(tty->port.txBufferHead, tty->port.txBufferTail, tty->port.txBufferSize);
}

static void rpmsgTtySendReply(rpmsgTtyPort_t *tty)
{
    if (tty->replyOwed && rpmsg_trysend(&tty->ept, &tty->reply, sizeof(tty->reply)) >= 0) {
        tty->replyOwed = false;
    }
}

static int rpmsgTtyCallback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
    UNUSED(src);
    UNUSED(priv);
    rpmsgTtyPort_t *tty = container_of(ept, rpmsgTtyPort_t, ept);
    const rpmsgTtyWrite_t *msg = data;

    if (len < sizeof(rpmsgTtyWrite_t) || msg->header.command != RPMSG_TTY_WRITE) {
        return 0;
    }

    const uint32_t count = MIN(msg->count, (uint32_t)(len - sizeof(rpmsgTtyWrite_t)));
    const uint32_t taken = MIN(count, rpmsgTtyRxFree(tty));
    const uint32_t head = tty->port.rxBufferHead;
    const uint32_t first = MIN(taken, tty->port.rxBufferSize - head);
    rpmsgCopy(&rpmsgTtyRxBuffer[head], msg->data, first);
    rpmsgCopy(rpmsgTtyRxBuffer, msg->data + first, taken - first);
    tty->port.rxBufferHead = (head + taken) % tty->port.rxBufferSize;
    if (taken < msg->count) {
        tty->wakeupOwed = true;
    }

    // Linux waits for this answer before it writes again, so it is kept until sent.
    tty->reply = (rpmsgTtyWrite_t){
        .header = {
            .command = RPMSG_TTY_WRITE | RPMSG_TTY_RESPONSE,
            .result = (int32_t)taken,
            .cookie = msg->header.cookie,
        },
    };
    tty->replyOwed = true;
    rpmsgTtySendReply(tty);
    return 0;
}

static void rpmsgTtyDropTx(rpmsgTtyPort_t *tty)
{
    tty->port.txBufferTail = tty->port.txBufferHead;
}

// Sends while the ring holds at least sendMin bytes, so loose writes go out as full messages.
// No buffer for RPMSG_TTY_STALL_MS means Linux has stopped; output is dropped as on a dead USB host.
static void rpmsgTtyFlush(rpmsgTtyPort_t *tty, uint32_t sendMin)
{
    while (rpmsgTtyTxUsed(tty) >= sendMin && is_rpmsg_ept_ready(&tty->ept)) {
        uint32_t size;
        rpmsgTtyWrite_t *msg = rpmsg_get_tx_payload_buffer(&tty->ept, &size, 0);
        if (!msg) {
            if (cmpTimeMs(millis(), tty->sentMs) >= RPMSG_TTY_STALL_MS) {
                rpmsgTtyDropTx(tty);
            }
            return;
        }
        const uint32_t count = MIN(rpmsgTtyTxUsed(tty), size - (uint32_t)sizeof(rpmsgTtyWrite_t));
        const uint32_t tail = tty->port.txBufferTail;
        const uint32_t first = MIN(count, tty->port.txBufferSize - tail);
        rpmsgCopy(msg->data, &rpmsgTtyTxBuffer[tail], first);
        rpmsgCopy(msg->data + first, rpmsgTtyTxBuffer, count - first);
        msg->header = (rpmsgTtyHeader_t){ .command = RPMSG_TTY_WRITE };
        msg->count = count;
        msg->resolved = 0;
        if (rpmsg_send_nocopy(&tty->ept, msg, sizeof(rpmsgTtyWrite_t) + count) < 0) {
            rpmsg_release_tx_buffer(&tty->ept, msg);
            return;
        }
        tty->port.txBufferTail = (tail + count) % tty->port.txBufferSize;
        tty->sentMs = millis();
    }
}

// Callers poll byte by byte in tight loops, so the link runs at most once a millisecond.
static void rpmsgTtyPoll(void)
{
    const timeMs_t now = millis();

    if (cmpTimeMs(now, rpmsgTty.nextPollMs) >= 0) {
        rpmsgTty.nextPollMs = now + 1;
        rpmsgPoll();
    }
}

static void rpmsgTtyService(struct rpmsg_device *rdev)
{
    rpmsgTtyPort_t *tty = &rpmsgTty;

    // An answer carries a pointer into the Linux writer's stack, so it must not reach a later link.
    if (!rdev) {
        tty->replyOwed = false;
        tty->wakeupOwed = false;
        rpmsgTtyDropTx(tty);
        return;
    }
    if (!tty->ept.rdev) {
        rpmsg_create_ept(&tty->ept, rdev, RPMSG_TTY_NAME, RPMSG_TTY_ADDR, RPMSG_ADDR_ANY, rpmsgTtyCallback, NULL);
        return;
    }

    rpmsgTtySendReply(tty);
    if (tty->wakeupOwed && !tty->replyOwed && rpmsgTtyRxFree(tty) >= tty->port.rxBufferSize / 2) {
        const rpmsgTtyHeader_t wakeup = { .command = RPMSG_TTY_WAKEUP };
        if (rpmsg_trysend(&tty->ept, &wakeup, sizeof(wakeup)) >= 0) {
            tty->wakeupOwed = false;
        }
    }
    if (!tty->buffering) {
        rpmsgTtyFlush(tty, 1);
    }
}

// Output is dropped until Linux has written to the tty and its address is known.
static void rpmsgTtyWrite(serialPort_t *instance, uint8_t ch)
{
    rpmsgTtyPort_t *tty = container_of(instance, rpmsgTtyPort_t, port);
    const uint32_t next = (instance->txBufferHead + 1) % instance->txBufferSize;

    if (!is_rpmsg_ept_ready(&tty->ept)) {
        return;
    }
    if (instance->txBufferHead == instance->txBufferTail) {
        tty->sentMs = millis();
    }
    if (next != instance->txBufferTail) {
        instance->txBuffer[instance->txBufferHead] = ch;
        instance->txBufferHead = next;
    }
    if (!tty->buffering) {
        rpmsgTtyFlush(tty, RPMSG_TTY_DATA_MAX);
    }
}

static void rpmsgTtyWriteBuf(serialPort_t *instance, const void *data, int count)
{
    rpmsgTtyPort_t *tty = container_of(instance, rpmsgTtyPort_t, port);
    const bool buffering = tty->buffering;

    tty->buffering = true;
    for (const uint8_t *p = data; count > 0; count--) {
        rpmsgTtyWrite(instance, *p++);
    }
    tty->buffering = buffering;
    if (!buffering) {
        rpmsgTtyFlush(tty, RPMSG_TTY_DATA_MAX);
    }
}

static uint32_t rpmsgTtyRxWaiting(const serialPort_t *instance)
{
    rpmsgTtyPoll();
    return rpmsgTtyRingUsed(instance->rxBufferHead, instance->rxBufferTail, instance->rxBufferSize);
}

// Callers wait in loops on these, so they run the link.
static uint32_t rpmsgTtyTxFree(const serialPort_t *instance)
{
    rpmsgTtyPoll();
    rpmsgTtyFlush(&rpmsgTty, RPMSG_TTY_DATA_MAX);
    return instance->txBufferSize - 1
           - rpmsgTtyRingUsed(instance->txBufferHead, instance->txBufferTail, instance->txBufferSize);
}

static uint8_t rpmsgTtyRead(serialPort_t *instance)
{
    const uint8_t ch = instance->rxBuffer[instance->rxBufferTail];
    instance->rxBufferTail = (instance->rxBufferTail + 1) % instance->rxBufferSize;
    return ch;
}

static bool rpmsgTtyTxEmpty(const serialPort_t *instance)
{
    rpmsgTtyPoll();
    rpmsgTtyFlush(&rpmsgTty, 1);
    return instance->txBufferHead == instance->txBufferTail;
}

static void rpmsgTtyBeginWrite(serialPort_t *instance)
{
    container_of(instance, rpmsgTtyPort_t, port)->buffering = true;
}

static void rpmsgTtyEndWrite(serialPort_t *instance)
{
    rpmsgTtyPort_t *tty = container_of(instance, rpmsgTtyPort_t, port);
    tty->buffering = false;
    rpmsgTtyFlush(tty, 1);
}

static void rpmsgTtySetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
}

static void rpmsgTtySetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
}

static void rpmsgTtySetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState),
                                       void *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
}

static void rpmsgTtySetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud),
                                  serialPort_t *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
}

static const struct serialPortVTable rpmsgTtyVTable = {
    .serialWrite = rpmsgTtyWrite,
    .serialTotalRxWaiting = rpmsgTtyRxWaiting,
    .serialTotalTxFree = rpmsgTtyTxFree,
    .serialRead = rpmsgTtyRead,
    .serialSetBaudRate = rpmsgTtySetBaudRate,
    .isSerialTransmitBufferEmpty = rpmsgTtyTxEmpty,
    .setMode = rpmsgTtySetMode,
    .setCtrlLineStateCb = rpmsgTtySetCtrlLineStateCb,
    .setBaudRateCb = rpmsgTtySetBaudRateCb,
    .writeBuf = rpmsgTtyWriteBuf,
    .beginWrite = rpmsgTtyBeginWrite,
    .endWrite = rpmsgTtyEndWrite,
};

// Called before the clocks and MPU are set up, so the link starts in usbVcpOpen().
void usbVcpInit(void)
{
}

serialPort_t *usbVcpOpen(void)
{
    rpmsgTtyPort_t *tty = &rpmsgTty;

    tty->port.vTable = &rpmsgTtyVTable;
    tty->port.mode = MODE_RXTX;
    tty->port.baudRate = 115200;
    tty->port.rxBuffer = rpmsgTtyRxBuffer;
    tty->port.rxBufferSize = sizeof(rpmsgTtyRxBuffer);
    tty->port.txBuffer = rpmsgTtyTxBuffer;
    tty->port.txBufferSize = sizeof(rpmsgTtyTxBuffer);
    tty->port.identifier = SERIAL_PORT_USB_VCP;

    rpmsgInit();
    rpmsgAddService(rpmsgTtyService);
    return &tty->port;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    return instance->baudRate;
}

uint8_t usbVcpIsConnected(void)
{
    return is_rpmsg_ept_ready(&rpmsgTty.ept);
}

uint8_t usbVcpIsActive(void)
{
    return is_rpmsg_ept_ready(&rpmsgTty.ept);
}

#endif // USE_RPMSG
