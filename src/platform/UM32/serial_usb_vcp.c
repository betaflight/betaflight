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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_VCP

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "pg/usb.h"


#include "drivers/usb_io.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "usb_dc.h"
#include "vcp_hal/cdc_acm.h"

// USBD_HandleTypeDef USBD_Device;

#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"

#define USB_TX_TIMEOUT_MS     200
#define USB_FLUSH_TIMEOUT_MS   50
typedef struct {
    uint8_t buffer[USB_VCP_RX_RING_SIZE];
    uint32_t head;
    uint32_t tail;
} CircularBuffer;

typedef struct {
    uint32_t session;
    uint32_t startedAt;
    bool active;
    bool expired;
} vcpTxStall_t;

static CircularBuffer cdc_buf;

uint8_t usbVcpIsActive(void)
{
    return usbIsConnected() && usbIsConfigured();
}

static vcpPort_t vcpPort = {0};
static uint32_t vcpWriteSession;
static bool vcpWriteAborted;
static vcpTxStall_t vcpTxStall;
static uint32_t g_baudRate = 0;


static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
    g_baudRate = baudRate;
    // TODO implement
}

static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
}

static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return cdc_acm_tx_empty();
}

bool usbVcpReceiveData(const uint8_t *data, uint32_t len)
{
    if (len == 0) {
        return true;
    }

    const uint32_t head = __atomic_load_n(&cdc_buf.head, __ATOMIC_ACQUIRE);
    const uint32_t tail = __atomic_load_n(&cdc_buf.tail, __ATOMIC_RELAXED);
    // Reserve one slot to distinguish a full ring from an empty one.
    const uint32_t freeSpace = (head + USB_VCP_RX_RING_SIZE - tail - 1U) % USB_VCP_RX_RING_SIZE;
    if (len > freeSpace) {
        return false;
    }

    const uint32_t first = MIN(len, USB_VCP_RX_RING_SIZE - tail);
    memcpy(&cdc_buf.buffer[tail], data, first);
    if (len > first) {
        memcpy(cdc_buf.buffer, data + first, len - first);
    }
    __atomic_store_n(&cdc_buf.tail, (tail + len) % USB_VCP_RX_RING_SIZE, __ATOMIC_RELEASE);
    return true;
}

void usbVcpResetRx(void)
{
    // The session reset masks USB, as does the reader while advancing head.
    const uint32_t tail = __atomic_load_n(&cdc_buf.tail, __ATOMIC_RELAXED);
    __atomic_store_n(&cdc_buf.head, tail, __ATOMIC_RELEASE);
}

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

    uint32_t available = 0;
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        const uint32_t head = __atomic_load_n(&cdc_buf.head, __ATOMIC_RELAXED);
        const uint32_t tail = __atomic_load_n(&cdc_buf.tail, __ATOMIC_ACQUIRE);
        available = (tail + USB_VCP_RX_RING_SIZE - head) % USB_VCP_RX_RING_SIZE;
    }
    return available;
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    uint8_t data = 0;
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        const uint32_t head = __atomic_load_n(&cdc_buf.head, __ATOMIC_RELAXED);
        if (head == __atomic_load_n(&cdc_buf.tail, __ATOMIC_ACQUIRE)) {
            return 0;
        }

        data = cdc_buf.buffer[head];
        __atomic_store_n(&cdc_buf.head, (head + 1U) % USB_VCP_RX_RING_SIZE, __ATOMIC_RELEASE);
    }
    cdc_acm_resume_rx();
    return data;
}

static void usbVcpFlush(uint32_t session)
{
    cdc_acm_flush_tx(session);
    const uint32_t start = millis();
    while (cdc_acm_tx_session() == session && !cdc_acm_tx_empty() && millis() - start < USB_FLUSH_TIMEOUT_MS) {
    }
}

static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    if (count <= 0) {
        return;
    }

    const vcpPort_t *port = container_of(instance, vcpPort_t, port);
    if (port->buffering && vcpWriteAborted) {
        return;
    }

    const uint32_t session = port->buffering ? vcpWriteSession : cdc_acm_tx_session();
    const uint8_t *bytes = data;
    uint32_t remaining = count;

    while (remaining && cdc_acm_tx_session() == session && usbIsConfigured()) {
        // Non-buffered writes are published to the IN endpoint by queue_tx.
        const uint32_t copied = cdc_acm_queue_tx(session, bytes, remaining, port->buffering);
        bytes += copied;
        remaining -= copied;
        if (copied) {
            vcpTxStall.active = false;
        } else if (!vcpTxStall.active || vcpTxStall.session != session) {
            // One timeout for a continuous stall, even across write calls.
            vcpTxStall = (vcpTxStall_t) { .session = session, .startedAt = millis(), .active = true };
        } else if (vcpTxStall.expired || millis() - vcpTxStall.startedAt >= USB_TX_TIMEOUT_MS) {
            vcpTxStall.expired = true;
            // Once a byte cannot be queued, suppress the rest of that batch.
            vcpWriteAborted = port->buffering;
            break;
        }
    }
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    usbVcpWriteBuf(instance, &c, 1);
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    if (!port->buffering) {
        // A response in progress must not continue into a newly configured link.
        vcpWriteSession = cdc_acm_tx_session();
        vcpWriteAborted = false;
    }
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return cdc_acm_tx_free();
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    const uint32_t session = port->buffering ? vcpWriteSession : cdc_acm_tx_session();
    port->buffering = false;
    usbVcpFlush(session);
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,
        .serialRead = usbVcpRead,
        .serialSetBaudRate = usbVcpSetBaudRate,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .setMode = usbVcpSetMode,
        .setCtrlLineStateCb = usbVcpSetCtrlLineStateCb,
        .setBaudRateCb = usbVcpSetBaudRateCb,
        .writeBuf = usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite
    }
};

void usbVcpInit(void)
{

}


serialPort_t *usbVcpOpen(void)
{
	vcpPort_t *s;

    IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);

    cdc_acm_init(0, USB_BASE);

    s = &vcpPort;
    s->buffering = false;
    s->port.vTable = usbVTable;

    return (serialPort_t *)s;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);

    return g_baudRate;
}

uint8_t usbVcpIsConnected(void)
{
    return usbIsConnected();
}
#endif
