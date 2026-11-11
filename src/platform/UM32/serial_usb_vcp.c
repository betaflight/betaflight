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

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"

#include "pg/usb.h"


#include "drivers/usb_io.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "usb_dc.h"
#include "vcp_hal/cdc_acm.h"

// USBD_HandleTypeDef USBD_Device;

#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"

#define USB_TIMEOUT  50
typedef struct {
    uint8_t buffer[USB_VCP_RX_RING_SIZE];
    uint32_t head;
    uint32_t tail;
} CircularBuffer;

static CircularBuffer cdc_buf;

uint8_t usbVcpIsActive(void)
{
    return usbIsConnected() && usbIsConfigured();
}

static vcpPort_t vcpPort = {0};
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
    return true;
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

static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);

    const uint32_t head = __atomic_load_n(&cdc_buf.head, __ATOMIC_RELAXED);
    const uint32_t tail = __atomic_load_n(&cdc_buf.tail, __ATOMIC_ACQUIRE);
    return (tail + USB_VCP_RX_RING_SIZE - head) % USB_VCP_RX_RING_SIZE;
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    const uint32_t head = __atomic_load_n(&cdc_buf.head, __ATOMIC_RELAXED);
    if (head == __atomic_load_n(&cdc_buf.tail, __ATOMIC_ACQUIRE)) {
        return 0;
    }

    const uint8_t data = cdc_buf.buffer[head];
    __atomic_store_n(&cdc_buf.head, (head + 1U) % USB_VCP_RX_RING_SIZE, __ATOMIC_RELEASE);
    cdc_acm_resume_rx();
    return data;
}

static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    if (count <= 0 || !usbVcpIsActive()) {
        return;
    }

    bf_usbd_ep_write_buffer(data, (unsigned int)count);
}

static bool usbVcpFlush(void)
{
    return bf_usbd_ep_wait_write(USB_TIMEOUT);
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    bf_usbd_ep_write_buffer(&c, 1U);
    if (!port->buffering) {
        usbVcpFlush();
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return bf_get_tx_flag() ? 0 : 2048;
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush();
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
