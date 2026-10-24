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

// USB CDC serial port: a serialPort_t over the CDC interface in usb/usb_cdc.c.

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_VCP

#include "common/utils.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/time.h"
#include "drivers/usb_io.h"

#include "scheduler/scheduler.h"

#include "usb/usb_cdc.h"

// One USB frame, matching CDC_WRITE_TIMEOUT_US.
#define USB_VCP_TIMEOUT_US 1000

static vcpPort_t vcpPort;

static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
}

static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
}

static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState),
                                     void *context)
{
    UNUSED(instance);
    cdcUsbSetCtrlLineStateCb(cb, context);
}

static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud),
                                serialPort_t *context)
{
    UNUSED(instance);
    cdcUsbSetBaudRateCb((void (*)(void *context, uint32_t baud))cb, context);
}

static FAST_CODE bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);

    static bool busy;
    static timeUs_t busySinceUs;

    if (cdcUsbTxIdle()) {
        busy = false;
        return true;
    }

    const timeUs_t nowUs = micros();

    if (!busy) {
        busy = true;
        busySinceUs = nowUs;
    }

    // Never waits; output stuck for longer than one USB frame counts as gone so polling callers end.
    return cmpTimeUs(nowUs, busySinceUs) > USB_VCP_TIMEOUT_US;
}

static FAST_CODE uint32_t usbVcpRxBytesAvailable(const serialPort_t *instance)
{
    UNUSED(instance);
    return cdcUsbBytesAvailable();
}

static FAST_CODE uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    uint8_t c;

    // Called with a byte waiting; bounded in case the host hung up meanwhile.
    const timeUs_t startUs = micros();

    while (cdcUsbRead(&c, 1) == 0) {
        if (!cdcUsbConnected() || cmpTimeUs(micros(), startUs) > USB_VCP_TIMEOUT_US) {
            return 0;
        }
    }

    return c;
}

static FAST_CODE void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    const uint8_t *p = data;

    while (count > 0) {
        const uint32_t written = cdcUsbWrite(p, count);

        if (written < (uint32_t)count) {
            // USB backpressure, not work the calling task did.
            schedulerIgnoreTaskExecTime();
        }

        if (written == 0) {
            break;
        }

        count -= (int)written;
        p += written;
    }
}

static void usbVcpFlush(vcpPort_t *port)
{
    const uint8_t count = port->txAt;
    port->txAt = 0;

    if (count > 0) {
        usbVcpWriteBuf(&port->port, port->txBuf, count);
    }
}

static FAST_CODE void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;

    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

static FAST_CODE void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static FAST_CODE void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

static FAST_CODE uint32_t usbVcpTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return cdcUsbTxBytesFree();
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpRxBytesAvailable,
        .serialTotalTxFree = usbVcpTxBytesFree,
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
    cdcUsbInit();
}

serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s = &vcpPort;
    s->port.vTable = usbVTable;
    return &s->port;
}

uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);
    return cdcUsbBaudRate();
}

uint8_t usbVcpIsConnected(void)
{
    // TASK_SERIAL calls this every cycle, so the device stack always gets a run here.
    cdcUsbTask();

#ifdef USE_USB_DETECT
    if (!usbCableIsInserted()) {
        cdcUsbUnplugged();
        return 0;
    }
#endif

    return cdcUsbMounted();
}

uint8_t usbVcpIsActive(void)
{
    // False when unplugged or suspended, even while still enumerated.
#ifdef USE_USB_DETECT
    if (!usbCableIsInserted()) {
        return 0;
    }
#endif
    return cdcUsbConnected();
}

#endif // USE_VCP
