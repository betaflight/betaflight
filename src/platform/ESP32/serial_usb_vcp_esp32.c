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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_VCP

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/usb_serial_jtag_ll.h"
#pragma GCC diagnostic pop

static vcpPort_t vcpPort = { 0 };

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
    return usb_serial_jtag_ll_txfifo_writable();
}

static uint32_t usbVcpRxBytesAvailable(const serialPort_t *instance)
{
    UNUSED(instance);
    return usb_serial_jtag_ll_rxfifo_data_available() ? 1 : 0;
}

static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);
    uint8_t ch = 0;
    usb_serial_jtag_ll_read_rxfifo(&ch, 1);
    return ch;
}

static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);
    const uint8_t *p = (const uint8_t *)data;
    int remaining = count;
    while (remaining > 0) {
        int written = usb_serial_jtag_ll_write_txfifo(p, remaining);
        p += written;
        remaining -= written;
        if (remaining > 0) {
            usb_serial_jtag_ll_txfifo_flush();
        }
    }
    usb_serial_jtag_ll_txfifo_flush();
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    UNUSED(instance);
    usb_serial_jtag_ll_write_txfifo(&c, 1);
    usb_serial_jtag_ll_txfifo_flush();
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    UNUSED(instance);
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return usb_serial_jtag_ll_txfifo_writable() ? 64 : 0;
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    UNUSED(instance);
    usb_serial_jtag_ll_txfifo_flush();
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpRxBytesAvailable,
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
    // The USB Serial/JTAG controller is enabled by default after ROM boot
    // Enable bus clock if needed
    // usb_serial_jtag_ll_enable_bus_clock(true);
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
    return 115200;
}

uint8_t usbVcpIsConnected(void)
{
    return 1;  // USB Serial/JTAG is always available when USB is connected
}

#endif // USE_VCP
