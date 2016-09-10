/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"
#include "common/utils.h"
#include "drivers/io.h"

#include "vcp_hal/usbd_cdc_interface.h"

#include "drivers/system.h"

#include "serial.h"
#include "serial_usb_vcp.h"


#define USB_TIMEOUT  50

static vcpPort_t vcpPort;
USBD_HandleTypeDef USBD_Device;


void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);

    // TODO implement
}

void usbVcpSetMode(serialPort_t *instance, portMode_t mode)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

bool isUsbVcpTransmitBufferEmpty(serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

uint32_t usbVcpAvailable(serialPort_t *instance)
{
    UNUSED(instance);
    uint32_t receiveLength = vcpAvailable();
    return receiveLength;
}

uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);
    return vcpRead();
}

static void usbVcpWriteBuf(serialPort_t *instance, void *data, int count)
{
    UNUSED(instance);


    if (!vcpIsConnected()) {
        return;
    }

    uint32_t start = millis();
    for (uint8_t *p = data; count > 0; ) {
        uint32_t txed = vcpWrite(*p);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
}

static bool usbVcpFlush(vcpPort_t *port)
{
    uint8_t count = port->txAt;
    port->txAt = 0;

    if (count == 0) {
        return true;
    }
    if (!vcpIsConnected()) {
        return false;
    }
    
    uint32_t txed;
    uint32_t start = millis();

    do {
        txed += vcpWrite(port->txBuf[txed]);
    } while (txed != count && (millis() - start < USB_TIMEOUT));

    return txed == count;
}

static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

uint8_t usbTxBytesFree()
{
    // Because we block upon transmit and don't buffer bytes, our "buffer" capacity is effectively unlimited.
    return 255;
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
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
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite,
        .writeBuf = usbVcpWriteBuf
    }
};

serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s;
    
    /* Init Device Library */
    USBD_Init(&USBD_Device, &VCP_Desc, 0);
    
    /* Add Supported Class */
    USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
    
    /* Add CDC Interface Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
    
    /* Start Device Process */
    USBD_Start(&USBD_Device);

    s = &vcpPort;
    s->port.vTable = usbVTable;

    return (serialPort_t *)s;
}
uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);

    return vcpBaudrate();
}
