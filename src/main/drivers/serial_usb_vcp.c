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

#include "usb_core.h"
#include "usb_init.h"
#include "hw_config.h"

#include "drivers/system.h"

#include "serial.h"
#include "serial_usb_vcp.h"


#define USB_TIMEOUT  50

static vcpPort_t vcpPort;

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

uint8_t usbVcpAvailable(serialPort_t *instance)
{
    UNUSED(instance);

    return receiveLength & 0xFF; // FIXME use uint32_t return type everywhere
}

uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

    uint8_t buf[1];

    uint32_t rxed = 0;

    while (rxed < 1) {
        rxed += CDC_Receive_DATA((uint8_t*)buf + rxed, 1 - rxed);
    }

    return buf[0];
}

void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    UNUSED(instance);

    uint32_t txed;
    uint32_t start = millis();

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    do {
        txed = CDC_Send_DATA((uint8_t*)&c, 1);
    } while (txed < 1 && (millis() - start < USB_TIMEOUT));

}

uint8_t usbTxBytesFree() {
    // Because we block upon transmit and don't buffer bytes, our "buffer" capacity is effectively unlimited.
    return 255;
}

const struct serialPortVTable usbVTable[] = { { usbVcpWrite, usbVcpAvailable, usbTxBytesFree, usbVcpRead, usbVcpSetBaudRate, isUsbVcpTransmitBufferEmpty, usbVcpSetMode } };

serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s;

    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    s = &vcpPort;
    s->port.vTable = usbVTable;

    return (serialPort_t *)s;
}
