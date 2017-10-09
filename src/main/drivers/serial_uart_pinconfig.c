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

/*
 * UART pin configuration common to all MCUs.
 */

/*
 * Authors:
 * jflyper - Created as a part of configurable UART/refactoring.
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

uartDevice_t uartDevice[UARTDEV_COUNT];      // Only those configured in target.h
uartDevice_t *uartDevmap[UARTDEV_COUNT_MAX]; // Full array

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    uartDevice_t *uartdev = uartDevice;

    for (size_t hindex = 0; hindex < UARTDEV_COUNT; hindex++) {

        const uartHardware_t *hardware = &uartHardware[hindex];
        const UARTDevice_e device = hardware->device;

        for (int pindex = 0 ; pindex < UARTHARDWARE_MAX_PINS ; pindex++) {
            if (hardware->rxPins[pindex] && (hardware->rxPins[pindex] == pSerialPinConfig->ioTagRx[device]))
                uartdev->rx = pSerialPinConfig->ioTagRx[device];

            if (hardware->txPins[pindex] && (hardware->txPins[pindex] == pSerialPinConfig->ioTagTx[device]))
                uartdev->tx = pSerialPinConfig->ioTagTx[device];
        }

        if (uartdev->rx || uartdev->tx) {
            uartdev->hardware = hardware;
            uartDevmap[device] = uartdev++;
        }
    }
}
