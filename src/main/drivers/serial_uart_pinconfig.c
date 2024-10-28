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
 * UART pin configuration common to all MCUs.
 */

/*
 * Authors:
 * jflyper - Created as a part of configurable UART/refactoring.
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static FAST_DATA_ZERO_INIT uartDevice_t uartDevice[UARTDEV_COUNT];      // Only those configured in target.h
FAST_DATA_ZERO_INIT uartDevice_t *uartDevmap[UARTDEV_COUNT_MAX]; // Full array

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    uartDevice_t *uartdev = uartDevice;

    for (size_t hindex = 0; hindex < UARTDEV_COUNT; hindex++) {

        const uartHardware_t *hardware = &uartHardware[hindex];
        const UARTDevice_e device = hardware->device;

#if !defined(STM32F4) || !defined(APM32F4) // Don't support pin swap.
        uartdev->pinSwap = false;
#endif
        for (int pindex = 0 ; pindex < UARTHARDWARE_MAX_PINS ; pindex++) {
            if (pSerialPinConfig->ioTagRx[device] && (pSerialPinConfig->ioTagRx[device] == hardware->rxPins[pindex].pin)) {
                uartdev->rx = hardware->rxPins[pindex];
            }

            if (pSerialPinConfig->ioTagTx[device] && (pSerialPinConfig->ioTagTx[device] == hardware->txPins[pindex].pin)) {
                uartdev->tx = hardware->txPins[pindex];
            }


#if !defined(STM32F4) || !defined(APM32F4)
            // Check for swapped pins
            if (pSerialPinConfig->ioTagTx[device] && (pSerialPinConfig->ioTagTx[device] == hardware->rxPins[pindex].pin)) {
                uartdev->tx = hardware->rxPins[pindex];
                uartdev->pinSwap = true;
            }

            if (pSerialPinConfig->ioTagRx[device] && (pSerialPinConfig->ioTagRx[device] == hardware->txPins[pindex].pin)) {
                uartdev->rx = hardware->txPins[pindex];
                uartdev->pinSwap = true;
            }
#endif
        }

        if (uartdev->rx.pin || uartdev->tx.pin) {
            uartdev->hardware = hardware;
            uartDevmap[device] = uartdev++;
        }
    }
}
#endif
