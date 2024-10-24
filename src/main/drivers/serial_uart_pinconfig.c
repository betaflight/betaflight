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

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    for (const uartHardware_t* hardware = uartHardware; hardware < ARRAYEND(uartHardware); hardware++) {
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t* uartdev = uartDeviceFromIdentifier(identifier);
        const int resourceIndex = serialResourceIndex(identifier);
        if (uartdev == NULL || resourceIndex < 0) {
            // malformed uartHardware
            continue;
        }
        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];
#if !(defined(STM32F4) || defined(APM32F4))
        bool swap = false;
#endif
        for (unsigned pindex = 0 ; pindex < UARTHARDWARE_MAX_PINS ; pindex++) {
            if (cfgRx && cfgRx == hardware->rxPins[pindex].pin) {
                uartdev->rx = hardware->rxPins[pindex];
            }

            if (cfgTx && cfgTx == hardware->txPins[pindex].pin) {
                uartdev->tx = hardware->txPins[pindex];
            }
#if !(defined(STM32F4) || defined(APM32F4))
            // Check for swapped pins
            if (cfgTx && cfgTx == hardware->rxPins[pindex].pin) {
                uartdev->tx = hardware->rxPins[pindex];
                swap = true;
            }

            if (cfgRx && cfgRx == hardware->txPins[pindex].pin) {
                uartdev->rx = hardware->txPins[pindex];
                swap = true;
            }
#endif
        }
        if (uartdev->rx.pin || uartdev->tx.pin ) {
            uartdev->hardware = hardware;
#if !(defined(STM32F4) || defined(APM32F4))
            uartdev->pinSwap = swap;
#endif
        }
    }
}
#endif
