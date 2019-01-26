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

#include "platform.h"

#ifdef USE_UART

#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

void uartPinSelect(
    uartDevice_t *device, const uartHardware_t *hardware, ioTag_t requestedRxTag, ioTag_t requestedTxTag)
{
    bool swapRequired = false;

    for (int pindex = 0; pindex < UARTHARDWARE_MAX_PINS; pindex++) {
        const uartPinDef_t hardwareRxPin = hardware->rxPins[pindex];
        const uartPinDef_t hardwareTxPin = hardware->txPins[pindex];

        if (hardwareRxPin.pin == requestedRxTag) {
            device->rx = hardwareRxPin;
        }

        if (hardwareTxPin.pin == requestedTxTag) {
            device->tx = hardwareTxPin;
        }

        // STM32F30x and STM32F7xx support RX and TX pin swapping
        if (hardwareTxPin.pin == requestedRxTag) {
            device->rx = hardwareTxPin;
            swapRequired = true;
        }

        if (hardwareRxPin.pin == requestedTxTag) {
            device->tx = hardwareRxPin;
            swapRequired = true;
        }
    }

    if (swapRequired) {
        SET_BIT(hardware->reg->CR2, USART_CR2_SWAP);
    } else {
        CLEAR_BIT(hardware->reg->CR2, USART_CR2_SWAP);
    }
}

#endif
