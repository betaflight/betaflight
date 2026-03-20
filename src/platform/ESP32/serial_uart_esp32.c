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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

// ESP32-S3 has 3 UART controllers (UART0, UART1, UART2)
// UART buffers are defined in drivers/serial_uart.c

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART0
    {
        .identifier = SERIAL_PORT_UART0,
        .reg = UART0,
        .rxPins = { { .pin = DEFIO_TAG_E(PA44) }, },
        .txPins = { { .pin = DEFIO_TAG_E(PA43) }, },
        .af = 0,
        .irqn = 0,
        .txPriority = 0,
        .rxPriority = 0,
        .txBuffer = uart0TxBuffer,
        .rxBuffer = uart0RxBuffer,
        .txBufferSize = sizeof(uart0TxBuffer),
        .rxBufferSize = sizeof(uart0RxBuffer),
    },
#endif
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_UART1,
        .reg = UART1,
        .rxPins = { { .pin = DEFIO_TAG_E(PA18) }, },
        .txPins = { { .pin = DEFIO_TAG_E(PA17) }, },
        .af = 0,
        .irqn = 0,
        .txPriority = 0,
        .rxPriority = 0,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
};

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    UNUSED(pSerialPinConfig);
    // TODO: configure UART pins via ESP-IDF
}

void uartReconfigure(uartPort_t *uartPort)
{
    UNUSED(uartPort);
    // TODO: uart_param_config()
}

void uartConfigureDma(uartDevice_t *uartdev)
{
    UNUSED(uartdev);
}

void uartIrqHandler(uartPort_t *s)
{
    UNUSED(s);
}

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
    UNUSED(uartPort);
}

void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
}

uartPort_t *serialUART(uartDevice_t *uart, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    UNUSED(uart);
    UNUSED(baudRate);
    UNUSED(mode);
    UNUSED(options);
    return NULL;
}
