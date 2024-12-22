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

#ifdef USE_UART

#include "drivers/system.h"
#include "drivers/io.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_impl.h"
#include "serial_uart_impl.h"

#include "hardware/uart.h"
#include "hardware/irq.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART0
    {
        .identifier = SERIAL_PORT_UART0,
        .reg = uart0,
        .rxPins = {
            { DEFIO_TAG_E(PA1) },
            { DEFIO_TAG_E(PA17) },
        },
        .txPins = {
            { DEFIO_TAG_E(PA0) },
            { DEFIO_TAG_E(PA16) },
        },
        .irqn = UART0_IRQ,
        .txBuffer = uart0TxBuffer,
        .rxBuffer = uart0RxBuffer,
        .txBufferSize = sizeof(uart0TxBuffer),
        .rxBufferSize = sizeof(uart0RxBuffer),
    },
#endif

#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_UART1,
        .reg = uart1,
        .rxPins = {
            { DEFIO_TAG_E(PA5) },
            { DEFIO_TAG_E(PA9) },
            { DEFIO_TAG_E(PA25) },
        },
        .txPins = {
            { DEFIO_TAG_E(PA4) },
            { DEFIO_TAG_E(PA20) },
            { DEFIO_TAG_E(PA24) },
        },
        .irqn = UART1_IRQ,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif

};

static void uartIrqHandler(uartPort_t *s)
{
    if ((uart_get_hw(s->USARTx)->imsc & UART_UARTIMSC_RXIM_BITS) != 0) {
        while (uart_is_readable(s->USARTx)) {
            const uint8_t ch = uart_getc(s->USARTx);
            if (s->port.rxCallback) {
                s->port.rxCallback(ch, s->port.rxCallbackData);
            } else {
                s->port.rxBuffer[s->port.rxBufferHead] = ch;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
    }

    if ((uart_get_hw(s->USARTx)->imsc & UART_UARTIMSC_TXIM_BITS) != 0) {
        while (uart_is_writable(s->USARTx)) {
            if (s->port.txBufferTail != s->port.txBufferHead) {
                uart_putc(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            } else {
                uart_set_irq_enables(s->USARTx, true, false);
                break;
            }
        }
    }
}

static void on_uart0(void)
{
    uartIrqHandler(&uartDevice[UARTDEV_0].port);
}

static void on_uart1(void)
{
    uartIrqHandler(&uartDevice[UARTDEV_1].port);
}

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    UNUSED(options);

    uartPort_t *s = &uartdev->port;

    const uartHardware_t *hardware = uartdev->hardware;

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    const serialPortIdentifier_e identifier = s->port.identifier;

    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1

    IOInit(txIO, ownerTxRx, ownerIndex);
    IOInit(txIO, ownerTxRx, ownerIndex);

    uart_init(hardware->reg, baudRate);

    if ((mode & MODE_TX) && txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
        gpio_set_function(IO_Pin(txIO), GPIO_FUNC_UART);
    }

    if ((mode & MODE_RX) && rxIO) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        gpio_set_function(IO_Pin(rxIO), GPIO_FUNC_UART);
    }

    uart_set_hw_flow(hardware->reg, false, false);
    uart_set_format(hardware->reg, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(hardware->reg, false);

    irq_set_exclusive_handler(hardware->irqn, hardware->irqn == UART0_IRQ ? on_uart0 : on_uart1);
    irq_set_enabled(hardware->irqn, true);
    if ((mode & MODE_RX) && rxIO) {
        uart_set_irq_enables(hardware->reg, true, false);
    }

    return s;
}

// called from platform-specific uartReconfigure
void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
#if !defined(USE_INVERTER)
    UNUSED(uartPort);
#else
    const bool inverted = uartPort->port.options & SERIAL_INVERTED;
    enableInverter(uartPort->port.identifier, inverted);
#endif
}

void uartTryStartTx(uartPort_t *s)
{
    if (s->port.txBufferTail == s->port.txBufferHead) {
        return;
    }
    uart_set_irq_enables(s->USARTx, uart_get_hw(s->USARTx)->imsc & UART_UARTIMSC_RXIM_BITS, true);
}

void uartReconfigure(uartPort_t *s)
{
    uart_deinit(s->USARTx);
    uart_init(s->USARTx, s->port.baudRate);
    uart_set_format(s->USARTx, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(s->USARTx, false);
    uartConfigureExternalPinInversion(s);
}

#endif /* USE_UART */
