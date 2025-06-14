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

#include "build/build_config.h"

#include "drivers/system.h"
#include "drivers/io.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart_impl.h"

#include "hardware/uart.h"
#include "hardware/irq.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART0
    {
        .identifier = SERIAL_PORT_UART0,
        .reg = uart0,
        .rxPins = {
            { DEFIO_TAG_E(PA1) },
            { DEFIO_TAG_E(PA3) },
            { DEFIO_TAG_E(PA13) },
            { DEFIO_TAG_E(PA15) },
            { DEFIO_TAG_E(PA17) },
            { DEFIO_TAG_E(PA19) },
            { DEFIO_TAG_E(PA29) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA31) },
            { DEFIO_TAG_E(PA33) },
            { DEFIO_TAG_E(PA35) },
            { DEFIO_TAG_E(PA45) },
            { DEFIO_TAG_E(PA47) },
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PA0) },
            { DEFIO_TAG_E(PA2) },
            { DEFIO_TAG_E(PA12) },
            { DEFIO_TAG_E(PA14) },
            { DEFIO_TAG_E(PA16) },
            { DEFIO_TAG_E(PA18) },
            { DEFIO_TAG_E(PA28) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA30) },
            { DEFIO_TAG_E(PA32) },
            { DEFIO_TAG_E(PA34) },
            { DEFIO_TAG_E(PA44) },
            { DEFIO_TAG_E(PA46) },
#endif
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
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PA9) },
            { DEFIO_TAG_E(PA11) },
            { DEFIO_TAG_E(PA21) },
            { DEFIO_TAG_E(PA23) },
            { DEFIO_TAG_E(PA25) },
            { DEFIO_TAG_E(PA27) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA37) },
            { DEFIO_TAG_E(PA39) },
            { DEFIO_TAG_E(PA41) },
            { DEFIO_TAG_E(PA43) },
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PA4) },
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PA8) },
            { DEFIO_TAG_E(PA10) },
            { DEFIO_TAG_E(PA20) },
            { DEFIO_TAG_E(PA22) },
            { DEFIO_TAG_E(PA24) },
            { DEFIO_TAG_E(PA26) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA36) },
            { DEFIO_TAG_E(PA38) },
            { DEFIO_TAG_E(PA40) },
            { DEFIO_TAG_E(PA42) },
#endif
        },
        .irqn = UART1_IRQ,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif

};

void uartIrqHandler(uartPort_t *s)
{
////    bprintf("uartIrqHandler");
    if ((uart_get_hw(s->USARTx)->imsc & (UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS)) != 0) {
        //bprintf("uartIrqHandler RX");
        while (uart_is_readable(s->USARTx)) {
            const uint8_t ch = uart_getc(s->USARTx);
            //bprintf("uartIrqHandler RX %x", ch);
            if (s->port.rxCallback) {
                s->port.rxCallback(ch, s->port.rxCallbackData);
            } else {
                bprintf("RX %x -> buffer",ch);
                s->port.rxBuffer[s->port.rxBufferHead] = ch;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
    }

    if ((uart_get_hw(s->USARTx)->imsc & UART_UARTIMSC_TXIM_BITS) != 0) {
///        bprintf("uartIrqHandler TX");
#ifdef PICO_TRACE
        int c = s->port.txBufferTail - s->port.txBufferHead;
#endif
        while (uart_is_writable(s->USARTx)) {
            if (s->port.txBufferTail != s->port.txBufferHead) {
                ///bprintf("uartIrqHandler TX put %x", s->port.txBuffer[s->port.txBufferTail]);
                uart_putc(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            } else {
                // TODO check, RX enabled based on mode?
                bprintf("uart done put %d, disabling tx interrupt",c);
                uart_set_irqs_enabled(s->USARTx, s->port.mode & MODE_RX, false);
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
    s->port.baudRate = baudRate; // TODO set by caller?
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;
    bprintf("====== setting USARTx to reg == %p", s->USARTx);

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);
    uint32_t txPin = IO_Pin(txIO);
    uint32_t rxPin = IO_Pin(rxIO);
    bprintf("serialUART retrieved recs tx,rx with pins %d, %d", txPin, rxPin);

    const serialPortIdentifier_e identifier = s->port.identifier;

    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1

    if (txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
        bprintf("gpio set function UART on tx pin %d", txPin);
        gpio_set_function(txPin, GPIO_FUNC_UART);
    }

    if (rxIO) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        gpio_set_function(rxPin, GPIO_FUNC_UART);
        gpio_set_pulls(rxPin, true, false); // Pull up
    }

    if (!txIO && !rxIO) {
        bprintf("serialUART no pins mapped for device %p", s->USARTx);
        return NULL;
    }

    bprintf("serialUART uart init %p baudrate %d", hardware->reg, baudRate);
    uart_init(hardware->reg, baudRate);

    // TODO implement - use options here...
    uart_set_hw_flow(hardware->reg, false, false);
    uart_set_format(hardware->reg, 8, 1, UART_PARITY_NONE);
    
// TODO want fifos?
////    uart_set_fifo_enabled(hardware->reg, false);
    uart_set_fifo_enabled(hardware->reg, true);

    irq_set_exclusive_handler(hardware->irqn, hardware->irqn == UART0_IRQ ? on_uart0 : on_uart1);
    irq_set_enabled(hardware->irqn, true);

    // Don't enable any uart irq yet, wait until a call to uartReconfigure...
    // (with current code in serial_uart.c, this prevents irq callback before rxCallback has been set)
    UNUSED(mode); // TODO review serial_uart.c uartOpen()

    return s;
}


// called from platform-specific uartReconfigure
void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
#if !defined(USE_INVERTER)
    UNUSED(uartPort);
#else
    const bool inverted = uartPort->port.options & SERIAL_INVERTED;
    // TODO support INVERTER, not using enableInverter(= pin based)
    enableInverter(uartPort->port.identifier, inverted);
#endif
}

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
//    bprintf("uartEnableTxInterrupt");
    if (uartPort->port.txBufferTail == uartPort->port.txBufferHead) {
        return;
    }

    // uart0TxBuffer has size 1024

#if 0
    bprintf("uartEnableTxInterrupt %d (head:0x%x, tail:0x%x)",
            uartPort->port.txBufferHead - uartPort->port.txBufferTail,
            uartPort->port.txBufferHead,
            uartPort->port.txBufferTail);
    bprintf("going to set interrupts for uart %p", uartPort->USARTx);
#endif

    // TODO Check: rx mask based on mode rather than RX interrupt pending?
    //    uart_set_irqs_enabled(s->USARTx, uart_get_hw(s->USARTx)->imsc & UART_UARTIMSC_RXIM_BITS, true);
    uart_set_irqs_enabled(uartPort->USARTx, uartPort->port.mode & MODE_RX, true);
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
    //TODO: Implement
}
#endif

void uartReconfigure(uartPort_t *s)
{
    uart_inst_t *uartInstance = s->USARTx;
    bprintf("uartReconfigure for port %p with USARTX %p", s, uartInstance);
    int achievedBaudrate = uart_init(uartInstance, s->port.baudRate);
#ifdef PICO_TRACE
    bprintf("uartReconfigure h/w %p, requested baudRate %d, achieving %d", uartInstance, s->port.baudRate, achievedBaudrate);
#else
    UNUSED(achievedBaudrate);
#endif
    uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);

    // TODO fifo or not to fifo?
    //    uart_set_fifo_enabled(s->USARTx, false);
    uart_set_fifo_enabled(uartInstance, true);
    uartConfigureExternalPinInversion(s);
    uart_set_hw_flow(uartInstance, false, false);

// TODO would like to verify rx pin has been setup?
//    if ((s->mode & MODE_RX) && rxIO) {
    if (s->port.mode & MODE_RX) {
        bprintf("serialUART setting RX irq");
        uart_set_irqs_enabled(uartInstance, true, false);
    }

    bprintf("uartReconfigure note port.mode = 0x%x", s->port.mode);
    // TODO should we care about MODE_TX ?

#if 0
    uint32_t uartFr = uart_get_hw(uartInstance)->fr;
    bprintf("uartReconfigure flag register 0x%x",uartFr);
    bprintf("uartReconfigure extra call to on_uart1");
    on_uart1();
    bprintf("put some in...");
    uart_putc(uartInstance, 'A');
    uart_putc(uartInstance, 'B');
    uart_putc(uartInstance, 'C');
    uartFr = uart_get_hw(uartInstance)->fr;
    bprintf("uartReconfigure flag register 0x%x",uartFr);
    bprintf("wait a mo");
    extern void delayMicroseconds(uint32_t);
    delayMicroseconds(123456);
    uartFr = uart_get_hw(uartInstance)->fr;
    bprintf("uartReconfigure flag register 0x%x",uartFr);
    bprintf("uartReconfigure extra special call to on_uart1");
    on_uart1();
#endif
}

#endif /* USE_UART */
