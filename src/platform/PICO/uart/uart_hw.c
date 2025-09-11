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


#include "platform.h"

#ifdef USE_UART

#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart_impl.h"
#include "hardware/irq.h"

#include "serial_uart_pico.h"

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

void uartPinConfigure_hw(const serialPinConfig_t *pSerialPinConfig)
{
    for (const uartHardware_t* hardware = uartHardware; hardware < ARRAYEND(uartHardware); hardware++) {
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t* uartdev = uartDeviceFromIdentifier(identifier);
        const int resourceIndex = serialResourceIndex(identifier);
        if (uartdev == NULL || resourceIndex < 0) {
            // malformed uartHardware
            bprintf("* pico uartPinConfigure %p malformed, uartdev %p, resourceIndex %d",hardware, uartdev, resourceIndex);
            continue;
        }
        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];
        bprintf("pico uartPinConfigure hw = %p (UART%d) dev = %p,  tags rx 0x%x, tx 0x%x", hardware, UART_NUM(hardware->reg), uartdev, cfgRx, cfgTx);
        if (!cfgRx && !cfgTx) {
            continue;
        }

        for (unsigned pindex = 0; pindex < UARTHARDWARE_MAX_PINS; pindex++) {
            if (cfgRx && cfgRx == hardware->rxPins[pindex].pin) {
                uartdev->rx = hardware->rxPins[pindex];
            }

            if (cfgTx && cfgTx == hardware->txPins[pindex].pin) {
                uartdev->tx = hardware->txPins[pindex];
            }
        }

        if (uartdev->rx.pin || uartdev->tx.pin ) {
            uartdev->hardware = hardware;
        } else {
            bprintf("** uartPinConfigure_hw no compatible rx or tx pin for this hardware UART");
        }
    }
}

static void uartIrqHandler_hw(uartPort_t *s)
{
    uart_inst_t *uartInstance = UART_INST(s->USARTx);
    if ((uart_get_hw(uartInstance)->imsc & (UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS)) != 0) {
        while (uart_is_readable(uartInstance)) {
            const uint8_t ch = uart_getc(uartInstance);
            if (s->port.rxCallback) {
                s->port.rxCallback(ch, s->port.rxCallbackData);
            } else {
                // bprintf("RX %x -> buffer",ch);
                s->port.rxBuffer[s->port.rxBufferHead] = ch;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
    }

    if ((uart_get_hw(uartInstance)->imsc & UART_UARTIMSC_TXIM_BITS) != 0) {
        while (uart_is_writable(uartInstance)) {
            if (s->port.txBufferTail != s->port.txBufferHead) {
                uart_putc(uartInstance, s->port.txBuffer[s->port.txBufferTail]);
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            } else {
                // Done sending the buffer, clear the uart TX interrupt enable
                hw_clear_bits(&(uart_get_hw(uartInstance)->imsc), UART_UARTIMSC_TXIM_BITS);
                break;
            }
        }
    }
}

static void on_uart0(void)
{
    uartIrqHandler_hw(&uartDevice[UARTDEV_0].port);
}

static void on_uart1(void)
{
    uartIrqHandler_hw(&uartDevice[UARTDEV_1].port);
}

bool serialUART_hw(uartPort_t *s, uint32_t baudRate, portMode_e mode, portOptions_e options,
                   const uartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO)
{
    UNUSED(options);
    UNUSED(mode);

    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1

    if (txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
        uint32_t txPin = IO_Pin(txIO);
        bprintf("gpio set function UART on tx pin %d", txPin);
        gpio_set_function(txPin, GPIO_FUNC_UART);
    }

    if (rxIO) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        uint32_t rxPin = IO_Pin(rxIO);
        gpio_set_function(rxPin, GPIO_FUNC_UART);
        bprintf("gpio set function UART on rx pin %d", rxPin);
        gpio_set_pulls(rxPin, true, false); // Pull up
    }

    uart_inst_t *uartInstance = UART_INST(hardware->reg);
    bprintf("serialUART uart init %p baudrate %d", uartInstance, baudRate);
    uart_init(uartInstance, baudRate);

    // TODO implement - use options here...
    uart_set_hw_flow(uartInstance, false, false);
    uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);

    uart_set_fifo_enabled(uartInstance, true);

    bprintf("serialUART_hw: set exclusive handler and enable for irqn %d", hardware->irqn);

    irq_set_exclusive_handler(hardware->irqn, hardware->irqn == UART0_IRQ ? on_uart0 : on_uart1);
    irq_set_enabled(hardware->irqn, true);

    // Don't enable any uart irq yet, wait until a call to uartReconfigure...
    // (with the code as it currently is in serial_uart.c, this will prevent irq callback before rxCallback has been set)
    // TODO review serial_uart.c uartOpen()

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    s->USARTx = hardware->reg;
    bprintf("====== setting USARTx to reg == %p", s->USARTx);
    return true;
}

void uartReconfigure_hw(uartPort_t *s)
{
    uart_inst_t *uartInstance = UART_INST(s->USARTx);
    bprintf("uartReconfigure for port %p with USARTX %p", s, uartInstance);
    int achievedBaudrate = uart_init(uartInstance, s->port.baudRate);
#ifdef PICO_TRACE
    bprintf("uartReconfigure h/w %p, requested baudRate %d, achieving %d", uartInstance, s->port.baudRate, achievedBaudrate);
#else
    UNUSED(achievedBaudrate);
#endif
    // TODO make use of s->port.options
    uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);
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
}

void uartEnableTxInterrupt_hw(uartPort_t *uartPort)
{
    uart_set_irqs_enabled(UART_INST(uartPort->USARTx), uartPort->port.mode & MODE_RX, true);
}

#endif
