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

#include "hardware/irq.h"
#include "hardware/uart.h"

#include "serial_uart_pico.h"

static const PIO uartPio = UART_PIO_INSTANCE;

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

    // PIO-based UARTs. For now, hardwired to UARTs 2,3 on PIO number UART_PIO_INDEX.
#ifdef USE_UART2
    {
        .identifier = SERIAL_PORT_UART2,
        .reg = (USART_TypeDef *)uartPio,
        .irqn = PIO_IRQ_NUM(uartPio, 0),
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif

#ifdef USE_UART3
    {
        .identifier = SERIAL_PORT_UART3,
        .reg = (USART_TypeDef *)uartPio,
        .irqn = PIO_IRQ_NUM(uartPio, 1),
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
};

bool isHardwareUART(serialPortIdentifier_e identifier)
{
    return identifier == SERIAL_PORT_UART0 || identifier == SERIAL_PORT_UART1;
}

uartPinDef_t makePinDef(ioTag_t tag)
{
    uartPinDef_t ret = { .pin = tag };
    return ret;
}

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
#if UART_TRAIT_PINSWAP
#error UART_TRAIT_PINSWAP should not be defined (not supported)
#endif
    bprintf("pico uartPinConfigure");

    int pinIndexMin = 48;
    int pinIndexMax = -1;
    uartPioBase = 0;
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
        bprintf("pico uartPinConfigure hw = %p dev = %p,  tags rx 0x%x, tx 0x%x", hardware, uartdev, cfgRx, cfgTx);
        if (!cfgRx && !cfgTx) {
            continue;
        }

        if (isHardwareUART(identifier)) {
            for (unsigned pindex = 0; pindex < UARTHARDWARE_MAX_PINS; pindex++) {
                if (cfgRx && cfgRx == hardware->rxPins[pindex].pin) {
                    uartdev->rx = hardware->rxPins[pindex];
                }

                if (cfgTx && cfgTx == hardware->txPins[pindex].pin) {
                    uartdev->tx = hardware->txPins[pindex];
                }
            }

        } else {
            // software UART by PIO
            // On a single PIO block, we are restricted either to pins 0-31 or pins 16-47.
            pinIndexMin = cfgRx && (DEFIO_TAG_PIN(cfgRx) < pinIndexMin) ? DEFIO_TAG_PIN(cfgRx) : pinIndexMin;
            pinIndexMax = cfgRx && (DEFIO_TAG_PIN(cfgRx) > pinIndexMax) ? DEFIO_TAG_PIN(cfgRx) : pinIndexMax;
            pinIndexMin = cfgTx && (DEFIO_TAG_PIN(cfgTx) < pinIndexMin) ? DEFIO_TAG_PIN(cfgTx) : pinIndexMin;
            pinIndexMax = cfgTx && (DEFIO_TAG_PIN(cfgTx) > pinIndexMax) ? DEFIO_TAG_PIN(cfgTx) : pinIndexMax;
            if (pinIndexMax >= 32) {
                if (pinIndexMin < 16) {
                    bprintf("* Not configuring UART%d (PIO can't span pins min %d max %d)",
                            uartDeviceIdxFromIdentifier(identifier), pinIndexMin, pinIndexMax);
                    continue;
                } else {
                    uartPioBase = 16;
                }
            }

            if (cfgRx) {
                uartdev->rx = makePinDef(cfgRx);
            }

            if (cfgTx) {
                uartdev->tx = makePinDef(cfgTx);
            }
        }

        if (uartdev->rx.pin || uartdev->tx.pin ) {
            uartdev->hardware = hardware;
        } else {
            bprintf("\n ** unexpected no rx.pin or tx.pin even though cfgRx or cfgTx");
        }
    }

    bprintf("pico uartPinConfigure pio%d pin min, max = %d, %d; setting gpio base to %d", PIO_NUM(uartPio), pinIndexMin, pinIndexMax, uartPioBase);
}


uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    bprintf("\nserialUART");
    uartPort_t *s = &uartdev->port;
    const uartHardware_t *hardware = uartdev->hardware;

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    if (!txIO && !rxIO) {
        bprintf("serialUART no pins mapped for device %p", s->USARTx);
        return NULL;
    }

    // SERIAL_PORT_UART0, 1, 2, 3, ...
    const serialPortIdentifier_e identifier = s->port.identifier;

    bool uartInitialised;
    if (isHardwareUART(identifier)) {
        uartInitialised = serialUART_hw(baudRate, mode, options,
                                              hardware, identifier, txIO, rxIO);
    } else {
        uartInitialised = serialUART_pio(baudRate, mode, options,
                                              hardware, identifier, txIO, rxIO);
    }

    if (!uartInitialised) {
        bprintf("* Failed to initialised uart device %p, id %d", hardware->reg, identifier);
        return NULL;
    }

    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate; // TODO set by caller?
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;
    bprintf("====== setting USARTx to reg == %p", s->USARTx);
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

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
    //TODO: Implement
}
#endif

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
    if (uartPort->port.txBufferTail == uartPort->port.txBufferHead) {
        return;
    }

    const serialPortIdentifier_e identifier = uartPort->port.identifier;
    if (isHardwareUART(identifier)) {
        uartEnableTxInterrupt_hw(uartPort);
    } else {
        uartEnableTxInterrupt_pio(uartPort);
    }
}

void uartReconfigure(uartPort_t *s)
{
    const serialPortIdentifier_e identifier = s->port.identifier;
    if (isHardwareUART(identifier)) {
        uartReconfigure_hw(s);
    } else {
        uartReconfigure_pio(s);
    }
}

#endif /* USE_UART */
