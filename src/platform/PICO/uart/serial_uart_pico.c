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

static bool isPioUART(serialPortIdentifier_e identifier)
{
    return serialType(identifier) == SERIALTYPE_PIOUART;
}

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
#if UART_TRAIT_PINSWAP
#error UART_TRAIT_PINSWAP is not supported on PICO
#endif
    bprintf("pico uartPinConfigure");
    uartPinConfigure_hw(pSerialPinConfig);
    uartPinConfigure_pio(pSerialPinConfig);
}

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    bprintf("\nserialUART");
    uartPort_t *s = &uartdev->port;
    const serialPortIdentifier_e identifier = s->port.identifier;

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    if (!txIO && !rxIO) {
        bprintf("serialUART no pins mapped for device %p, id %d", uartdev, identifier);
        return NULL;
    }

    // SERIAL_PORT_UART0, 1, 2, 3, ...
    bool uartInitialised = false;
    if (isPioUART(identifier)) {
        uartInitialised = serialUART_pio(s, baudRate, mode, options, (pioUartHardware_t *)uartdev->hardware,
                                         identifier, txIO, rxIO);
    } else {
        uartInitialised = serialUART_hw(s, baudRate, mode, options, uartdev->hardware,
                                        identifier, txIO, rxIO);
    }

    if (!uartInitialised) {
        bprintf("* Failed to initialised uart device %p, id %d", uartdev, identifier);
        return NULL;
    }

    s->port.vTable = uartVTable;
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
    if (isPioUART(identifier)) {
        uartEnableTxInterrupt_pio(uartPort);
    } else {
        uartEnableTxInterrupt_hw(uartPort);
    }
}

void uartReconfigure(uartPort_t *s)
{
    const serialPortIdentifier_e identifier = s->port.identifier;
    if (isPioUART(identifier)) {
        uartReconfigure_pio(s);
    } else {
        uartReconfigure_hw(s);
    }
}

#endif /* USE_UART */
