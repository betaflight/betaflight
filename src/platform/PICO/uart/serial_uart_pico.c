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

static void setTxMonitorState(uartDevice_t *uartDev, IO_t txIO)
{
    // Disable USART TX output, set pin state to MONITOR.
    uint32_t txPin = IO_Pin(txIO);
    uartDev->txPinState = TX_PIN_MONITOR;
    gpio_set_function(txPin, GPIO_FUNC_SIO);
    gpio_set_dir(txPin, false); // set as input
    gpio_pull_up(txPin);
}

static void clearTxMonitorState(uartPort_t *s, bool isPio, uartDevice_t *uartDev, IO_t txIO)
{
    // Enable USART TX output, revert pin state to ACTIVE.
    uint32_t txPin = IO_Pin(txIO);
    gpio_set_pulls(txPin, false, false); // lose the pull-up.

    if (isPio) {
        uartSelectFunction_pio(s, txPin);
    } else {
        uartSelectFunction_hw(s, txPin);
    }

    uartDev->txPinState = TX_PIN_ACTIVE;
}

bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uartDev = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uartDev->tx.pin);

    if (txIO) {
        bool isPio = isPioUART(s->port.identifier);

        // For SERIAL_CHECK_TX option, check at this point whether a transmission has finished.
        if (uartDev->txPinState == TX_PIN_ACTIVE && s->port.txBufferHead == s->port.txBufferTail) {
            // Nothing in TX buffer, check for FIFO and transmission completion.
            bool isTxComplete = isPio ? isTxComplete_pio(s) : isTxComplete_hw(s);
            if (isTxComplete) {
                setTxMonitorState(uartDev, txIO);
            }
        }

        // SERIAL_CHECK_TX option, if monitoring, check to see if good to go ahead with transmission.
        if (uartDev->txPinState == TX_PIN_MONITOR) {
            if (IORead(txIO)) {
                // TX is high so we're good to transmit.
                clearTxMonitorState(s, isPio, uartDev, txIO);
                return true;
            } else {
                // TX line is pulled low so don't enable USART TX
                return false;
            }
        }
    }

    return true;
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

    if (options & SERIAL_INVERTED) {
        if (rxIO) {
            bprintf("UART inverting input on GPIO %d", IO_Pin(rxIO));
            gpio_set_inover(IO_Pin(rxIO), GPIO_OVERRIDE_INVERT);
        }

        if (txIO) {
            bprintf("UART inverting output on GPIO %d", IO_Pin(txIO));
            gpio_set_outover(IO_Pin(txIO), GPIO_OVERRIDE_INVERT);
        }
    }

    if (txIO && (options & SERIAL_CHECK_TX)) {
        bprintf("serialUART option SERIAL_CHECK_TX");
        s->checkUsartTxOutput = checkUsartTxOutput;
        setTxMonitorState(uartdev, txIO);
    } else {
        // Not strictly necessary, because calling code checks for checkUsartTxOutput being set on the port.
        uartdev->txPinState = TX_PIN_IGNORE;
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
    //TODO: Implement in place of uartEnableTxInterrupt
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
