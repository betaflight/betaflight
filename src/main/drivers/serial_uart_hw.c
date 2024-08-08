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

/*
 * Common  UART hardware functions
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_UART) && !defined(SIMULATOR_BUILD)

#include "build/build_config.h"

#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/inverter.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartPort_t *s = &(uartdev->port);

    const uartHardware_t *hardware = uartdev->hardware;

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;

#ifdef USE_HAL_DRIVER
    s->Handle.Instance = hardware->reg;
#endif

    s->checkUsartTxOutput = checkUsartTxOutput;

#ifdef USE_DMA
    uartConfigureDma(uartdev);
    s->txDMAEmpty = true;
#endif

    if (hardware->rcc) {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    uartdev->txPinState = TX_PIN_IGNORE;

    const int ownerIndex = serialOwnerIndex(hardware->identifier); // s->port.identifier may not be initialized yet
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(hardware->identifier); // rx is always +1

// F4 - no txIO check
    if ((options & SERIAL_BIDIR) && txIO) {
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        ioConfig_t ioCfg = IO_CONFIG(
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_PULLDOWN : GPIO_PULLUP
            // STM32G4 was: (options & SERIAL_INVERTED) ? GPIO_PULLDOWN : GPIO_PULLUP
        );
#elif defined(AT32F4)
        ioConfig_t ioCfg = IO_CONFIG(
            GPIO_MODE_MUX,
            GPIO_DRIVE_STRENGTH_STRONGER,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_OUTPUT_PUSH_PULL : GPIO_OUTPUT_OPEN_DRAIN,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP_PD)) ? GPIO_PULL_DOWN : GPIO_PULL_UP
        );
#elif defined(STM32F4)
        // no inverter on F4
        ioConfig_t ioCfg =  (options & SERIAL_BIDIR_PP_PD) ? IOCFG_AF_PP_PD
                            : (options & SERIAL_BIDIR_PP) ? IOCFG_AF_PP
                            : IOCFG_AF_OD;
#endif
        IOInit(txIO, ownerTxRx, ownerIndex);

#if defined(STM32F4)
        uint8_t af = hardware->af;
#else   // all except F4 per-pin definitions
        uint8_t af = uartdev->tx.af;
#endif
        IOConfigGPIOAF(txIO, ioCfg, af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, ownerTxRx, ownerIndex);

            // AF$: if (((options & SERIAL_INVERTED) == SERIAL_NOT_INVERTED) && !(options & SERIAL_BIDIR_PP_PD)) {
            if (options & SERIAL_CHECK_TX) {
#if defined(STM32F7)  // TODO: Probably mistake
                uartdev->txPinState = TX_PIN_MONITOR;
                // Switch TX to UART output whilst UART sends idle preamble
                checkUsartTxOutput(s);
#else
                uartdev->txPinState = TX_PIN_ACTIVE;
                // Switch TX to an input with pullup so it's state can be monitored
                uartTxMonitor(s);
#endif
            } else {
#if defined(STM32F4)
                IOConfigGPIOAF(txIO, IOCFG_AF_PP_UP, hardware->af);
#else
                IOConfigGPIOAF(txIO, IOCFG_AF_PP, uartdev->tx.af);
#endif
            }
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, ownerTxRx + 1, ownerIndex);
#if defined(STM32F4)
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, hardware->af);
#else
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP, uartdev->rx.af);
#endif
        }
    }

#ifdef USE_DMA
    if (!s->rxDMAResource)
#endif
    {
#if defined(USE_HAL_DRIVER)
        HAL_NVIC_SetPriority(hardware->irqn, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));
        HAL_NVIC_EnableIRQ(hardware->irqn);
#elif defined(STM32F4)
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = hardware->irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
#elif defined(AT32F4)
        nvic_irq_enable(hardware->irqn, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));
#endif
    }
    return s;
}


void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
#if !defined(USE_INVERTER)
    UNUSED(uartPort);
#else
    bool inverted = uartPort->port.options & SERIAL_INVERTED;
    if (inverted) {
        // Enable hardware inverter if available.
        const uartDevice_t* dev = container_of(uartPort, uartDevice_t, port);  // TODO
        enableInverter(dev->hardware->identifier, true);
    }
#endif
}

#endif /* USE_UART */
