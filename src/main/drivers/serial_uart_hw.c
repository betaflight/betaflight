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
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

// TODO: split this function into mcu-specific UART files
static void enableRxIrq(const uartHardware_t *hardware)
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
#elif defined(APM32F4)
        DAL_NVIC_SetPriority(hardware->irqn, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));
        DAL_NVIC_EnableIRQ(hardware->irqn);
#else
#error "Unhandled MCU type"
#endif
}

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

    const serialPortIdentifier_e identifier = uartdev->port.port.identifier;

    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1

    // prepare AF modes
#if UART_TRAIT_AF_PORT
    uint8_t rxAf = hardware->af;
    uint8_t txAf = hardware->af;
#elif UART_TRAIT_AF_PIN
    uint8_t rxAf = uartdev->rx.af;
    uint8_t txAf = uartdev->tx.af;
#endif

// Note: F4 did not check txIO before refactoring
    if ((options & SERIAL_BIDIR) && txIO) {
        // pushPull / openDrain
        const bool pushPull = serialOptions_pushPull(options);
        // pull direction
        const serialPullMode_t pull = serialOptions_pull(options);
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
        // Note: APM32F4 is different from STM32F4 here
        ioConfig_t ioCfg = IO_CONFIG(
            pushPull ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((unsigned[]){GPIO_NOPULL, GPIO_PULLDOWN, GPIO_PULLUP})[pull]
        );
#elif defined(AT32F4)
        ioConfig_t ioCfg = IO_CONFIG(
            GPIO_MODE_MUX,
            GPIO_DRIVE_STRENGTH_STRONGER,
            pushPull ? GPIO_OUTPUT_PUSH_PULL : GPIO_OUTPUT_OPEN_DRAIN,
            ((gpio_pull_type[]){GPIO_PULL_NONE, GPIO_PULL_DOWN, GPIO_PULL_UP})[pull]
        );
#elif defined(STM32F4)
        // no inverter on F4, but keep it in line with other CPUs
        // External inverter in bidir mode would be quite problematic anyway
        ioConfig_t ioCfg = IO_CONFIG(
            GPIO_Mode_AF,
            GPIO_Low_Speed,  // TODO: should use stronger drive
            pushPull ? GPIO_OType_PP : GPIO_OType_OD,
            ((unsigned[]){GPIO_PuPd_NOPULL, GPIO_PuPd_DOWN, GPIO_PuPd_UP})[pull]
        );
#endif
        IOInit(txIO, ownerTxRx, ownerIndex);
        IOConfigGPIOAF(txIO, ioCfg, txAf);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, ownerTxRx, ownerIndex);

            if (options & SERIAL_CHECK_TX) {
#if defined(STM32F7)
                // see https://github.com/betaflight/betaflight/pull/13021
                // Allow for F7 UART idle preamble to be sent on startup
                uartdev->txPinState = TX_PIN_MONITOR;
                // Switch TX to UART output whilst UART sends idle preamble
                checkUsartTxOutput(s);
#else
                uartdev->txPinState = TX_PIN_ACTIVE;
                // Switch TX to an input with pullup so it's state can be monitored
                uartTxMonitor(s);
#endif
            } else {
#if defined(STM32F4) || defined(APM32F4)
                // TODO: no need for pullup on TX only pin
                IOConfigGPIOAF(txIO, IOCFG_AF_PP_UP, txAf);
#else
                IOConfigGPIOAF(txIO, IOCFG_AF_PP, txAf);
#endif
            }
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, ownerTxRx + 1, ownerIndex);
#if defined(STM32F4) || defined(APM32F4)
            // no inversion possible on F4, always use pullup
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, rxAf);
#else
            // TODO: pullup/pulldown should be enabled for RX (based on inversion)
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP, rxAf);
#endif
        }
    }

    if (true
#ifdef USE_DMA
        && !s->rxDMAResource  // do not enable IRW if using rxDMA
#endif
        ) {
        enableRxIrq(hardware);
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

#endif /* USE_UART */
