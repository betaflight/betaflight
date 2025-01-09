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

#if defined(USE_UART)

#include "build/build_config.h"

#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/inverter.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"

#include "pg/serial_uart.h"

// TODO: split this function into mcu-specific UART files ?
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
# error "Unhandled MCU type"
#endif
}

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartPort_t *s = &uartdev->port;

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

    if (hardware->rcc) {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }

#ifdef USE_DMA
    uartConfigureDma(uartdev);
    s->txDMAEmpty = true;
#endif

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    uartdev->txPinState = TX_PIN_IGNORE;

    const serialPortIdentifier_e identifier = s->port.identifier;

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
        const ioConfig_t ioCfg = IO_CONFIG(
            pushPull ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((const unsigned[]){GPIO_NOPULL, GPIO_PULLDOWN, GPIO_PULLUP})[pull]
        );
#elif defined(AT32F4)
        const ioConfig_t ioCfg = IO_CONFIG(
            GPIO_MODE_MUX,
            GPIO_DRIVE_STRENGTH_STRONGER,
            pushPull ? GPIO_OUTPUT_PUSH_PULL : GPIO_OUTPUT_OPEN_DRAIN,
            ((const gpio_pull_type[]){GPIO_PULL_NONE, GPIO_PULL_DOWN, GPIO_PULL_UP})[pull]
        );
#elif defined(STM32F4)
        // UART inverter is not supproted on F4, but keep it in line with other CPUs
        // External inverter in bidir mode would be quite problematic anyway
        const ioConfig_t ioCfg = IO_CONFIG(
            GPIO_Mode_AF,
            GPIO_Low_Speed,  // TODO: should use stronger drive
            pushPull ? GPIO_OType_PP : GPIO_OType_OD,
            ((const unsigned[]){GPIO_PuPd_NOPULL, GPIO_PuPd_DOWN, GPIO_PuPd_UP})[pull]
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
                const ioConfig_t ioCfg = IOCFG_AF_PP_UP;
#else
                const ioConfig_t ioCfg = IOCFG_AF_PP;
#endif
                IOConfigGPIOAF(txIO, ioCfg, txAf);
            }
        }

        if ((mode & MODE_RX) && rxIO) {
#if defined(STM32F4) || defined(APM32F4)
            // no inversion possible on F4, always use pullup
            const ioConfig_t ioCfg = IOCFG_AF_PP_UP;
#else
            // TODO: pullup/pulldown should be enabled for RX (based on inversion)
            const ioConfig_t ioCfg = IOCFG_AF_PP;
#endif
            IOInit(rxIO, ownerTxRx + 1, ownerIndex);
            IOConfigGPIOAF(rxIO, ioCfg, rxAf);
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

// TODO - move to serial_uart_hw.c
#ifdef USE_DMA
void uartConfigureDma(uartDevice_t *uartdev)
{
    uartPort_t *uartPort = &(uartdev->port);
    const uartHardware_t *hardware = uartdev->hardware;

#ifdef USE_DMA_SPEC
    const serialPortIdentifier_e uartPortIdentifier = hardware->identifier;
    const uartDeviceIdx_e uartDeviceIdx = uartDeviceIdxFromIdentifier(uartPortIdentifier);
    if (uartDeviceIdx == UARTDEV_INVALID) {
        return;
    }
    const int resourceIdx = serialResourceIndex(uartPortIdentifier);
    const int ownerIndex = serialOwnerIndex(uartPortIdentifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(uartPortIdentifier); // rx is always +1

    const dmaChannelSpec_t *dmaChannelSpec;
    const serialUartConfig_t *cfg = serialUartConfig(resourceIdx);
    if (!cfg) {
        return;
    }
    if (cfg->txDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, uartDeviceIdx, cfg->txDmaopt);
        if (dmaChannelSpec) {
            uartPort->txDMAResource = dmaChannelSpec->ref;
#if DMA_TRAIT_CHANNEL
            uartPort->txDMAChannel = dmaChannelSpec->channel;
#elif DMA_TRAIT_MUX
            uartPort->txDMAMuxId = dmaChannelSpec->dmaMuxId;
#endif
        }
    }

    if (cfg->rxDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, uartDeviceIdx, cfg->txDmaopt);
        if (dmaChannelSpec) {
            uartPort->rxDMAResource = dmaChannelSpec->ref;
#if DMA_TRAIT_CHANNEL
            uartPort->rxDMAChannel = dmaChannelSpec->channel;
#elif DMA_TRAIT_MUX
            uartPort->rxDMAMuxId = dmaChannelSpec->dmaMuxId;
#endif
        }
    }
#else /* USE_DMA_SPEC */
    // Non USE_DMA_SPEC does not support configurable ON/OFF of UART DMA

    if (hardware->rxDMAResource) {
        uartPort->rxDMAResource = hardware->rxDMAResource;
#if DMA_TRAIT_CHANNEL
        uartPort->rxDMAChannel = hardware->rxDMAChannel;
#elif DMA_TRAIT_MUX
        uartPort->rxDMAMuxId = hardware->rxDMAMuxId;
#endif
    }

    if (hardware->txDMAResource) {
        uartPort->txDMAResource = hardware->txDMAResource;
#if DMA_TRAIT_CHANNEL
        uartPort->txDMAChannel = hardware->txDMAChannel;
#elif DMA_TRAIT_MUX
        uartPort->txDMAMuxId = hardware->txDMAMuxId;
#endif
    }
#endif /* USE_DMA_SPEC */

    if (uartPort->txDMAResource) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->txDMAResource);
        if (dmaAllocate(identifier, ownerTxRx, ownerIndex)) {
            dmaEnable(identifier);
#if DMA_TRAIT_MUX
            dmaMuxEnable(identifier, uartPort->txDMAMuxId);
#endif
            dmaSetHandler(identifier, uartDmaIrqHandler, hardware->txPriority, (uint32_t)uartdev);
            uartPort->txDMAPeripheralBaseAddr = (uint32_t)&UART_REG_TXD(hardware->reg);
        }
    }

    if (uartPort->rxDMAResource) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->rxDMAResource);
        if (dmaAllocate(identifier, ownerTxRx + 1, ownerIndex)) {
            dmaEnable(identifier);
#if DMA_TRAIT_MUX
            dmaMuxEnable(identifier, uartPort->rxDMAMuxId);
#endif
            uartPort->rxDMAPeripheralBaseAddr = (uint32_t)&UART_REG_RXD(hardware->reg);
        }
    }
}
#endif

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
#if defined(USE_HAL_DRIVER)
    __HAL_UART_ENABLE_IT(&uartPort->Handle, UART_IT_TXE);
#elif defined(USE_ATBSP_DRIVER)
    usart_interrupt_enable(uartPort->USARTx, USART_TDBE_INT, TRUE);
#else
    USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
#endif
}

#endif /* USE_UART */
