/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton/Hydra - Various cleanups for Cleanflight
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifdef USE_UART1_RX_DMA
# define UART1_RX_DMA_CHANNEL DMA1_Channel5
#else
# define UART1_RX_DMA_CHANNEL 0
#endif

#ifdef USE_UART1_TX_DMA
# define UART1_TX_DMA_CHANNEL DMA1_Channel4
#else
# define UART1_TX_DMA_CHANNEL 0
#endif

#define UART2_RX_DMA_CHANNEL 0
#define UART2_TX_DMA_CHANNEL 0
#define UART3_RX_DMA_CHANNEL 0
#define UART3_TX_DMA_CHANNEL 0

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
        .rxDMAChannel = UART1_RX_DMA_CHANNEL,
        .txDMAChannel = UART1_TX_DMA_CHANNEL,
        .rxPins = { { DEFIO_TAG_E(PA10) }, { DEFIO_TAG_E(PB7) } },
        .txPins = { { DEFIO_TAG_E(PA9) }, { DEFIO_TAG_E(PB6) } },
        //.af = GPIO_AF_USART1,
        .rcc = RCC_APB2(USART1),
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1
    },
#endif
#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = USART2,
        .rxDMAChannel = UART2_RX_DMA_CHANNEL,
        .txDMAChannel = UART2_TX_DMA_CHANNEL,
        .rxPins = { { DEFIO_TAG_E(PA3) }, { DEFIO_TAG_E(PD6) } },
        .txPins = { { DEFIO_TAG_E(PA2) }, { DEFIO_TAG_E(PD5) } },
        //.af = GPIO_AF_USART2,
        .rcc = RCC_APB1(USART2),
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2,
        .rxPriority = NVIC_PRIO_SERIALUART2
    },
#endif
#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
        .rxDMAChannel = UART3_RX_DMA_CHANNEL,
        .txDMAChannel = UART3_TX_DMA_CHANNEL,
        .rxPins = { { DEFIO_TAG_E(PB11) }, { DEFIO_TAG_E(PD9) }, { DEFIO_TAG_E(PC11) } },
        .txPins = { { DEFIO_TAG_E(PB10) }, { DEFIO_TAG_E(PD8) }, { DEFIO_TAG_E(PC10) } },
        //.af = GPIO_AF_USART3,
        .rcc = RCC_APB1(USART3),
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3,
        .rxPriority = NVIC_PRIO_SERIALUART3
    },
#endif
};

void uart_tx_dma_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = (uartPort_t*)(descriptor->userParam);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    DMA_Cmd(descriptor->ref, DISABLE); // XXX F1 needs this!!!

    uartTryStartTxDMA(s);
}

// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartdev = uartDevmap[device];
    if (!uartdev) {
        return NULL;
    }

    uartPort_t *s = &uartdev->port;

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uartdev->rxBuffer;
    s->port.txBuffer = uartdev->txBuffer;
    s->port.rxBufferSize = ARRAYLEN(uartdev->rxBuffer);
    s->port.txBufferSize = ARRAYLEN(uartdev->txBuffer);

    const uartHardware_t *hardware = uartdev->hardware;

    s->USARTx = hardware->reg;

    RCC_ClockCmd(hardware->rcc, ENABLE);

    if (hardware->rxDMAChannel) {
        dmaInit(dmaGetIdentifier(hardware->rxDMAChannel), OWNER_SERIAL_RX, RESOURCE_INDEX(device));
        s->rxDMAChannel = hardware->rxDMAChannel;
        s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    }

    if (hardware->txDMAChannel) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(hardware->txDMAChannel);
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(identifier, uart_tx_dma_IRQHandler, hardware->txPriority, (uint32_t)s);
        s->txDMAChannel = hardware->txDMAChannel;
        s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    }

    IO_t rxIO = IOGetByTag(uartdev->rx.pin);
    IO_t txIO = IOGetByTag(uartdev->tx.pin);

    if (options & SERIAL_BIDIR) {
        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIO(txIO, (options & SERIAL_BIDIR_PP) ? IOCFG_AF_PP : IOCFG_AF_OD);
    } else {
        if (mode & MODE_TX) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIO(txIO, IOCFG_AF_PP);
        }

        if (mode & MODE_RX) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIO(rxIO, IOCFG_IPU);
        }
    }

    // RX/TX Interrupt
    if (!hardware->rxDMAChannel || !hardware->txDMAChannel) {
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = hardware->irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    return s;
}

void uartIrqHandler(uartPort_t *s)
{
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE && !s->rxDMAChannel) {
        // If we registered a callback, pass crap there
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->DR, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead++] = s->USARTx->DR;
            if (s->port.rxBufferHead >= s->port.rxBufferSize) {
                s->port.rxBufferHead = 0;
            }
        }
    }
    if (SR & USART_FLAG_TXE) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            s->USARTx->DR = s->port.txBuffer[s->port.txBufferTail++];
            if (s->port.txBufferTail >= s->port.txBufferSize) {
                s->port.txBufferTail = 0;
            }
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}
#endif // USE_UART
