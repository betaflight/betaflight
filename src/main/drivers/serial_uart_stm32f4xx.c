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
 * jflyper - Refactoring, cleanup and made pin-configurable
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
        .DMAChannel = DMA_Channel_4,
#ifdef USE_UART1_RX_DMA
        .rxDMAStream = DMA2_Stream5,
#endif
#ifdef USE_UART1_TX_DMA
        .txDMAStream = DMA2_Stream7,
#endif
        .rxPins = { { DEFIO_TAG_E(PA10) }, { DEFIO_TAG_E(PB7) } },
        .txPins = { { DEFIO_TAG_E(PA9) }, { DEFIO_TAG_E(PB6) } },
        .af = GPIO_AF_USART1,
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
        .DMAChannel = DMA_Channel_4,
#ifdef USE_UART2_RX_DMA
        .rxDMAStream = DMA1_Stream5,
#endif
#ifdef USE_UART2_TX_DMA
        .txDMAStream = DMA1_Stream6,
#endif
        .rxPins = { { DEFIO_TAG_E(PA3) }, { DEFIO_TAG_E(PD6) } },
        .txPins = { { DEFIO_TAG_E(PA2) }, { DEFIO_TAG_E(PD5) } },
        .af = GPIO_AF_USART2,
        .rcc = RCC_APB1(USART2),
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2
    },
#endif

#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
        .DMAChannel = DMA_Channel_4,
#ifdef USE_UART3_RX_DMA
        .rxDMAStream = DMA1_Stream1,
#endif
#ifdef USE_UART3_TX_DMA
        .txDMAStream = DMA1_Stream3,
#endif
        .rxPins = { { DEFIO_TAG_E(PB11) }, { DEFIO_TAG_E(PC11) }, { DEFIO_TAG_E(PD9) } },
        .txPins = { { DEFIO_TAG_E(PB10) }, { DEFIO_TAG_E(PC10) }, { DEFIO_TAG_E(PD8) } },
        .af = GPIO_AF_USART3,
        .rcc = RCC_APB1(USART3),
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3
    },
#endif

#ifdef USE_UART4
    {
        .device = UARTDEV_4,
        .reg = UART4,
        .DMAChannel = DMA_Channel_4,
#ifdef USE_UART4_RX_DMA
        .rxDMAStream = DMA1_Stream2,
#endif
#ifdef USE_UART4_TX_DMA
        .txDMAStream = DMA1_Stream4,
#endif
        .rxPins = { { DEFIO_TAG_E(PA1) }, { DEFIO_TAG_E(PC11) } },
        .txPins = { { DEFIO_TAG_E(PA0) }, { DEFIO_TAG_E(PC10) } },
        .af = GPIO_AF_UART4,
        .rcc = RCC_APB1(UART4),
        .irqn = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4
    },
#endif

#ifdef USE_UART5
    {
        .device = UARTDEV_5,
        .reg = UART5,
        .DMAChannel = DMA_Channel_4,
#ifdef USE_UART5_RX_DMA
        .rxDMAStream = DMA1_Stream0,
#endif
#ifdef USE_UART5_TX_DMA
        .txDMAStream = DMA1_Stream7,
#endif
        .rxPins = { { DEFIO_TAG_E(PD2) } },
        .txPins = { { DEFIO_TAG_E(PC12) } },
        .af = GPIO_AF_UART5,
        .rcc = RCC_APB1(UART5),
        .irqn = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5
    },
#endif

#ifdef USE_UART6
    {
        .device = UARTDEV_6,
        .reg = USART6,
        .DMAChannel = DMA_Channel_5,
#ifdef USE_UART6_RX_DMA
        .rxDMAStream = DMA2_Stream1,
#endif
#ifdef USE_UART6_TX_DMA
        .txDMAStream = DMA2_Stream6,
#endif
        .rxPins = { { DEFIO_TAG_E(PC7) }, { DEFIO_TAG_E(PG9) } },
        .txPins = { { DEFIO_TAG_E(PC6) }, { DEFIO_TAG_E(PG14) } },
        .af = GPIO_AF_USART6,
        .rcc = RCC_APB2(USART6),
        .irqn = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6
    },
#endif
};

static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
}

void dmaIRQHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_FEIF))
        {
            DMA_CLEAR_FLAG(descriptor, DMA_IT_FEIF);
        }
        handleUsartTxDma(s);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_DMEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_DMEIF);
    }
}

// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uart = uartDevmap[device];
    if (!uart) return NULL;

    const uartHardware_t *hardware = uart->hardware;

    if (!hardware) return NULL; // XXX Can't happen !?

    uartPort_t *s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uart->rxBuffer;
    s->port.txBuffer = uart->txBuffer;
    s->port.rxBufferSize = sizeof(uart->rxBuffer);
    s->port.txBufferSize = sizeof(uart->txBuffer);

    s->USARTx = hardware->reg;

    if (hardware->rxDMAStream) {
        dmaInit(dmaGetIdentifier(hardware->rxDMAStream), OWNER_SERIAL_RX, RESOURCE_INDEX(device));
        s->rxDMAChannel = hardware->DMAChannel;
        s->rxDMAStream = hardware->rxDMAStream;
        s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    }

    if (hardware->txDMAStream) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(hardware->txDMAStream);
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(identifier, dmaIRQHandler, hardware->txPriority, (uint32_t)uart);
        s->txDMAChannel = hardware->DMAChannel;
        s->txDMAStream = hardware->txDMAStream;
        s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    }

    IO_t txIO = IOGetByTag(uart->tx.pin);
    IO_t rxIO = IOGetByTag(uart->rx.pin);

    if (hardware->rcc) {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }

    if (options & SERIAL_BIDIR) {
        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, (options & SERIAL_BIDIR_PP) ? IOCFG_AF_PP : IOCFG_AF_OD, hardware->af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(txIO, IOCFG_AF_PP_UP, hardware->af);
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, hardware->af);
        }
    }

    if (!(s->rxDMAChannel)) {
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
    if (!s->rxDMAStream && (USART_GetITStatus(s->USARTx, USART_IT_RXNE) == SET)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->DR, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    if (!s->txDMAStream && (USART_GetITStatus(s->USARTx, USART_IT_TXE) == SET)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }

    if (USART_GetITStatus(s->USARTx, USART_IT_ORE) == SET)
    {
        USART_ClearITPendingBit (s->USARTx, USART_IT_ORE);
    }
}
#endif
