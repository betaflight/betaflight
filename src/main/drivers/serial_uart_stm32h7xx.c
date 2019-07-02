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
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifndef UART1_TX_DMA_STREAM
#define UART1_TX_DMA_STREAM NULL
#endif
#ifndef UART1_RX_DMA_STREAM
#define UART1_RX_DMA_STREAM NULL
#endif
#ifndef UART2_TX_DMA_STREAM
#define UART2_TX_DMA_STREAM NULL
#endif
#ifndef UART2_RX_DMA_STREAM
#define UART2_RX_DMA_STREAM NULL
#endif
#ifndef UART3_TX_DMA_STREAM
#define UART3_TX_DMA_STREAM NULL
#endif
#ifndef UART3_RX_DMA_STREAM
#define UART3_RX_DMA_STREAM NULL
#endif
#ifndef UART4_TX_DMA_STREAM
#define UART4_TX_DMA_STREAM NULL
#endif
#ifndef UART4_RX_DMA_STREAM
#define UART4_RX_DMA_STREAM NULL
#endif
#ifndef UART5_TX_DMA_STREAM
#define UART5_TX_DMA_STREAM NULL
#endif
#ifndef UART5_RX_DMA_STREAM
#define UART5_RX_DMA_STREAM NULL
#endif
#ifndef UART6_TX_DMA_STREAM
#define UART6_TX_DMA_STREAM NULL
#endif
#ifndef UART6_RX_DMA_STREAM
#define UART6_RX_DMA_STREAM NULL
#endif
#ifndef UART7_TX_DMA_STREAM
#define UART7_TX_DMA_STREAM NULL
#endif
#ifndef UART7_RX_DMA_STREAM
#define UART7_RX_DMA_STREAM NULL
#endif
#ifndef UART8_TX_DMA_STREAM
#define UART8_TX_DMA_STREAM NULL
#endif
#ifndef UART8_RX_DMA_STREAM
#define UART8_RX_DMA_STREAM NULL
#endif

#define UART_BUFFERS(n) \
  DMA_RAM volatile uint8_t uart ## n ## RxBuffer[UART_RX_BUFFER_SIZE]; \
  DMA_RAM volatile uint8_t uart ## n ## TxBuffer[UART_TX_BUFFER_SIZE]; struct dummy_s

#ifdef USE_UART1
UART_BUFFERS(1);
#endif

#ifdef USE_UART2
UART_BUFFERS(2);
#endif

#ifdef USE_UART3
UART_BUFFERS(3);
#endif

#ifdef USE_UART4
UART_BUFFERS(4);
#endif

#ifdef USE_UART5
UART_BUFFERS(5);
#endif

#ifdef USE_UART6
UART_BUFFERS(6);
#endif

#ifdef USE_UART7
UART_BUFFERS(7);
#endif

#ifdef USE_UART8
UART_BUFFERS(8);
#endif

#undef UART_BUFFERS

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_USART1_RX,
        .rxDMAStream = UART1_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_USART1_TX,
        .txDMAStream = UART1_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA10), GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB7),  GPIO_AF4_USART1 },
            { DEFIO_TAG_E(PB15), GPIO_AF4_USART1 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA9),  GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB6),  GPIO_AF4_USART1 },
            { DEFIO_TAG_E(PB14), GPIO_AF4_USART1 },
        },
        .rcc_apb2 = RCC_APB2(USART1),
        .rxIrq = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif

#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = USART2,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_USART2_RX,
        .rxDMAStream = UART2_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_USART2_TX,
        .txDMAStream = UART2_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA3), GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD6), GPIO_AF7_USART2 }
        },
        .txPins = {
            { DEFIO_TAG_E(PA2), GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD5), GPIO_AF7_USART2 }
        },
        .rcc_apb1 = RCC_APB1L(USART2),
        .rxIrq = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif

#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_USART3_RX,
        .rxDMAStream = UART3_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_USART3_TX,
        .txDMAStream = UART3_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB11), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC11), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD9), GPIO_AF7_USART3 }
        },
        .txPins = {
            { DEFIO_TAG_E(PB10), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC10), GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD8), GPIO_AF7_USART3 }
        },
        .rcc_apb1 = RCC_APB1L(USART3),
        .rxIrq = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif

#ifdef USE_UART4
    {
        .device = UARTDEV_4,
        .reg = UART4,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_UART4_RX,
        .rxDMAStream = UART4_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_UART4_TX,
        .txDMAStream = UART4_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA1),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PA11), GPIO_AF6_UART4 },
            { DEFIO_TAG_E(PB8),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC11), GPIO_AF8_UART4 },         
            { DEFIO_TAG_E(PD0),  GPIO_AF8_UART4 }
        },
        .txPins = {
            { DEFIO_TAG_E(PA0),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PA12), GPIO_AF6_UART4 },
            { DEFIO_TAG_E(PB9),  GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC10), GPIO_AF8_UART4 },         
            { DEFIO_TAG_E(PD1),  GPIO_AF8_UART4 }
        },
        .rcc_apb1 = RCC_APB1L(UART4),
        .rxIrq = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4,
        .txBuffer = uart4TxBuffer,
        .rxBuffer = uart4RxBuffer,
        .txBufferSize = sizeof(uart4TxBuffer),
        .rxBufferSize = sizeof(uart4RxBuffer),
    },
#endif

#ifdef USE_UART5
    {
        .device = UARTDEV_5,
        .reg = UART5,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_UART5_RX,
        .rxDMAStream = UART5_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_UART5_TX,
        .txDMAStream = UART5_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB5),  GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PB12), GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PD2),  GPIO_AF8_UART5 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB6),  GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PB13), GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PC12), GPIO_AF8_UART5 },
        },
        .rcc_apb1 = RCC_APB1L(UART5),
        .rxIrq = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif

#ifdef USE_UART6
    {
        .device = UARTDEV_6,
        .reg = USART6,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_USART6_RX,
        .rxDMAStream = UART6_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_USART6_TX,
        .txDMAStream = UART6_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PC7), GPIO_AF7_USART6  },
            { DEFIO_TAG_E(PG9), GPIO_AF7_USART6 }
        },
        .txPins = {
            { DEFIO_TAG_E(PC6), GPIO_AF7_USART6 },
            { DEFIO_TAG_E(PG14), GPIO_AF7_USART6 }
        },
        .rcc_apb2 = RCC_APB2(USART6),
        .rxIrq = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif

#ifdef USE_UART7
    {
        .device = UARTDEV_7,
        .reg = UART7,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_UART7_RX,
        .rxDMAStream = UART7_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_UART7_TX,
        .txDMAStream = UART7_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA8), GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PB3), GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PE7), GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PF6), GPIO_AF7_UART7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA15), GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PB4),  GPIO_AF11_UART7 },
            { DEFIO_TAG_E(PE8),  GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PF7),  GPIO_AF7_UART7 },
        },
        .rcc_apb1 = RCC_APB1L(UART7),
        .rxIrq = UART7_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART7,
        .txBuffer = uart7TxBuffer,
        .rxBuffer = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif

#ifdef USE_UART8
    {
        .device = UARTDEV_8,
        .reg = UART8,
#ifdef USE_DMA
        .rxDMARequest = DMA_REQUEST_UART8_RX,
        .rxDMAStream = UART8_RX_DMA_STREAM,
        .txDMARequest = DMA_REQUEST_UART8_TX,
        .txDMAStream = UART8_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PE0), GPIO_AF8_UART8 }
        },
        .txPins = {
            { DEFIO_TAG_E(PE1), GPIO_AF8_UART8 }
        },
        .rcc_apb1 = RCC_APB1L(UART8),
        .rxIrq = UART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    },
#endif
};

#ifdef USE_DMA
static void handleUsartTxDma(uartPort_t *s);
#endif

void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t) 0xff);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    /* UART in mode Transmitter ------------------------------------------------*/
    if (
#ifdef USE_DMA
        !s->txDMAStream &&
#endif
        (__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET)) {
        /* Check that a Tx process is ongoing */
        if (huart->gState != HAL_UART_STATE_BUSY_TX) {
            if (s->port.txBufferTail == s->port.txBufferHead) {
                huart->TxXferCount = 0;
                /* Disable the UART Transmit Data Register Empty Interrupt */
                CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
            } else {
                if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE)) {
                    huart->Instance->TDR = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
                } else {
                    huart->Instance->TDR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail]);
                }
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            }
        }
    }

    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        HAL_UART_IRQHandler(huart);
#ifdef USE_DMA
        if (s->txDMAStream) {
            handleUsartTxDma(s);
        }
#endif
    }
}

#ifdef USE_DMA
static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
}

static void dmaIRQHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    HAL_DMA_IRQHandler(&s->txDMAHandle);
}
#endif

// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartdev = uartDevmap[device];
    if (!uartdev) {
        return NULL;
    }

    uartPort_t *s = &(uartdev->port);

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    const uartHardware_t *hardware = uartdev->hardware;

    s->USARTx = hardware->reg;

#ifdef STM32H7
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
#endif

#ifdef USE_DMA
    if (hardware->rxDMAStream) {
        s->rxDMAStream = hardware->rxDMAStream;
#if defined(STM32H7)
        s->rxDMARequest = hardware->rxDMARequest;
#else // F4 & F7
        s->rxDMAChannel = hardware->DMAChannel;
#endif
    }

    if (hardware->txDMAStream) {
        s->txDMAStream = hardware->txDMAStream;
#if defined(STM32H7)
        s->txDMARequest = hardware->txDMARequest;
#else // F4 & F7
        s->txDMAChannel = hardware->DMAChannel;
#endif

        // DMA TX Interrupt
        dmaIdentifier_e identifier = dmaGetIdentifier(hardware->txDMAStream);
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(identifier, dmaIRQHandler, hardware->txPriority, (uint32_t)uartdev);
    }

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;
#endif

    s->Handle.Instance = hardware->reg;

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    if ((options & SERIAL_BIDIR) && txIO) {
        ioConfig_t ioCfg = IO_CONFIG(
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_MODE_AF_PP : GPIO_MODE_AF_OD,
            GPIO_SPEED_FREQ_HIGH,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_PULLDOWN : GPIO_PULLUP
        );

        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, ioCfg, uartdev->tx.af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uartdev->tx.af);
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP, uartdev->rx.af);
        }
    }

#ifdef USE_DMA
#if defined(STM32H7)
    if (!s->rxDMAStream)
#else
    if (!s->rxDMAChannel)
#endif
#endif
    {
        HAL_NVIC_SetPriority(hardware->rxIrq, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));
        HAL_NVIC_EnableIRQ(hardware->rxIrq);
    }

    return s;
}
#endif // USE_UART
