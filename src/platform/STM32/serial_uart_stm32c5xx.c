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

#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

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

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_USART1,
        .reg = (usartResource_t *)USART1,
#ifdef USE_DMA
        .rxDMAChannel = HAL_LPDMA1_REQUEST_USART1_RX,
        .rxDMAResource = (dmaResource_t *)UART1_RX_DMA_STREAM,
        .txDMAChannel = HAL_LPDMA1_REQUEST_USART1_TX,
        .txDMAResource = (dmaResource_t *)UART1_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA10), HAL_GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB7),  HAL_GPIO_AF7_USART1 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA9),  HAL_GPIO_AF7_USART1 },
            { DEFIO_TAG_E(PB6),  HAL_GPIO_AF7_USART1 },
        },
        .rcc = RCC_APB2(USART1),
        .irqn = USART1_IRQn,
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
        .identifier = SERIAL_PORT_USART2,
        .reg = (usartResource_t *)USART2,
#ifdef USE_DMA
        .rxDMAChannel = HAL_LPDMA1_REQUEST_USART2_RX,
        .rxDMAResource = (dmaResource_t *)UART2_RX_DMA_STREAM,
        .txDMAChannel = HAL_LPDMA1_REQUEST_USART2_TX,
        .txDMAResource = (dmaResource_t *)UART2_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA3), HAL_GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD6), HAL_GPIO_AF7_USART2 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA2), HAL_GPIO_AF7_USART2 },
            { DEFIO_TAG_E(PD5), HAL_GPIO_AF7_USART2 },
        },
        .rcc = RCC_APB1L(USART2),
        .irqn = USART2_IRQn,
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
        .identifier = SERIAL_PORT_USART3,
        .reg = (usartResource_t *)USART3,
#ifdef USE_DMA
        .rxDMAChannel = HAL_LPDMA1_REQUEST_USART3_RX,
        .rxDMAResource = (dmaResource_t *)UART3_RX_DMA_STREAM,
        .txDMAChannel = HAL_LPDMA1_REQUEST_USART3_TX,
        .txDMAResource = (dmaResource_t *)UART3_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB11), HAL_GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC11), HAL_GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD9),  HAL_GPIO_AF7_USART3 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB10), HAL_GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PC10), HAL_GPIO_AF7_USART3 },
            { DEFIO_TAG_E(PD8),  HAL_GPIO_AF7_USART3 },
        },
        .rcc = RCC_APB1L(USART3),
        .irqn = USART3_IRQn,
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
        .identifier = SERIAL_PORT_UART4,
        .reg = (usartResource_t *)UART4,
#ifdef USE_DMA
        .rxDMAChannel = HAL_LPDMA1_REQUEST_UART4_RX,
        .rxDMAResource = (dmaResource_t *)UART4_RX_DMA_STREAM,
        .txDMAChannel = HAL_LPDMA1_REQUEST_UART4_TX,
        .txDMAResource = (dmaResource_t *)UART4_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA1),  HAL_GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC11), HAL_GPIO_AF8_UART4 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA0),  HAL_GPIO_AF8_UART4 },
            { DEFIO_TAG_E(PC10), HAL_GPIO_AF8_UART4 },
        },
        .rcc = RCC_APB1L(UART4),
        .irqn = UART4_IRQn,
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
        .identifier = SERIAL_PORT_UART5,
        .reg = (usartResource_t *)UART5,
#ifdef USE_DMA
        .rxDMAChannel = HAL_LPDMA1_REQUEST_UART5_RX,
        .rxDMAResource = (dmaResource_t *)UART5_RX_DMA_STREAM,
        .txDMAChannel = HAL_LPDMA1_REQUEST_UART5_TX,
        .txDMAResource = (dmaResource_t *)UART5_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB5),  HAL_GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PD2),  HAL_GPIO_AF8_UART5 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB6),  HAL_GPIO_AF14_UART5 },
            { DEFIO_TAG_E(PC12), HAL_GPIO_AF8_UART5 },
        },
        .rcc = RCC_APB1L(UART5),
        .irqn = UART5_IRQn,
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
        .identifier = SERIAL_PORT_USART6,
        .reg = (usartResource_t *)USART6,
#ifdef USE_DMA
        .rxDMAChannel = HAL_LPDMA1_REQUEST_USART6_RX,
        .rxDMAResource = (dmaResource_t *)UART6_RX_DMA_STREAM,
        .txDMAChannel = HAL_LPDMA1_REQUEST_USART6_TX,
        .txDMAResource = (dmaResource_t *)UART6_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PC7), HAL_GPIO_AF7_USART6 },
        },
        .txPins = {
            { DEFIO_TAG_E(PC6), HAL_GPIO_AF7_USART6 },
        },
        .rcc = RCC_APB1L(USART6),
        .irqn = USART6_IRQn,
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
        .identifier = SERIAL_PORT_USART7,
        .reg = (usartResource_t *)UART7,
#ifdef USE_DMA
        .rxDMAChannel = HAL_LPDMA1_REQUEST_UART7_RX,
        .rxDMAResource = (dmaResource_t *)UART7_RX_DMA_STREAM,
        .txDMAChannel = HAL_LPDMA1_REQUEST_UART7_TX,
        .txDMAResource = (dmaResource_t *)UART7_TX_DMA_STREAM,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA8), HAL_GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PB3), HAL_GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PF6), HAL_GPIO_AF7_UART7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA15), HAL_GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PB4),  HAL_GPIO_AF7_UART7 },
            { DEFIO_TAG_E(PF7),  HAL_GPIO_AF7_UART7 },
        },
        .rcc = RCC_APB1L(UART7),
        .irqn = UART7_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART7,
        .txBuffer = uart7TxBuffer,
        .rxBuffer = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif

};

#endif // USE_UART
