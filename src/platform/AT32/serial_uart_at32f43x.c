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
 *
 * Author: jflyper
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

#ifndef UART1_TX_DMA_CHANNEL
#define UART1_TX_DMA_CHANNEL NULL
#endif
#ifndef UART1_RX_DMA_CHANNEL
#define UART1_RX_DMA_CHANNEL NULL
#endif
#ifndef UART2_TX_DMA_CHANNEL
#define UART2_TX_DMA_CHANNEL NULL
#endif
#ifndef UART2_RX_DMA_CHANNEL
#define UART2_RX_DMA_CHANNEL NULL
#endif
#ifndef UART3_TX_DMA_CHANNEL
#define UART3_TX_DMA_CHANNEL NULL
#endif
#ifndef UART3_RX_DMA_CHANNEL
#define UART3_RX_DMA_CHANNEL NULL
#endif
#ifndef UART4_TX_DMA_CHANNEL
#define UART4_TX_DMA_CHANNEL NULL
#endif
#ifndef UART4_RX_DMA_CHANNEL
#define UART4_RX_DMA_CHANNEL NULL
#endif
#ifndef UART5_TX_DMA_CHANNEL
#define UART5_TX_DMA_CHANNEL NULL
#endif
#ifndef UART5_RX_DMA_CHANNEL
#define UART5_RX_DMA_CHANNEL NULL
#endif
#ifndef UART6_RX_DMA_CHANNEL
#define UART6_RX_DMA_CHANNEL NULL
#endif
#ifndef UART6_TX_DMA_CHANNEL
#define UART6_TX_DMA_CHANNEL NULL
#endif
#ifndef UART7_RX_DMA_CHANNEL
#define UART7_RX_DMA_CHANNEL NULL
#endif
#ifndef UART7_TX_DMA_CHANNEL
#define UART7_TX_DMA_CHANNEL NULL
#endif
#ifndef UART8_TX_DMA_CHANNEL
#define UART8_TX_DMA_CHANNEL NULL
#endif
#ifndef UART8_RX_DMA_CHANNEL
#define UART8_RX_DMA_CHANNEL NULL
#endif

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_USART1,
        .reg = USART1,
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_USART1_RX,
        .rxDMAResource = (dmaResource_t *)UART1_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_USART1_TX,
        .txDMAResource = (dmaResource_t *)UART1_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA10), GPIO_MUX_7 },
            { DEFIO_TAG_E(PB7),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PB3),  GPIO_MUX_7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA9),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PA15), GPIO_MUX_7 },
            { DEFIO_TAG_E(PB6),  GPIO_MUX_7 },
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
        .reg = USART2,
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_USART2_RX,
        .rxDMAResource = (dmaResource_t *)UART2_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_USART2_TX,
        .txDMAResource = (dmaResource_t *)UART2_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA3),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PA15), GPIO_MUX_8 },
            { DEFIO_TAG_E(PB0),  GPIO_MUX_6 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA2),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PA8),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PA14), GPIO_MUX_8 },
        },
        .rcc = RCC_APB1(USART2),
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
        .reg = USART3,
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_USART3_RX,
        .rxDMAResource = (dmaResource_t *)UART3_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_USART3_TX,
        .txDMAResource = (dmaResource_t *)UART3_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB11), GPIO_MUX_7 },
            { DEFIO_TAG_E(PC5),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PC11), GPIO_MUX_7 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB10), GPIO_MUX_7 },
            { DEFIO_TAG_E(PC4),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PC10), GPIO_MUX_7 },
        },
        .rcc = RCC_APB1(USART3),
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
        .reg = UART4,
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_UART4_RX,
        .rxDMAResource = (dmaResource_t *)UART4_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_UART4_TX,
        .txDMAResource = (dmaResource_t *)UART4_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA1),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PC11), GPIO_MUX_8 },
            { DEFIO_TAG_E(PH2),  GPIO_MUX_8 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA0),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PC10), GPIO_MUX_8 },
            { DEFIO_TAG_E(PH3),  GPIO_MUX_8 },
        },
        .rcc = RCC_APB1(UART4),
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
        .reg = UART5,
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_UART5_RX,
        .rxDMAResource = (dmaResource_t *)UART5_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_UART5_TX,
        .txDMAResource = (dmaResource_t *)UART5_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB5),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PB8),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PD2),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PE11), GPIO_MUX_8 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB6),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PB9),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PC12), GPIO_MUX_8 },
            { DEFIO_TAG_E(PE10), GPIO_MUX_8 },
        },
        .rcc = RCC_APB1(UART5),
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
        .reg = USART6,
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_USART6_RX,
        .rxDMAResource = (dmaResource_t *)UART6_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_USART6_RX,
        .txDMAResource = (dmaResource_t *)UART6_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PA5),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PA12), GPIO_MUX_8 },
            { DEFIO_TAG_E(PC7),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PG9),  GPIO_MUX_8 },
        },
        .txPins = {
            { DEFIO_TAG_E(PA4),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PA11), GPIO_MUX_8 },
            { DEFIO_TAG_E(PC6),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PG4),  GPIO_MUX_8 },
        },
        .rcc = RCC_APB2(USART6),
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
        .identifier = SERIAL_PORT_UART7,
        .reg = UART7,
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_UART7_RX,
        .rxDMAResource = (dmaResource_t *)UART7_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_UART7_TX,
        .txDMAResource = (dmaResource_t *)UART7_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PB3),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PC1),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PE7),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PF6),  GPIO_MUX_8 },
        },
        .txPins = {
            { DEFIO_TAG_E(PB4),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PC0),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PE8),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PF7),  GPIO_MUX_8 },
        },
        .rcc = RCC_APB1(UART7),
        .irqn = UART7_IRQn,
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
        .identifier = SERIAL_PORT_UART8,
        .reg = UART8, //USE UART8 FOR PIN CONFIG
#ifdef USE_DMA
        .rxDMAMuxId = DMAMUX_DMAREQ_ID_UART8_RX,
        .rxDMAResource = (dmaResource_t *)UART8_RX_DMA_CHANNEL,
        .txDMAMuxId = DMAMUX_DMAREQ_ID_UART8_TX,
        .txDMAResource = (dmaResource_t *)UART8_TX_DMA_CHANNEL,
#endif
        .rxPins = {
            { DEFIO_TAG_E(PC3),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PC9),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PD14), GPIO_MUX_8 },
            { DEFIO_TAG_E(PE0),  GPIO_MUX_8 },
        },
        .txPins = {
            { DEFIO_TAG_E(PC2),  GPIO_MUX_8 },
            { DEFIO_TAG_E(PC8),  GPIO_MUX_7 },
            { DEFIO_TAG_E(PD13), GPIO_MUX_8 },
            { DEFIO_TAG_E(PE1),  GPIO_MUX_8 },
        },
        .rcc = RCC_APB1(UART8),
        .irqn = UART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    },
#endif

//TODO: ADD UART 6\7\8 HERE!

};

#endif // USE_UART
