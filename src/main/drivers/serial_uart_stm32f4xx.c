/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "build/debug.h"

#include "platform.h"

#include "system.h"
#include "io.h"
#include "rcc.h"
#include "nvic.h"
#include "dma.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

#define UART_RX_BUFFER_SIZE 512
#define UART_TX_BUFFER_SIZE 512

typedef enum UARTDevice {
    UARTDEV_1 = 0,
    UARTDEV_2 = 1,
    UARTDEV_3 = 2,
    UARTDEV_4 = 3,
    UARTDEV_5 = 4,
    UARTDEV_6 = 5
} UARTDevice;

typedef struct uartPinPair_s {
    ioTag_t rx;
    ioTag_t tx;
} uartPinPair_t;

typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    uint32_t DMAChannel;
    DMA_Stream_TypeDef *txDMAStream;
    DMA_Stream_TypeDef *rxDMAStream;
    uartPinPair_t pinPair[3];
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE]; // XXX Waste if not configured...
    volatile uint8_t txBuffer[UART_TX_BUFFER_SIZE]; // XXX Ditto
    rccPeriphTag_t rcc;
    uint8_t af;
    uint8_t rxIrq;
    uint32_t txPriority;
    uint32_t rxPriority;
} uartDevice_t;

#ifdef USE_UART1_RX_DMA
# define UART1_RX_DMA_STREAM DMA2_Stream5
#else
# define UART1_RX_DMA_STREAM 0
#endif
#ifdef USE_UART1_TX_DMA
# define UART1_TX_DMA_STREAM DMA2_Stream7
#else
# define UART1_TX_DMA_STREAM 0
#endif

#ifdef USE_UART2_RX_DMA
# define UART2_RX_DMA_STREAM DMA1_Stream5
#else
# define UART2_RX_DMA_STREAM 0
#endif
#ifdef USE_UART2_TX_DMA
# define UART2_TX_DMA_STREAM DMA1_Stream6
#else
# define UART2_TX_DMA_STREAM 0
#endif

#ifdef USE_UART3_RX_DMA
# define UART3_RX_DMA_STREAM DMA1_Stream1
#else
# define UART3_RX_DMA_STREAM 0
#endif
#ifdef USE_UART3_TX_DMA
# define UART3_TX_DMA_STREAM DMA1_Stream3
#else
# define UART3_TX_DMA_STREAM 0
#endif

#ifdef USE_UART4_RX_DMA
# define UART4_RX_DMA_STREAM DMA1_Stream2
#else
# define UART4_RX_DMA_STREAM 0
#endif
#ifdef USE_UART4_TX_DMA
# define UART4_TX_DMA_STREAM DMA1_Stream4
#else
# define UART4_TX_DMA_STREAM 0
#endif

#ifdef USE_UART5_RX_DMA
# define UART5_RX_DMA_STREAM DMA1_Stream0
#else
# define UART5_RX_DMA_STREAM 0
#endif
#ifdef USE_UART5_TX_DMA
# define UART5_TX_DMA_STREAM DMA1_Stream7
#else
# define UART5_TX_DMA_STREAM 0
#endif

#ifdef USE_UART6_RX_DMA
# define UART6_RX_DMA_STREAM DMA2_Stream1
#else
# define UART6_RX_DMA_STREAM 0
#endif
#ifdef USE_UART6_TX_DMA
# define UART6_TX_DMA_STREAM DMA2_Stream6
#else
# define UART6_TX_DMA_STREAM 0
#endif

static uartDevice_t uartHardware[] = {
    {
        .dev = USART1,
        .DMAChannel = DMA_Channel_4,
        .rxDMAStream = UART1_RX_DMA_STREAM,
        .txDMAStream = UART1_TX_DMA_STREAM,
        .pinPair = {
            { IO_TAG(PA10), IO_TAG(PA9) },
            { IO_TAG(PB7), IO_TAG(PB6) },
            { IO_TAG_NONE, IO_TAG_NONE }
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .af = GPIO_AF_USART1,
        .rcc = RCC_APB2(USART1),
        .rxIrq = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1
    },
    {
        .dev = USART2,
        .DMAChannel = DMA_Channel_4,
        .rxDMAStream = UART2_RX_DMA_STREAM,
        .txDMAStream = UART2_TX_DMA_STREAM,
        .pinPair = {
            { IO_TAG(PA3), IO_TAG(PA2) },
            { IO_TAG(PD6), IO_TAG(PD5) },
            { IO_TAG_NONE, IO_TAG_NONE }
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .af = GPIO_AF_USART2,
        .rcc = RCC_APB1(USART2),
        .rxIrq = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2
    },
    {
        .dev = USART3,
        .DMAChannel = DMA_Channel_4,
        .rxDMAStream = UART3_RX_DMA_STREAM,
        .txDMAStream = UART3_TX_DMA_STREAM,
        .pinPair = {
            { IO_TAG(PB11), IO_TAG(PB10) },
            { IO_TAG(PC11), IO_TAG(PC10) },
            { IO_TAG(PD9), IO_TAG(PD8) }
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .af = GPIO_AF_USART3,
        .rcc = RCC_APB1(USART3),
        .rxIrq = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3
    },
    {
        .dev = UART4,
        .DMAChannel = DMA_Channel_4,
        .rxDMAStream = UART4_RX_DMA_STREAM,
        .txDMAStream = UART4_TX_DMA_STREAM,
        .pinPair = {
            { IO_TAG(PA1), IO_TAG(PA0) },
            { IO_TAG(PC11), IO_TAG(PC10) },
            { IO_TAG_NONE, IO_TAG_NONE }
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .af = GPIO_AF_UART4,
        .rcc = RCC_APB1(UART4),
        .rxIrq = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4
    },
    {
        .dev = UART5,
        .DMAChannel = DMA_Channel_4,
        .rxDMAStream = UART5_RX_DMA_STREAM,
        .txDMAStream = UART5_TX_DMA_STREAM,
        .pinPair = {
            { IO_TAG(PD2), IO_TAG(PC12) },
            { IO_TAG_NONE, IO_TAG_NONE },
            { IO_TAG_NONE, IO_TAG_NONE }
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .af = GPIO_AF_UART5,
        .rcc = RCC_APB1(UART5),
        .rxIrq = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5
    },
    {
        .dev = USART6,
        .DMAChannel = DMA_Channel_5,
        .rxDMAStream = UART6_RX_DMA_STREAM,
        .txDMAStream = UART6_TX_DMA_STREAM,
        .pinPair = {
            { IO_TAG(PC7), IO_TAG(PC6) },
#if ((TARGET_IO_PORTG & BIT(9)) && (TARGET_IO_PORTG & BIT(14)))
            { IO_TAG(PG9), IO_TAG(PG14) },
#else
            { IO_TAG_NONE, IO_TAG_NONE },
#endif
            { IO_TAG_NONE, IO_TAG_NONE }
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .af = GPIO_AF_USART6,
        .rcc = RCC_APB2(USART6),
        .rxIrq = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6
    },
};

static uartDevice_t* uartHardwareMap[6];

void serialInitHardwareMap(serialPinConfig_t *pSerialPinConfig)
{
    memset(uartHardwareMap, 0, ARRAYLEN(uartHardwareMap));

    for (size_t index = 0 ; index < ARRAYLEN(uartHardware) ; index++) {
        uartDevice_t *uartDev = &uartHardware[index];
        for (int pair = 0 ; pair < 3 ; pair++) {

            if (uartDev->pinPair[pair].rx == pSerialPinConfig->ioTagRx[index]
                    && uartDev->pinPair[pair].tx == pSerialPinConfig->ioTagTx[index]) {
                uartDev->rx = uartDev->pinPair[pair].rx;
                uartDev->tx = uartDev->pinPair[pair].tx;
                uartHardwareMap[index] = uartDev;

                break;
            }
        }
    }
}

void uartIrqHandler(uartPort_t *s)
{
    if (!s->rxDMAStream && (USART_GetITStatus(s->USARTx, USART_IT_RXNE) == SET)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->DR);
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

    if (USART_GetITStatus(s->USARTx, USART_FLAG_ORE) == SET)
    {
        USART_ClearITPendingBit (s->USARTx, USART_IT_ORE);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    DMA_Cmd(s->txDMAStream, DISABLE);

    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
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

uartPort_t *serialUART(UARTDevice device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    NVIC_InitTypeDef NVIC_InitStructure;

    uartDevice_t *uart = uartHardwareMap[device];
    if (!uart) return NULL;

    s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uart->rxBuffer;
    s->port.txBuffer = uart->txBuffer;
    s->port.rxBufferSize = sizeof(uart->rxBuffer);
    s->port.txBufferSize = sizeof(uart->txBuffer);

    s->USARTx = uart->dev;
    if (uart->rxDMAStream) {
        s->rxDMAChannel = uart->DMAChannel;
        s->rxDMAStream = uart->rxDMAStream;
        dmaInit(dmaGetIdentifier(uart->rxDMAStream), OWNER_SERIAL_RX, RESOURCE_INDEX(device));
    }
    if (uart->txDMAStream) {
        s->txDMAChannel = uart->DMAChannel;
        s->txDMAStream = uart->txDMAStream;
        const dmaIdentifier_e identifier = dmaGetIdentifier(uart->txDMAStream);
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        // DMA TX Interrupt
        dmaSetHandler(identifier, dmaIRQHandler, uart->txPriority, (uint32_t)uart);
    }

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

    IO_t tx = IOGetByTag(uart->tx);
    IO_t rx = IOGetByTag(uart->rx);

    if (uart->rcc) {
        RCC_ClockCmd(uart->rcc, ENABLE);
    }

    if (options & SERIAL_BIDIR) {
        IOInit(tx, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        if (options & SERIAL_BIDIR_PP)
            IOConfigGPIOAF(tx, IOCFG_AF_PP, uart->af);
        else
            IOConfigGPIOAF(tx, IOCFG_AF_OD, uart->af);
    }
    else {
        if (mode & MODE_TX) {
            IOInit(tx, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(tx, IOCFG_AF_PP_UP, uart->af);
        }

        if (mode & MODE_RX) {
            IOInit(rx, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rx, IOCFG_AF_PP_UP, uart->af);
        }
    }

    if (!(s->rxDMAChannel)) {
        NVIC_InitStructure.NVIC_IRQChannel = uart->rxIrq;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(uart->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(uart->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    return s;
}

#ifdef USE_UART1
uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_1, baudRate, mode, options);
}

// USART1 Rx/Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_1]->port);
    uartIrqHandler(s);
}

#endif

#ifdef USE_UART2
// USART2 - GPS or Spektrum or ?? (RX + TX by IRQ)
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_2, baudRate, mode, options);
}

void USART2_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_2]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART3
// USART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_3, baudRate, mode, options);
}

void USART3_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_3]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART4
// USART4
uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_4, baudRate, mode, options);
}

void UART4_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_4]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART5
// USART5
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_5, baudRate, mode, options);
}

void UART5_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_5]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART6
// USART6
uartPort_t *serialUART6(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_6, baudRate, mode, options);
}

void USART6_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_6]->port);
    uartIrqHandler(s);
}
#endif
