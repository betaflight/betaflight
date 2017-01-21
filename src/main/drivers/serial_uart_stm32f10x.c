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

/*
 * Authors:
 * Dominic Clifton/Hydra - Various cleanups for Cleanflight
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>

#include "system.h"
#include "io.h"
#include "nvic.h"
#include "dma.h"
#include "rcc.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

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

#define UART_PINPAIR_COUNT 3

typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    DMA_Channel_TypeDef *txDMAChannel;
    DMA_Channel_TypeDef *rxDMAChannel;
    uartPinPair_t pinPair[UART_PINPAIR_COUNT];
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    rccPeriphTag_t rcc;
    uint8_t af;
    uint8_t irqn;
    uint32_t txPriority;
    uint32_t rxPriority;
} uartDevice_t;

#ifdef USE_UART1
//static uartPort_t uartPort1;
#endif

#ifdef USE_UART2
//static uartPort_t uartPort2;
#endif

#ifdef USE_UART3
static uartPort_t uartPort3;
#endif

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

static uartDevice_t uartHardware[] = {
    {
        .dev = USART1,
        .rxDMAChannel = UART1_RX_DMA_CHANNEL,
        .txDMAChannel = UART1_TX_DMA_CHANNEL,
        .pinPair = {
            { DEFIO_TAG_E(PA10), DEFIO_TAG_E(PA9) },
            { DEFIO_TAG_E(PB7), DEFIO_TAG_E(PB6) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        //.af = GPIO_AF_USART1,
        .rcc = RCC_APB2(USART1),
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1
    },
    {
        .dev = USART2,
        .rxDMAChannel = UART2_RX_DMA_CHANNEL,
        .txDMAChannel = UART2_TX_DMA_CHANNEL,
        .pinPair = {
            { DEFIO_TAG_E(PA3), DEFIO_TAG_E(PA2) },
            { DEFIO_TAG_E(PD6), DEFIO_TAG_E(PD5) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        //.af = GPIO_AF_USART2,
        .rcc = RCC_APB1(USART2),
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2,
        .rxPriority = NVIC_PRIO_SERIALUART2
    },
#ifdef USE_UART3 // Limitation by FLASH & RAM (Operation not tested)
    {
        .dev = USART3,
        .rxDMAChannel = UART3_RX_DMA_CHANNEL,
        .txDMAChannel = UART3_TX_DMA_CHANNEL,
        .pinPair = {
            { DEFIO_TAG_E(PB11), DEFIO_TAG_E(PB10) },
            { DEFIO_TAG_E(PD9), DEFIO_TAG_E(PD8) },
            { DEFIO_TAG_E(PC11), DEFIO_TAG_E(PC10) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        //.af = GPIO_AF_USART3,
        .rcc = RCC_APB1(USART3),
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3,
        .rxPriority = NVIC_PRIO_SERIALUART3
    },
#endif
};

static uartDevice_t *uartHardwareMap[ARRAYLEN(uartHardware)];

static volatile uint8_t rxBuffers[ARRAYLEN(uartHardware)][UART_RX_BUFFER_SIZE];
static volatile uint8_t txBuffers[ARRAYLEN(uartHardware)][UART_TX_BUFFER_SIZE];

void serialInitHardwareMap(serialPinConfig_t *pSerialPinConfig)
{
    memset(uartHardwareMap, 0, sizeof(uartHardwareMap));

    for (size_t index = 0 ; index < ARRAYLEN(uartHardware) ; index++) {
        uartDevice_t *uartDev = &uartHardware[index];
        for (int pair = 0 ; pair < UART_PINPAIR_COUNT ; pair++) {

            if (uartDev->pinPair[pair].rx == pSerialPinConfig->ioTagRx[index]
                    && uartDev->pinPair[pair].tx == pSerialPinConfig->ioTagTx[index]) {
                uartDev->rx = uartDev->pinPair[pair].rx;
                uartDev->tx = uartDev->pinPair[pair].tx;
                uartDev->rxBuffer = rxBuffers[index];
                uartDev->txBuffer = txBuffers[index];
                uartHardwareMap[index] = uartDev;

                break;
            }
        }
    }
}

void uartIrqCallback(uartPort_t *s)
{
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE && !s->rxDMAChannel) {
        // If we registered a callback, pass crap there
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->DR);
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

// Tx DMA Handler
void uart_tx_dma_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = (uartPort_t*)(descriptor->userParam);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    DMA_Cmd(descriptor->channel, DISABLE);

    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}

uartPort_t *serialUART(UARTDevice device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartDevice_t *uartDev = uartHardwareMap[device];
    if (!uartDev)
        return NULL;

    uartPort_t *s = &(uartDev->port);

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uartDev->rxBuffer;
    s->port.txBuffer = uartDev->txBuffer;
    s->port.rxBufferSize = UART_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART_TX_BUFFER_SIZE;

    s->USARTx = uartDev->dev;

    if (uartDev->rxDMAChannel) {
        dmaInit(dmaGetIdentifier(uartDev->rxDMAChannel), OWNER_SERIAL_RX, RESOURCE_INDEX(device));
        s->rxDMAChannel = uartDev->rxDMAChannel;
        s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    }

    RCC_ClockCmd(uartDev->rcc, ENABLE);

    if (uartDev->txDMAChannel) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(uartDev->txDMAChannel);
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(identifier, uart_tx_dma_IRQHandler, uartDev->txPriority, (uint32_t)s);
        s->txDMAChannel = uartDev->txDMAChannel;
        s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    }

    IO_t rxIO = IOGetByTag(uartDev->rx);
    IO_t txIO = IOGetByTag(uartDev->tx);

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
    if (!uartDev->rxDMAChannel || !uartDev->txDMAChannel) {
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = uartDev->irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(uartDev->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(uartDev->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    return s;
}

uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_1, baudRate, mode, options);
}

// USART1 Rx/Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartDevice_t *uartDev = uartHardwareMap[UARTDEV_1];
    uartIrqCallback(&(uartDev->port));
}

uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_2, baudRate, mode, options);
}

// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartDevice_t *uartDev = uartHardwareMap[UARTDEV_2];
    uartIrqCallback(&(uartDev->port));
}

#ifdef USE_UART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_2, baudRate, mode, options);
}

void USART3_IRQHandler(void)
{
    uartDevice_t *uartDev = uartHardwareMap[UARTDEV_3];
    uartIrqCallback(&(uartDev->port));
}
#endif
