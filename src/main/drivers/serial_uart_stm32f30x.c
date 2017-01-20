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
 * Dominic Clifton - Port baseflight STM32F10x to STM32F30x for cleanflight
 * J. Ihlein - Code from FocusFlight32
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>

#include "build/debug.h"

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
} UARTDevice;

typedef struct uartPinPair_s {
    ioTag_t rx;
    ioTag_t tx;
} uartPinPair_t;

#define UART_PINPAIR_COUNT 4

typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;
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

// XXX Will DMA eventually be configurable?
// XXX Do these belong here?

#ifdef USE_UART1_RX_DMA
# define UART1_RX_DMA DMA1_Channel5
#else
# define UART1_RX_DMA 0
#endif

#ifdef USE_UART1_TX_DMA
# define UART1_TX_DMA DMA1_Channel4
#else
# define UART1_TX_DMA 0
#endif

#ifdef USE_UART2_RX_DMA
# define UART2_RX_DMA DMA1_Channel6
#else
# define UART2_RX_DMA 0
#endif

#ifdef USE_UART2_TX_DMA
# define UART2_TX_DMA DMA1_Channel7
#else
# define UART2_TX_DMA 0
#endif

#ifdef USE_UART3_RX_DMA
# define UART3_RX_DMA DMA1_Channel3
#else
# define UART3_RX_DMA 0
#endif

#ifdef USE_UART3_TX_DMA
# define UART3_TX_DMA DMA1_Channel2
#else
# define UART3_TX_DMA 0
#endif

#ifdef USE_UART4_RX_DMA
# define UART4_RX_DMA DMA2_Channel3
#else
# define UART4_RX_DMA 0
#endif

#ifdef USE_UART4_TX_DMA
# define UART4_TX_DMA DMA2_Channel5
#else
# define UART4_TX_DMA 0
#endif

#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

static uartDevice_t uartHardware[] = {
    // USART1
    // PA10,PA9
    // PB7,PB6
    // PE1,PE0
    // PC5,PC4
    {
        .dev = USART1,
        .rxDMAChannel = UART1_RX_DMA,
        .txDMAChannel = UART1_TX_DMA,
        .pinPair = {
            { DEFIO_TAG_E(PA10), DEFIO_TAG_E(PA9) },
            { DEFIO_TAG_E(PB7),  DEFIO_TAG_E(PB6) },
            { DEFIO_TAG_E(PC5),  DEFIO_TAG_E(PC4) },
            { DEFIO_TAG_E(PE1),  DEFIO_TAG_E(PE0) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .rcc = RCC_APB2(USART1),
        .af = GPIO_AF_7,
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1_RXDMA,
    },

    // USART2
    // PA3, PA2
    // PA15, PA14
    // PD6, PD5
    // PB4, PB3
    {
        .dev = USART2,
        .rxDMAChannel = UART2_RX_DMA,
        .txDMAChannel = UART2_TX_DMA,
        .pinPair = {
            { DEFIO_TAG_E(PA15), DEFIO_TAG_E(PA14) },
            { DEFIO_TAG_E(PA3),  DEFIO_TAG_E(PA2) },
            { DEFIO_TAG_E(PB4),  DEFIO_TAG_E(PB3) },
            { DEFIO_TAG_E(PD6),  DEFIO_TAG_E(PD5) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .rcc = RCC_APB1(USART2),
        .af = GPIO_AF_7,
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2_RXDMA,
    },

    // USART3
    // PB11,PB10
    // PC11,PC10
    // PD9,PD8
    {
        .dev = USART3,
        .rxDMAChannel = UART3_RX_DMA,
        .txDMAChannel = UART3_TX_DMA,
        .pinPair = {
            { DEFIO_TAG_E(PB11), DEFIO_TAG_E(PB10) },
            { DEFIO_TAG_E(PC11),  DEFIO_TAG_E(PC10) },
            { DEFIO_TAG_E(PD9),  DEFIO_TAG_E(PD8) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .rcc = RCC_APB1(USART3),
        .af = GPIO_AF_7,
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3_RXDMA,
    },

    // UART4 XXX Not tested (yet!?) Need 303RC, e.g. LUX for testing
    // PC11,PC10
    {
        .dev = UART4,
        .rxDMAChannel = UART4_RX_DMA,
        .txDMAChannel = UART4_TX_DMA,
        .pinPair = {
            { DEFIO_TAG_E(PC11), DEFIO_TAG_E(PC10) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .rcc = RCC_APB1(UART4),
        .af = GPIO_AF_5,
        .irqn = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4_RXDMA,
    },

    // UART5 XXX Not tested (yet!?) Need 303RC; e.g. LUX for testing
    // PD2,PC12
    {
        .dev = UART5,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
        .pinPair = {
            { DEFIO_TAG_E(PD2), DEFIO_TAG_E(PC12) },
        },
        .rx = IO_TAG_NONE,
        .tx = IO_TAG_NONE,
        .rcc = RCC_APB1(UART5),
        .af = GPIO_AF_5,
        .irqn = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5,
        .rxPriority = NVIC_PRIO_SERIALUART5,
    },
};

static uartDevice_t *uartHardwareMap[ARRAYLEN(uartHardware)];

// XXX What a waste...
// XXX These could be left inside the serialUARTx() function,
// XXX but they will eventually go away...
// XXX ARRAYLEN(uartHardware) is overkill; What was the number (count) of UART serial ports???

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

void usartIrqHandler(uartPort_t *s)
{
    uint32_t ISR = s->USARTx->ISR;

    if (!s->rxDMAChannel && (ISR & USART_FLAG_RXNE)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->RDR);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead++] = s->USARTx->RDR;
            if (s->port.rxBufferHead >= s->port.rxBufferSize) {
                s->port.rxBufferHead = 0;
            }
        }
    }

    if (!s->txDMAChannel && (ISR & USART_FLAG_TXE)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail++]);
            if (s->port.txBufferTail >= s->port.txBufferSize) {
                s->port.txBufferTail = 0;
            }
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }

    if (ISR & USART_FLAG_ORE)
    {
        USART_ClearITPendingBit (s->USARTx, USART_IT_ORE);
    }
}

#if defined(USE_UART1_TX_DMA) || defined(USE_UART2_TX_DMA) || defined(USE_UART3_TX_DMA)
static void handleUsartTxDma(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = (uartPort_t*)(descriptor->userParam);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    DMA_Cmd(descriptor->channel, DISABLE);

    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}
#endif

void serialUARTInitIO(IO_t tx, IO_t rx, portMode_t mode, portOptions_t options, uint8_t af, uint8_t index)
{
    if (options & SERIAL_BIDIR) {
        ioConfig_t ioCfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_OType_PP : GPIO_OType_OD,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP
        );

        IOInit(tx, OWNER_SERIAL_TX, index);
        IOConfigGPIOAF(tx, ioCfg, af);

        if (!(options & SERIAL_INVERTED))
            IOLo(tx);   // OpenDrain output should be inactive
    } else {
        ioConfig_t ioCfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, (options & SERIAL_INVERTED) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP);
        if (mode & MODE_TX) {
            IOInit(tx, OWNER_SERIAL_TX, index);
            IOConfigGPIOAF(tx, ioCfg, af);
        }

        if (mode & MODE_RX) {
            IOInit(rx, OWNER_SERIAL_RX, index);
            IOConfigGPIOAF(rx, ioCfg, af);
        }
    }
}

uartPort_t *serialUART(int device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartDevice_t *uartDev = uartHardwareMap[device];
    if (!uartDev)
        return NULL;

    uartPort_t *s = &(uartDev->port);

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uartDev->rxBuffer;
    s->port.txBuffer = uartDev->txBuffer;
    s->port.rxBufferSize = sizeof(uartDev->rxBuffer);
    s->port.txBufferSize = sizeof(uartDev->txBuffer);

    s->USARTx = uartDev->dev;

#if defined(USE_UART1_RX_DMA) || defined(USE_UART2_RX_DMA) || defined(USE_UART3_RX_DMA)
    // This doesn't work... Was original RX DMA working at all???
    if (uartDev->rxDMAChannel) {
        s->rxDMAChannel = uartDev->rxDMAChannel;
        s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;
        dmaInit(dmaGetIdentifier(uartDev->rxDMAChannel), OWNER_SERIAL_RX, RESOURCE_INDEX(device));
    }
#endif

// Get rid of this when DMA become configurable
#if defined(USE_UART1_TX_DMA) || defined(USE_UART2_TX_DMA) || defined(USE_UART3_TX_DMA)
    if (uartDev->txDMAChannel) {
        s->txDMAChannel = uartDev->txDMAChannel;
        s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
        const dmaIdentifier_e identifier = dmaGetIdentifier(uartDev->txDMAChannel);

        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(identifier, handleUsartTxDma, uartDev->txPriority, (uint32_t)s);
    }
#endif

    RCC_ClockCmd(uartDev->rcc, ENABLE);

#if defined(USE_UART1_TX_DMA) || defined(USE_UART1_RX_DMA)
// Is this handled in dmaInit or dmaSetHandler?
    RCC_ClockCmd(RCC_AHB(DMA1), ENABLE);
#endif

    serialUARTInitIO(IOGetByTag(uartDev->tx), IOGetByTag(uartDev->rx), mode, options, uartDev->af, device);

    if (!uartDev->rxDMAChannel) {
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = uartDev->irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(uartDev->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(uartDev->rxPriority);
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

void USART1_IRQHandler(void)
{
    usartIrqHandler(&(uartHardwareMap[UARTDEV_1]->port));
}
#endif

#ifdef USE_UART2
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_2, baudRate, mode, options);
}

void USART2_IRQHandler(void)
{
    usartIrqHandler(&(uartHardwareMap[UARTDEV_2]->port));
}
#endif

#ifdef USE_UART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_3, baudRate, mode, options);
}

void USART3_IRQHandler(void)
{
    usartIrqHandler(&(uartHardwareMap[UARTDEV_3]->port));
}
#endif

#ifdef USE_UART4
uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_4, baudRate, mode, options);
}

void UART4_IRQHandler(void)
{
    usartIrqHandler(&(uartHardwareMap[UARTDEV_4]->port));
}
#endif

#ifdef USE_UART5
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_5, baudRate, mode, options);
}

void UART5_IRQHandler(void)
{
    usartIrqHandler(&(uartHardwareMap[UARTDEV_5]->port));
}
#endif
