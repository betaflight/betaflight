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
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Port baseflight STM32F10x to STM32F30x for cleanflight
 * J. Ihlein - Code from FocusFlight32
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifdef USE_UART

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

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
        .rxDMAChannel = UART1_RX_DMA,
        .txDMAChannel = UART1_TX_DMA,
        .rxPins = { DEFIO_TAG_E(PA10), DEFIO_TAG_E(PB7), DEFIO_TAG_E(PC5), DEFIO_TAG_E(PE1) },
        .txPins = { DEFIO_TAG_E(PA9), DEFIO_TAG_E(PB6), DEFIO_TAG_E(PC4), DEFIO_TAG_E(PE0) },
        .rcc = RCC_APB2(USART1),
        .af = GPIO_AF_7,
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1_RXDMA,
    },
#endif

#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = USART2,
        .rxDMAChannel = UART2_RX_DMA,
        .txDMAChannel = UART2_TX_DMA,
        .rxPins = { DEFIO_TAG_E(PA15), DEFIO_TAG_E(PA3), DEFIO_TAG_E(PB4), DEFIO_TAG_E(PD6) },
        .txPins = { DEFIO_TAG_E(PA14), DEFIO_TAG_E(PA2), DEFIO_TAG_E(PB3), DEFIO_TAG_E(PD5) },
        .rcc = RCC_APB1(USART2),
        .af = GPIO_AF_7,
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2_RXDMA,
    },
#endif

#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
        .rxDMAChannel = UART3_RX_DMA,
        .txDMAChannel = UART3_TX_DMA,
        .rxPins = { DEFIO_TAG_E(PB11), DEFIO_TAG_E(PC11), DEFIO_TAG_E(PD9), IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PB10), DEFIO_TAG_E(PC10), DEFIO_TAG_E(PD8), IO_TAG_NONE },
        .rcc = RCC_APB1(USART3),
        .af = GPIO_AF_7,
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3_RXDMA,
    },
#endif

#ifdef USE_UART4
    // UART4 XXX Not tested (yet!?) Need 303RC, e.g. LUX for testing
    {
        .device = UARTDEV_4,
        .reg = UART4,
        .rxDMAChannel = 0, // XXX UART4_RX_DMA !?
        .txDMAChannel = 0, // XXX UART4_TX_DMA !?
        .rxPins = { DEFIO_TAG_E(PC11), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PC10), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .rcc = RCC_APB1(UART4),
        .af = GPIO_AF_5,
        .irqn = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4_RXDMA,
    },
#endif

#ifdef USE_UART5
    // UART5 XXX Not tested (yet!?) Need 303RC; e.g. LUX for testing
    {
        .device = UARTDEV_5,
        .reg = UART5,
        .rxDMAChannel = 0,
        .txDMAChannel = 0,
        .rxPins = { DEFIO_TAG_E(PD2), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .txPins = { DEFIO_TAG_E(PC12), IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE },
        .rcc = RCC_APB1(UART5),
        .af = GPIO_AF_5,
        .irqn = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5,
        .rxPriority = NVIC_PRIO_SERIALUART5,
    },
#endif
};

static void handleUsartTxDma(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = (uartPort_t*)(descriptor->userParam);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    DMA_Cmd(descriptor->ref, DISABLE);

    uartTryStartTxDMA(s);
}

void serialUARTInitIO(IO_t txIO, IO_t rxIO, portMode_e mode, portOptions_e options, uint8_t af, uint8_t index)
{
    if ((options & SERIAL_BIDIR) && txIO) {
        ioConfig_t ioCfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_OType_PP : GPIO_OType_OD,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP
        );

        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(index));
        IOConfigGPIOAF(txIO, ioCfg, af);

        if (!(options & SERIAL_INVERTED))
            IOLo(txIO);   // OpenDrain output should be inactive
    } else {
        ioConfig_t ioCfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, (options & SERIAL_INVERTED) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP);
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(index));
            IOConfigGPIOAF(txIO, ioCfg, af);
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(index));
            IOConfigGPIOAF(rxIO, ioCfg, af);
        }
    }
}

// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartDev = uartDevmap[device];
    if (!uartDev) {
        return NULL;
    }

    uartPort_t *s = &(uartDev->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uartDev->rxBuffer;
    s->port.txBuffer = uartDev->txBuffer;
    s->port.rxBufferSize = sizeof(uartDev->rxBuffer);
    s->port.txBufferSize = sizeof(uartDev->txBuffer);

    const uartHardware_t *hardware = uartDev->hardware;

    s->USARTx = hardware->reg;

    RCC_ClockCmd(hardware->rcc, ENABLE);

    if (hardware->rxDMAChannel) {
        dmaInit(dmaGetIdentifier(hardware->rxDMAChannel), OWNER_SERIAL_RX, RESOURCE_INDEX(device));
        s->rxDMAChannel = hardware->rxDMAChannel;
        s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;
    }

    if (hardware->txDMAChannel) {
        const dmaIdentifier_e identifier = dmaGetIdentifier(hardware->txDMAChannel);
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        dmaSetHandler(identifier, handleUsartTxDma, hardware->txPriority, (uint32_t)s);
        s->txDMAChannel = hardware->txDMAChannel;
        s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
    }

    serialUARTInitIO(IOGetByTag(uartDev->tx), IOGetByTag(uartDev->rx), mode, options, hardware->af, device);

    if (!s->rxDMAChannel || !s->txDMAChannel) {
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
    uint32_t ISR = s->USARTx->ISR;

    if (!s->rxDMAChannel && (ISR & USART_FLAG_RXNE)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->RDR, s->port.rxCallbackData);
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
#endif // USE_UART
