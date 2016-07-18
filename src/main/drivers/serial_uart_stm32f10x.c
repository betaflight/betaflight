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
#include <stdlib.h>

#include <platform.h>

#include "common/utils.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"
#include "dma.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"
#include "serial_uart_stm32f10x.h"

#ifdef USE_UART1
static uartPort_t uartPort1;
#endif

#ifdef USE_UART2
static uartPort_t uartPort2;
#endif

#ifdef USE_UART3
static uartPort_t uartPort3;
#endif

#ifdef USE_UART4
static uartPort_t uartPort4;
#endif

#ifdef USE_UART5
static uartPort_t uartPort5;
#endif

// Using RX DMA disables the use of receive callbacks
#define USE_UART1_RX_DMA

#if defined(CC3D) // FIXME move board specific code to target.h files.
#undef USE_UART1_RX_DMA
#endif

void usartIrqHandler(uartPort_t *s)
{
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE && !s->rxDMAChannel) {
        // If we registered a callback, pass crap there
        if (s->port.callback) {
            s->port.callback(s->USARTx->DR);
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

#ifdef USE_UART1

// UART1 Tx DMA Handler
void UART_TX_DMA_IRQHandler(dmaChannel_t* descriptor, dmaCallbackHandler_t* handler)
{
    uartPort_t *s = container_of(handler, uartPort_t, dmaTxHandler);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    DMA_Cmd(descriptor->channel, DISABLE);

    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}

// UART1 - Telemetry (RX/TX by DMA)
uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];
    gpio_config_t gpio;

    s = &uartPort1;
    s->port.vTable = uartVTable;
    
    s->port.baudRate = baudRate;
    
    s->port.rxBuffer = rx1Buffer;
    s->port.txBuffer = tx1Buffer;
    s->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART1_TX_BUFFER_SIZE;
    
    s->USARTx = USART1;


#ifdef USE_UART1_RX_DMA
    s->rxDMAChannel = DMA1_Channel5;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
#endif
    s->txDMAChannel = DMA1_Channel4;
    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

#ifdef UART1_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(UART1_APB1_PERIPHERALS, ENABLE);
#endif
#ifdef UART1_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(UART1_APB2_PERIPHERALS, ENABLE);
#endif
#ifdef UART1_AHB_PERIPHERALS
    RCC_AHBPeriphClockCmd(UART1_AHB_PERIPHERALS, ENABLE);
#endif

    gpio.speed = Speed_2MHz;

    gpio.pin = UART1_TX_PIN;
    if (options & SERIAL_BIDIR) {
        gpio.mode = Mode_AF_OD;
        gpioInit(UART1_GPIO, &gpio);
    } else {
        if (mode & MODE_TX) {
            gpio.mode = Mode_AF_PP;
            gpioInit(UART1_GPIO, &gpio);
        }

        if (mode & MODE_RX) {
            gpio.pin = UART1_RX_PIN;
            gpio.mode = Mode_IPU;
            gpioInit(UART1_GPIO, &gpio);
        }
    }

    // DMA TX Interrupt
    dmaHandlerInit(&uartPort1.dmaTxHandler, UART_TX_DMA_IRQHandler);
    dmaSetHandler(DMA1Channel4Descriptor, &uartPort1.dmaTxHandler, NVIC_PRIO_SERIALUART1_TXDMA);

#ifndef USE_UART1_RX_DMA
    // RX/TX Interrupt
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART1);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART1);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    return s;
}

// UART1 Rx/Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;
    usartIrqHandler(s);
}

#endif

#ifdef USE_UART2
// UART2 - GPS or Spektrum or ?? (RX + TX by IRQ)
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx2Buffer[UART2_RX_BUFFER_SIZE];
    static volatile uint8_t tx2Buffer[UART2_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort2;
    s->port.vTable = uartVTable;
    
    s->port.baudRate = baudRate;
    
    s->port.rxBufferSize = UART2_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART2_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx2Buffer;
    s->port.txBuffer = tx2Buffer;
    
    s->USARTx = USART2;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

#ifdef UART2_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(UART2_APB1_PERIPHERALS, ENABLE);
#endif
#ifdef UART2_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(UART2_APB2_PERIPHERALS, ENABLE);
#endif
#ifdef UART2_AHB_PERIPHERALS
    RCC_APB2PeriphClockCmd(UART2_AHB_PERIPHERALS, ENABLE);
#endif

    gpio.speed = Speed_2MHz;

    gpio.pin = UART2_TX_PIN;
    if (options & SERIAL_BIDIR) {
        gpio.mode = Mode_AF_OD;
        gpioInit(UART2_GPIO, &gpio);
    } else {
        if (mode & MODE_TX) {
            gpio.mode = Mode_AF_PP;
            gpioInit(UART2_GPIO, &gpio);
        }

        if (mode & MODE_RX) {
            gpio.pin = UART2_RX_PIN;
            gpio.mode = Mode_IPU;
            gpioInit(UART2_GPIO, &gpio);
        }
    }

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART2);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART2);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}


// UART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartPort_t *s = &uartPort2;
    usartIrqHandler(s);
}

#endif

#ifdef USE_UART3
// UART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx3Buffer[UART3_RX_BUFFER_SIZE];
    static volatile uint8_t tx3Buffer[UART3_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort3;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = rx3Buffer;
    s->port.txBuffer = tx3Buffer;
    s->port.rxBufferSize = UART3_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART3_TX_BUFFER_SIZE;

    s->USARTx = USART3;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

#ifdef UART3_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(UART3_APB1_PERIPHERALS, ENABLE);
#endif
#ifdef UART3_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(UART3_APB2_PERIPHERALS, ENABLE);
#endif

    gpio.speed = Speed_2MHz;

    gpio.pin = UART3_TX_PIN;
    if (options & SERIAL_BIDIR) {
        gpio.mode = Mode_AF_OD;
        gpioInit(UART3_GPIO, &gpio);
    } else {
        if (mode & MODE_TX) {
            gpio.mode = Mode_AF_PP;
            gpioInit(UART3_GPIO, &gpio);
        }

        if (mode & MODE_RX) {
            gpio.pin = UART3_RX_PIN;
            gpio.mode = Mode_IPU;
            gpioInit(UART3_GPIO, &gpio);
        }
    }

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART3);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART3);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

// UART2 Rx/Tx IRQ Handler
void USART3_IRQHandler(void)
{
    uartPort_t *s = &uartPort3;
    usartIrqHandler(s);
}
#endif

#ifdef USE_UART4
// UART4
uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx4Buffer[UART4_RX_BUFFER_SIZE];
    static volatile uint8_t tx4Buffer[UART4_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort4;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = rx4Buffer;
    s->port.txBuffer = tx4Buffer;
    s->port.rxBufferSize = UART4_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART4_TX_BUFFER_SIZE;

    s->USARTx = UART4;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

#ifdef UART4_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(UART4_APB1_PERIPHERALS, ENABLE);
#endif
#ifdef UART4_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(UART4_APB2_PERIPHERALS, ENABLE);
#endif

    gpio.speed = Speed_2MHz;

    gpio.pin = UART4_TX_PIN;
    if (options & SERIAL_BIDIR) {
        gpio.mode = Mode_AF_OD;
        gpioInit(UART4_GPIO, &gpio);
    } else {
        if (mode & MODE_TX) {
            gpio.mode = Mode_AF_PP;
            gpioInit(UART4_GPIO, &gpio);
        }

        if (mode & MODE_RX) {
            gpio.pin = UART4_RX_PIN;
            gpio.mode = Mode_IPU;
            gpioInit(UART4_GPIO, &gpio);
        }
    }

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART4);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART4);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

// UART4 Rx/Tx IRQ Handler
void UART4_IRQHandler(void)
{
    uartPort_t *s = &uartPort4;
    usartIrqHandler(s);
}
#endif

#ifdef USE_UART5
// UART5
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx5Buffer[UART5_RX_BUFFER_SIZE];
    static volatile uint8_t tx5Buffer[UART5_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort5;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = rx5Buffer;
    s->port.txBuffer = tx5Buffer;
    s->port.rxBufferSize = UART5_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART5_TX_BUFFER_SIZE;

    s->USARTx = UART5;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

#ifdef UART5_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(UART5_APB1_PERIPHERALS, ENABLE);
#endif
#ifdef UART5_APB2_PERIPHERALS_TX
    RCC_APB2PeriphClockCmd(UART5_APB2_PERIPHERALS_TX, ENABLE);
#endif
#ifdef UART5_APB2_PERIPHERALS_RX
    RCC_APB2PeriphClockCmd(UART5_APB2_PERIPHERALS_RX, ENABLE);
#endif

    gpio.speed = Speed_2MHz;

    gpio.pin = UART5_TX_PIN;
    if (options & SERIAL_BIDIR) {
        gpio.mode = Mode_AF_OD;
        gpioInit(UART5_GPIO_TX, &gpio);
    } else {
        if (mode & MODE_TX) {
            gpio.mode = Mode_AF_PP;
            gpioInit(UART5_GPIO_TX, &gpio);
        }

        if (mode & MODE_RX) {
            gpio.pin = UART5_RX_PIN;
            gpio.mode = Mode_IPU;
            gpioInit(UART5_GPIO_RX, &gpio);
        }
    }

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART5);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART5);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

// UART5 Rx/Tx IRQ Handler
void UART5_IRQHandler(void)
{
    uartPort_t *s = &uartPort5;
    usartIrqHandler(s);
}
#endif
