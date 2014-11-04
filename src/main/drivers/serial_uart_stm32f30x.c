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
#include <stdlib.h>

#include "platform.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"

#include "serial.h"
#include "serial_uart.h"

// Using RX DMA disables the use of receive callbacks
#define USE_USART1_RX_DMA
//#define USE_USART2_RX_DMA
//#define USE_USART2_TX_DMA

#define UART1_TX_PIN GPIO_Pin_9  // PA9
#define UART1_RX_PIN GPIO_Pin_10 // PA10
#define UART1_GPIO GPIOA
#define UART1_TX_PINSOURCE GPIO_PinSource9
#define UART1_RX_PINSOURCE GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_5 // PD5
#define UART2_RX_PIN        GPIO_Pin_6 // PD6
#define UART2_GPIO          GPIOD
#define UART2_TX_PINSOURCE  GPIO_PinSource5
#define UART2_RX_PINSOURCE  GPIO_PinSource6

static uartPort_t uartPort1;
static uartPort_t uartPort2;

void uartStartTxDMA(uartPort_t *s);

uartPort_t *serialUSART1(uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s;
    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    s = &uartPort1;
    s->port.vTable = uartVTable;
    
    s->port.baudRate = baudRate;
    
    s->port.rxBuffer = rx1Buffer;
    s->port.txBuffer = tx1Buffer;
    s->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART1_TX_BUFFER_SIZE;
    
#ifdef USE_USART1_RX_DMA
    s->rxDMAChannel = DMA1_Channel5;
#endif
    s->txDMAChannel = DMA1_Channel4;

    s->USARTx = USART1;

    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;
    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    if (mode & MODE_TX) {
        GPIO_InitStructure.GPIO_Pin = UART1_TX_PIN;
        GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PINSOURCE, GPIO_AF_7);
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
    }

    if (mode & MODE_RX) {
        GPIO_InitStructure.GPIO_Pin = UART1_RX_PIN;
        GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PINSOURCE, GPIO_AF_7);
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
    }

    if (mode & MODE_BIDIR) {
        GPIO_InitStructure.GPIO_Pin = UART1_TX_PIN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PINSOURCE, GPIO_AF_7);
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
    }

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART1_TXDMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART1_TXDMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#ifndef USE_USART1_RX_DMA
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART1_RXDMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART1_RXDMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    return s;
}

uartPort_t *serialUSART2(uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s;
    static volatile uint8_t rx2Buffer[UART2_RX_BUFFER_SIZE];
    static volatile uint8_t tx2Buffer[UART2_TX_BUFFER_SIZE];
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    s = &uartPort2;
    s->port.vTable = uartVTable;
    
    s->port.baudRate = baudRate;
    
    s->port.rxBufferSize = UART2_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART2_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx2Buffer;
    s->port.txBuffer = tx2Buffer;

    s->USARTx = USART2;
    
#ifdef USE_USART2_RX_DMA
    s->rxDMAChannel = DMA1_Channel6;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->RDR;
#endif
#ifdef USE_USART2_TX_DMA
    s->txDMAChannel = DMA1_Channel7;
    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->TDR;
#endif

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

#if defined(USE_USART2_TX_DMA) || defined(USE_USART2_RX_DMA)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    if (mode & MODE_TX) {
        GPIO_InitStructure.GPIO_Pin = UART2_TX_PIN;
        GPIO_PinAFConfig(UART2_GPIO, UART2_TX_PINSOURCE, GPIO_AF_7);
        GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
    }

    if (mode & MODE_RX) {
        GPIO_InitStructure.GPIO_Pin = UART2_RX_PIN;
        GPIO_PinAFConfig(UART2_GPIO, UART2_RX_PINSOURCE, GPIO_AF_7);
        GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
    }

    if (mode & MODE_BIDIR) {
        GPIO_InitStructure.GPIO_Pin = UART2_TX_PIN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_PinAFConfig(UART2_GPIO, UART2_TX_PINSOURCE, GPIO_AF_7);
        GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
    }

#ifdef USE_USART2_TX_DMA
    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART2_TXDMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART2_TXDMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#ifndef USE_USART2_RX_DMA
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART2_RXDMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART2_RXDMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    return s;
}

static void handleUsartTxDma(uartPort_t *s)
{
    DMA_Cmd(s->txDMAChannel, DISABLE);

    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}

// USART1 Tx DMA Handler
void DMA1_Channel4_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    handleUsartTxDma(s);
}

// USART2 Tx DMA Handler
void DMA1_Channel7_IRQHandler(void)
{
    uartPort_t *s = &uartPort2;
    DMA_ClearITPendingBit(DMA1_IT_TC7);
    DMA_Cmd(DMA1_Channel7, DISABLE);
    handleUsartTxDma(s);
}

void usartIrqHandler(uartPort_t *s)
{
    uint32_t ISR = s->USARTx->ISR;

    if (!s->rxDMAChannel && (ISR & USART_FLAG_RXNE)) {
        if (s->port.callback) {
            s->port.callback(s->USARTx->RDR);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->RDR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    if (!s->txDMAChannel && (ISR & USART_FLAG_TXE)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }

    if (ISR & USART_FLAG_ORE)
    {
        USART_ClearITPendingBit (s->USARTx, USART_IT_ORE);
    }
}

void USART1_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;

    usartIrqHandler(s);
}

void USART2_IRQHandler(void)
{
    uartPort_t *s = &uartPort2;

    usartIrqHandler(s);
}

