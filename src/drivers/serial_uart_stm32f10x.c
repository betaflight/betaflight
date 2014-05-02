
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "system_common.h"
#include "gpio_common.h"

#include "serial_common.h"
#include "serial_uart_common.h"

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

static uartPort_t uartPort1;
static uartPort_t uartPort2;

void uartStartTxDMA(uartPort_t *s);

// USART1 - Telemetry (RX/TX by DMA)
uartPort_t *serialUSART1(uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s;
    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort1;
    s->port.vTable = uartVTable;
    
    s->port.baudRate = baudRate;
    
    s->port.rxBuffer = rx1Buffer;
    s->port.txBuffer = tx1Buffer;
    s->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART1_TX_BUFFER_SIZE;
    
    s->USARTx = USART1;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

    s->rxDMAChannel = DMA1_Channel5;
    s->txDMAChannel = DMA1_Channel4;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    // USART1_TX    PA9
    // USART1_RX    PA10
    gpio.speed = Speed_2MHz;
    gpio.pin = Pin_9;
    gpio.mode = Mode_AF_PP;
    if (mode & MODE_TX)
        gpioInit(GPIOA, &gpio);
    gpio.pin = Pin_10;
    gpio.mode = Mode_IPU;
    if (mode & MODE_RX)
        gpioInit(GPIOA, &gpio);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

// USART2 - GPS or Spektrum or ?? (RX + TX by IRQ)
uartPort_t *serialUSART2(uint32_t baudRate, portMode_t mode)
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    // USART2_TX    PA2
    // USART2_RX    PA3
    gpio.speed = Speed_2MHz;
    gpio.pin = Pin_2;
    gpio.mode = Mode_AF_PP;
    if (mode & MODE_TX)
        gpioInit(GPIOA, &gpio);
    gpio.pin = Pin_3;
    gpio.mode = Mode_IPU;
    if (mode & MODE_RX)
        gpioInit(GPIOA, &gpio);

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

// Handlers

// USART1 Tx DMA Handler
void DMA1_Channel4_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(s->txDMAChannel, DISABLE);

    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}

// USART1 Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_TXE) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            s->USARTx->DR = s->port.txBuffer[s->port.txBufferTail];
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}

// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartPort_t *s = &uartPort2;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
        // If we registered a callback, pass crap there
        if (s->port.callback) {
            s->port.callback(s->USARTx->DR);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }
    if (SR & USART_FLAG_TXE) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            s->USARTx->DR = s->port.txBuffer[s->port.txBufferTail];
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}
