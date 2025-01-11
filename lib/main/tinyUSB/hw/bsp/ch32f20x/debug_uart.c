/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Denis Krasutski
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */
#include <ch32f20x.h>

#include "board.h"
#include "debug_uart.h"

#define UART_RINGBUFFER_SIZE_TX 64
#define UART_RINGBUFFER_MASK_TX (UART_RINGBUFFER_SIZE_TX-1)

static char tx_buf[UART_RINGBUFFER_SIZE_TX];
static unsigned int tx_produce = 0;
static volatile unsigned int tx_consume = 0;

void UART_DEV_IRQHandler(void)
{
  if(USART_GetITStatus(UART_DEV, USART_IT_TC) != RESET) {
    USART_ClearITPendingBit(UART_DEV, USART_IT_TC);

    if(tx_consume != tx_produce) {
      USART_SendData(UART_DEV, tx_buf[tx_consume]);
      tx_consume = (tx_consume + 1) & UART_RINGBUFFER_MASK_TX;
    }
  }
}

void uart_write(char c)
{
  unsigned int tx_produce_next = (tx_produce + 1) & UART_RINGBUFFER_MASK_TX;

  NVIC_DisableIRQ(UART_DEV_IRQn);
  if((tx_consume != tx_produce) || (USART_GetFlagStatus(UART_DEV, USART_FLAG_TXE) == RESET)) {
    tx_buf[tx_produce] = c;
    tx_produce = tx_produce_next;
  } else {
    USART_SendData(UART_DEV, c);
  }
  NVIC_EnableIRQ(UART_DEV_IRQn);
}

void uart_sync(void)
{
  while(tx_consume != tx_produce) {
    //Waiting for transfer complete
  }
}

void usart_printf_init(uint32_t baudrate)
{
  tx_produce = 0;
  tx_consume = 0;

  UART_DEV_CLK_EN();

  GPIO_InitTypeDef gpio_config = {
    .GPIO_Pin = UART_DEV_TX_PIN,
    .GPIO_Speed = GPIO_Speed_50MHz,
    .GPIO_Mode = GPIO_Mode_AF_PP,
  };
  GPIO_Init(UART_DEV_GPIO_PORT, &gpio_config);

  USART_InitTypeDef uart_config = {
    .USART_BaudRate = baudrate,
    .USART_WordLength = USART_WordLength_8b,
    .USART_StopBits = USART_StopBits_1,
    .USART_Parity = USART_Parity_No,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
    .USART_Mode = USART_Mode_Tx,
  };

  USART_Init(UART_DEV, &uart_config);
  USART_ITConfig(UART_DEV, USART_IT_TC, ENABLE);
  USART_Cmd(UART_DEV, ENABLE);

  NVIC_InitTypeDef nvic_config = {
    .NVIC_IRQChannel = UART_DEV_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 1,
    .NVIC_IRQChannelSubPriority = 3,
    .NVIC_IRQChannelCmd = ENABLE,
  };
  NVIC_Init(&nvic_config);
}
