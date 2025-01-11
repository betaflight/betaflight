/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Greg Davill
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

#include "debug_uart.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif

#include <ch32v30x.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#define UART_RINGBUFFER_SIZE_TX 128
#define UART_RINGBUFFER_MASK_TX (UART_RINGBUFFER_SIZE_TX-1)

static char tx_buf[UART_RINGBUFFER_SIZE_TX];
static unsigned int tx_produce;
static volatile unsigned int tx_consume;

void USART1_IRQHandler(void) __attribute__((naked));
void USART1_IRQHandler(void) {
      __asm volatile ("call USART1_IRQHandler_impl; mret");
}

__attribute__((used)) void USART1_IRQHandler_impl(void)
{
	if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);

		if(tx_consume != tx_produce) {
			USART_SendData(USART1, tx_buf[tx_consume]);
			tx_consume = (tx_consume + 1) & UART_RINGBUFFER_MASK_TX;
		}
    }

}

void uart_write(char c)
{
	unsigned int tx_produce_next = (tx_produce + 1) & UART_RINGBUFFER_MASK_TX;

    NVIC_DisableIRQ(USART1_IRQn);
    if((tx_consume != tx_produce) || (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)) {
		tx_buf[tx_produce] = c;
		tx_produce = tx_produce_next;
	} else {
		USART_SendData(USART1, c);
	}
    NVIC_EnableIRQ(USART1_IRQn);
}


void uart_sync(void)
{
	while(tx_consume != tx_produce);
}


void usart_printf_init(uint32_t baudrate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  tx_produce = 0;
  tx_consume = 0;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  USART_Cmd(USART1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure = { 0 };
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
