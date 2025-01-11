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

#include "stdio.h"
#include "debug_uart.h"

#include "ch32f20x.h"

#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

void USBHS_IRQHandler(void)
{
  tud_int_handler(0);
}

void board_init(void) {

  /* Disable interrupts during init */
  __disable_irq();

#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(SystemCoreClock / 1000);
#endif

#if CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USBHS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

	usart_printf_init(115200);

  // USB HS Clock config
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(RCC_USBPLL_Div2);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);

  // LED
  LED_CLOCK_EN();
  GPIO_InitTypeDef led_pin_config = {
    .GPIO_Pin = LED_PIN,
    .GPIO_Mode = GPIO_Mode_Out_OD,
    .GPIO_Speed = GPIO_Speed_50MHz,
  };
  GPIO_Init(LED_PORT, &led_pin_config);

  // Button
  BUTTON_CLOCK_EN();
  GPIO_InitTypeDef button_pin_config = {
    .GPIO_Pin = BUTTON_PIN,
    .GPIO_Mode = GPIO_Mode_IPU,
    .GPIO_Speed = GPIO_Speed_50MHz,
  };
  GPIO_Init(BUTTON_PORT, &button_pin_config);

  /* Enable interrupts globally */
  __enable_irq();
}

#if CFG_TUSB_OS == OPT_OS_NONE

volatile uint32_t system_ticks = 0;

void SysTick_Handler(void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}

#endif

void HardFault_Handler(void)
{
  __asm("BKPT #0\n");
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  GPIO_WriteBit(LED_PORT, LED_PIN, state);
}

uint32_t board_button_read(void)
{
  return BUTTON_STATE_ACTIVE == GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN);
}

int board_uart_read(uint8_t *buf, int len)
{
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const *buf, int len)
{
  int txsize = len;
  while ( txsize-- )
  {
    uart_write(*(uint8_t const*) buf);
    buf++;
  }
  return len;
}
