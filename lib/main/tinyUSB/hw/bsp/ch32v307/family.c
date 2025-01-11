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

#include "stdio.h"

// https://github.com/openwch/ch32v307/pull/90
// https://github.com/openwch/ch32v20x/pull/12

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif

#include "debug_uart.h"
#include "ch32v30x.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

// TODO maybe having FS as port0, HS as port1

__attribute__((interrupt)) void USBHS_IRQHandler(void) {
  #if CFG_TUD_WCH_USBIP_USBHS
  tud_int_handler(0);
  #endif
}

__attribute__((interrupt)) void OTG_FS_IRQHandler(void) {
  #if CFG_TUD_WCH_USBIP_USBFS
  tud_int_handler(0);
  #endif
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

uint32_t SysTick_Config(uint32_t ticks) {
  NVIC_EnableIRQ(SysTicK_IRQn);
  SysTick->CTLR = 0;
  SysTick->SR = 0;
  SysTick->CNT = 0;
  SysTick->CMP = ticks - 1;
  SysTick->CTLR = 0xF;
  return 0;
}

void board_init(void) {

  /* Disable interrupts during init */
  __disable_irq();

#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(SystemCoreClock / 1000);
#endif

  usart_printf_init(CFG_BOARD_UART_BAUDRATE);

#ifdef CH32V30x_D8C
  // v305/v307: Highspeed USB
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(RCC_USBPLL_Div2);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif

  // Fullspeed USB
  uint8_t otg_div;
  switch (SystemCoreClock) {
    case 48000000:  otg_div = RCC_OTGFSCLKSource_PLLCLK_Div1; break;
    case 96000000:  otg_div = RCC_OTGFSCLKSource_PLLCLK_Div2; break;
    case 144000000: otg_div = RCC_OTGFSCLKSource_PLLCLK_Div3; break;
    default: TU_ASSERT(0,); break;
  }
  RCC_OTGFSCLKConfig(otg_div);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure = {0};

  // LED
  LED_CLOCK_EN();
  GPIO_InitStructure.GPIO_Pin = LED_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);

  // Button
  BUTTON_CLOCK_EN();
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);

  /* Enable interrupts globally */
  __enable_irq();

  board_delay(2);
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

__attribute__((interrupt)) void SysTick_Handler(void) {
  SysTick->SR = 0;
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}

#endif

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_WriteBit(LED_PORT, LED_PIN, state);
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN);
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  int txsize = len;
  const char* bufc = (const char*) buf;
  while (txsize--) {
    uart_write(*bufc++);
  }
  uart_sync();
  return len;
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
