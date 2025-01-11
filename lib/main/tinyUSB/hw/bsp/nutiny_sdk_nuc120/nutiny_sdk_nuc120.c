/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#include "bsp/board_api.h"
#include "NUC100Series.h"
#include "clk.h"
#include "sys.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USBD_IRQHandler(void)
{
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#define LED_PORT     PB
#define LED_PIN      0
#define LED_PIN_IO   PB0
#define LED_STATE_ON 0

void board_init(void)
{
  SYS_UnlockReg();

  /* Enable Internal RC 22.1184 MHz clock */
  CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

  /* Waiting for Internal RC clock ready */
  CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

  /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
  CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

  /* Enable external XTAL 12 MHz clock */
  CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

  /* Waiting for external XTAL clock ready */
  CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

  /* Set core clock */
  CLK_SetCoreClock(48000000);

  /* Enable module clock */
  CLK_EnableModuleClock(USBD_MODULE);

  /* Select module clock source */
  CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV_USB(1));

  SYS_LockReg();

#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(48000000 / 1000);
#endif

  GPIO_SetMode(LED_PORT, 1UL << LED_PIN, GPIO_PMD_OUTPUT);
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
#if 0
  /* this would be the simplest solution... *IF* the part supported the pin data interface */
  LED_PIN_IO = (state) ? LED_STATE_ON : (1-LED_STATE_ON);
#else
  /* if the part's *PDIO pin data registers don't work, a more elaborate approach is needed */
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  uint32_t current = LED_PORT->DOUT & ~(1UL << LED_PIN);
  LED_PORT->DOUT = current | (((state) ? LED_STATE_ON : (1UL-LED_STATE_ON)) << LED_PIN);
  __set_PRIMASK(irq_state);
#endif
}

uint32_t board_button_read(void)
{
  return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  (void) buf; (void) len;
  return 0;
}
