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
#include "NUC505Series.h"

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
#define LED_PORT     PC
#define LED_PIN      3
#define LED_STATE_ON 0

void board_init(void)
{
  /* Enable XTAL */
  CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

  CLK_SetCoreClock(96000000);

  /* Set PCLK divider */
  CLK_SetModuleClock(PCLK_MODULE, 0, 1);

  /* Update System Core Clock */
  SystemCoreClockUpdate();

  /* Enable USB IP clock */
  CLK_EnableModuleClock(USBD_MODULE);

  /* Select USB IP clock source */
  CLK_SetModuleClock(USBD_MODULE, CLK_USBD_SRC_EXT, 0);

  CLK_SetModuleClock(PCLK_MODULE, 0, 1);

  /* Enable PHY */
  USBD_ENABLE_PHY();
  /* wait PHY clock ready */
  while (1) {
      USBD->EP[EPA].EPMPS = 0x20;
      if (USBD->EP[EPA].EPMPS == 0x20)
          break;
  }

  /* Force SE0, and then clear it to connect*/
  USBD_SET_SE0();

#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(96000000 / 1000);
#endif

  GPIO_SetMode(LED_PORT, 1UL << LED_PIN, GPIO_MODE_OUTPUT);
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
  uint32_t current = (state) ? LED_STATE_ON : (1-LED_STATE_ON);
  current <<= LED_PIN;
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  current |= LED_PORT->DOUT & ~(1UL << LED_PIN);
  LED_PORT->DOUT = current;
  __set_PRIMASK(irq_state);
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
