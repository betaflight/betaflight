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
#include "NuMicro.h"
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
#define LED_PORT              PC
#define LED_PIN               9
#define LED_PIN_IO            PC9
#define LED_STATE_ON          0

#define CRYSTAL_LESS /* system will be 48MHz when defined, otherwise, system is 72MHz */
#define HIRC48_AUTO_TRIM    SYS_IRCTCTL1_REFCKSEL_Msk | (1UL << SYS_IRCTCTL1_LOOPSEL_Pos) | (2UL << SYS_IRCTCTL1_FREQSEL_Pos)
#define TRIM_INIT           (SYS_BASE+0x118)

void board_init(void)
{
  /* Unlock protected registers */
  SYS_UnlockReg();

  /*---------------------------------------------------------------------------------------------------------*/
  /* Init System Clock                                                                                       */
  /*---------------------------------------------------------------------------------------------------------*/

  /* Enable Internal RC 22.1184 MHz clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

  /* Waiting for Internal RC clock ready */
  CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

  /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#ifndef CRYSTAL_LESS
  /* Enable external XTAL 12 MHz clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

  /* Waiting for external XTAL clock ready */
  CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

  /* Set core clock */
  CLK_SetCoreClock(72000000);

  /* Use HIRC as UART clock source */
  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

  /* Use PLL as USB clock source */
  CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));

#else
  /* Enable Internal RC 48MHz clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

  /* Waiting for Internal RC clock ready */
  CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

  /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC48, CLK_CLKDIV0_HCLK(1));

  /* Use HIRC as UART clock source */
  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

  /* Use HIRC48 as USB clock source */
  CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC48, CLK_CLKDIV0_USB(1));
#endif

  /* Enable module clock */
  CLK_EnableModuleClock(USBD_MODULE);

#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(48000000 / 1000);
#endif

  // LED
  GPIO_SetMode(LED_PORT, 1 << LED_PIN, GPIO_MODE_OUTPUT);
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
  LED_PIN_IO = (state ? LED_STATE_ON : (1-LED_STATE_ON));
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
