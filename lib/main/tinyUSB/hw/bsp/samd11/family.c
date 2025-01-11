/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org)
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

#include "sam.h"

// Suppress warning caused by mcu driver
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

#include "hal/include/hal_gpio.h"
#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d11.h"

#include "hpl/gclk/hpl_gclk_base.h"
#include "hpl_pm_config.h"
#include "hpl/pm/hpl_pm_base.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "bsp/board_api.h"
#include "board.h"


//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_Handler(void)
{
  tud_int_handler(0);
}


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

/* Referenced GCLKs, should be initialized firstly */
#define _GCLK_INIT_1ST (1 << 0 | 1 << 1)

/* Not referenced GCLKs, initialized last */
#define _GCLK_INIT_LAST (~_GCLK_INIT_1ST)

void board_init(void)
{
  // Clock init ( follow hpl_init.c )
  hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);

  _pm_init();
  _sysctrl_init_sources();
#if _GCLK_INIT_1ST
  _gclk_init_generators_by_fref(_GCLK_INIT_1ST);
#endif
  _sysctrl_init_referenced_generators();
  _gclk_init_generators_by_fref(_GCLK_INIT_LAST);

  // 1ms tick timer (samd SystemCoreClock may not correct)
  SystemCoreClock = CONF_CPU_FREQUENCY;
  SysTick_Config(CONF_CPU_FREQUENCY / 1000);

  // Led init
  gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
  gpio_set_pin_level(LED_PIN, 0);

  // Button init
  gpio_set_pin_direction(BUTTON_PIN, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BUTTON_PIN, BUTTON_PULL_MODE);

  /* USB Clock init
   * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
   * for low speed and full speed operation. */
  _pm_enable_bus_clock(PM_BUS_APBB, USB);
  _pm_enable_bus_clock(PM_BUS_AHB, USB);
  _gclk_enable_channel(USB_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

  // USB Pin Init
  gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
  gpio_set_pin_level(PIN_PA24, false);
  gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
  gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
  gpio_set_pin_level(PIN_PA25, false);
  gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

  gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
  gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  gpio_set_pin_level(LED_PIN, state ? LED_STATE_ON : (1-LED_STATE_ON));
}

uint32_t board_button_read(void)
{
  return BUTTON_STATE_ACTIVE == gpio_get_pin_level(BUTTON_PIN);
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

void _init(void)
{
  // This _init() standin makes certain GCC environments happier.
  // They expect the main binary to have a constructor called _init; but don't provide a weak default.
  // Providing an empty constructor satisfies this odd case, and doesn't harm anything.
}


#endif
