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

#include <stdint.h>
#include <stdbool.h>
#include "csr.h"
#include "irq.h"

#include "bsp/board_api.h"

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void fomu_error(uint32_t line)
{
  (void)line;
  TU_BREAKPOINT();
}

volatile uint32_t system_ticks = 0;
static void timer_init(void)
{
	int t;

	timer0_en_write(0);
	t = CONFIG_CLOCK_FREQUENCY / 1000; // 1000 kHz tick
	timer0_reload_write(t);
	timer0_load_write(t);
	timer0_en_write(1);
  timer0_ev_enable_write(1);
  timer0_ev_pending_write(1);
	irq_setmask(irq_getmask() | (1 << TIMER0_INTERRUPT));
}

void isr(void)
{
  unsigned int irqs;

  irqs = irq_pending() & irq_getmask();

#if CFG_TUD_ENABLED
  if (irqs & (1 << USB_INTERRUPT)) {
    tud_int_handler(0);
  }
#endif
  if (irqs & (1 << TIMER0_INTERRUPT)) {
    system_ticks++;
    timer0_ev_pending_write(1);
  }
}

void board_init(void)
{
  irq_setmask(0);
  irq_setie(1);
  timer_init();
  return;
}

void board_led_write(bool state)
{
  rgb_ctrl_write(0xff);
  rgb_raw_write(state);
}

uint32_t board_button_read(void)
{
  return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  int32_t offset = 0;
  uint8_t const* buf8 = (uint8_t const*) buf;
  for (offset = 0; offset < len; offset++)
  {
    if (!(messible_status_read() & CSR_MESSIBLE_STATUS_FULL_OFFSET))
    {
      messible_in_write(buf8[offset]);
    }
  }
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
uint32_t board_millis(void)
{
  return system_ticks;
}
#endif
