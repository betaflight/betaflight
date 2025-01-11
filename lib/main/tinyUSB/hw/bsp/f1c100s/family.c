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
#include <malloc.h>
#include <irqflags.h>
#include <f1c100s-irq.h>
#include "bsp/board_api.h"
#include "board.h"

extern void sys_uart_putc(char c);

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

static void timer_init(void);

void board_init(void) {
  arch_local_irq_disable();
  do_init_mem_pool();
  f1c100s_intc_init();
  timer_init();
  printf("Timer INIT done\n");
  arch_local_irq_enable();
}

// No LED, no button
void board_led_write(bool state) {
  (void) state;
}

uint32_t board_button_read(void) {
  return 0;
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  int txsize = len;
  while (txsize--) {
    sys_uart_putc(*(uint8_t const*) buf);
    buf++;
  }
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

uint32_t board_millis(void) {
  return system_ticks;
}

static void timer_handler(void) {
  volatile uint32_t* temp_addr = (uint32_t*) (0x01C20C00 + 0x04);

  /* clear timer */
  *temp_addr |= 0x01;

  system_ticks++;
}

static void timer_init(void) {
  uint32_t temp;
  volatile uint32_t* temp_addr;

  /* reload value */
  temp = 12000000 / 1000;
  temp_addr = (uint32_t*) (0x01C20C00 + 0x14);
  *temp_addr = temp;

  /* continuous | /2 | 24Mhz |  reload*/
  temp = (0x00 << 7) | (0x01 << 4) | (0x01 << 2) | (0x00 << 1);
  temp_addr = (uint32_t*) (0x01C20C00 + 0x10);
  *temp_addr &= 0xffffff00;
  *temp_addr |= temp;

  /* open timer irq */
  temp = 0x01 << 0;
  temp_addr = (uint32_t*) (0x01C20C00);
  *temp_addr |= temp;

  /* set init value */
  temp_addr = (uint32_t*) (0x01C20C00 + 0x18);
  *temp_addr = 0;

  /* begin run timer */
  temp = 0x01 << 0;
  temp_addr = (uint32_t*) (0x01C20C00 + 0x10);
  *temp_addr |= temp;

  f1c100s_intc_set_isr(F1C100S_IRQ_TIMER0, timer_handler);
  f1c100s_intc_enable_irq(F1C100S_IRQ_TIMER0);
}

#else
static void timer_init(void) { }
#endif
