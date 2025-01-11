/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Jerzy Kasenberg
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
#include <xc.h>
#include "tusb.h"

void __attribute__((interrupt(IPL2AUTO), vector(_USB_VECTOR), no_fpu))
USBD_IRQHandler(void)
{
  IFS4CLR = _IFS4_USBIF_MASK;
  tud_int_handler(0);
}

TU_ATTR_WEAK void button_init(void)
{
}

TU_ATTR_WEAK void led_init(void)
{
}

TU_ATTR_WEAK void uart_init(void)
{
}

void board_init(void)
{
  button_init();
  led_init();
  uart_init();

  // Force device mode by overriding USB ID and settings it to 1
  USBCRCONbits.PHYIDEN = 0;
  USBCRCONbits.USBIDVAL = 1;
  USBCRCONbits.USBIDOVEN = 1;

  // set interrupt priority (must much IPL2AUTO)
  IPC33CLR = _IPC33_USBIP_MASK;
  IPC33SET = (2 << _IPC33_USBIP_POSITION);
  // set interrupt subpriority
  IPC33CLR = _IPC33_USBIS_MASK;
  IPC33SET = (0 << _IPC33_USBIS_POSITION);

  USBCRCONbits.USBIE = 0;
  IFS4CLR = _IFS4_USBIF_MASK;
  IEC4SET = _IEC4_USBIE_MASK;

  __builtin_enable_interrupts();
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

TU_ATTR_WEAK void board_led_write(bool state)
{
  (void) state;
}

TU_ATTR_WEAK uint32_t board_button_read(void)
{
  return 0;
}

TU_ATTR_WEAK int board_uart_read(uint8_t * buf, int len)
{
  (void) buf;
  (void) len;

  return 0;
}

TU_ATTR_WEAK int board_uart_write(void const * buf, int len)
{
  (void) buf;
  return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
uint32_t board_millis(void)
{
  // COUNTER is system clock (200MHz / 2 = 100MHz) convert to ms)
  return _CP0_GET_COUNT() / (100000000 / 1000);
}
#endif
