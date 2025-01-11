/*
 * The MIT License (MIT)
 *
 * Copyright 2019 Sony Semiconductor Solutions Corporation
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

#include <sys/boardctl.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include "bsp/board_api.h"

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
#define LED_PIN         PIN_I2S1_BCK

#define BUTTON_PIN      PIN_HIF_IRQ_OUT

// Initialize on-board peripherals : led, button, uart and USB
void board_init(void)
{
  boardctl(BOARDIOC_INIT, 0);

  board_gpio_write(PIN_I2S1_BCK, -1);
  board_gpio_config(PIN_I2S1_BCK, 0, false, true, PIN_FLOAT);

  board_gpio_write(PIN_HIF_IRQ_OUT, -1);
  board_gpio_config(PIN_HIF_IRQ_OUT, 0, true, true, PIN_FLOAT);
};

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

// Turn LED on or off
void board_led_write(bool state)
{
  board_gpio_write(LED_PIN, state);
}

// Get the current state of button
// a '1' means active (pressed), a '0' means inactive.
uint32_t board_button_read(void)
{
  if (board_gpio_read(BUTTON_PIN))
  {
    return 0;
  }

  return 1;
}

// Get characters from UART
int board_uart_read(uint8_t *buf, int len)
{
  int r = read(0, buf, len);

  return r;
}

// Send characters to UART
int board_uart_write(void const *buf, int len)
{
  int r = write(1, buf, len);

  return r;
}

// Get current milliseconds
uint32_t board_millis(void)
{
  struct timespec tp;

    /* Wait until RTC is available */
    while (g_rtc_enabled == false);

    if (clock_gettime(CLOCK_MONOTONIC, &tp))
    {
        return 0;
    }

    return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}
