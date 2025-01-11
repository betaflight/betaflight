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
#include "board.h"

// Suppress warning caused by mcu driver
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#endif

#include "broadcom/cpu.h"
#include "broadcom/gpio.h"
#include "broadcom/interrupts.h"
#include "broadcom/mmu.h"
#include "broadcom/caches.h"
#include "broadcom/vcmailbox.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

// LED
#define LED_PIN               18
#define LED_STATE_ON          1

// UART TX
#define UART_TX_PIN           14

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_IRQHandler(void) {
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+
void board_init(void) {
  setup_mmu_flat_map();
  init_caches();

  // LED
  gpio_set_function(LED_PIN, GPIO_FUNCTION_OUTPUT);
  gpio_set_pull(LED_PIN, BP_PULL_NONE);
  board_led_write(true);

  // Uart
  COMPLETE_MEMORY_READS;
  AUX->ENABLES_b.UART_1 = true;

  UART1->IER = 0;
  UART1->CNTL = 0;
  UART1->LCR_b.DATA_SIZE = UART1_LCR_DATA_SIZE_MODE_8BIT;
  UART1->MCR = 0;
  UART1->IER = 0;

  uint32_t source_clock = vcmailbox_get_clock_rate_measured(VCMAILBOX_CLOCK_CORE);
  UART1->BAUD = ((source_clock / (115200 * 8)) - 1);
  UART1->CNTL |= UART1_CNTL_TX_ENABLE_Msk;
  COMPLETE_MEMORY_READS;

  gpio_set_function(UART_TX_PIN, GPIO_FUNCTION_ALT5);

  // Turn on USB peripheral.
  vcmailbox_set_power_state(VCMAILBOX_DEVICE_USB_HCD, true);

  // Timer 1/1024 second tick
  SYSTMR->CS_b.M1 = 1;
  SYSTMR->C1 = SYSTMR->CLO + 977;
  BP_EnableIRQ(TIMER_1_IRQn);

  BP_SetPriority(USB_IRQn, 0x00);
  BP_ClearPendingIRQ(USB_IRQn);
  BP_EnableIRQ(USB_IRQn);
  BP_EnableIRQs();
}

void board_led_write(bool state) {
  gpio_set_value(LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
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
  for (int i = 0; i < len; i++) {
    const char* cbuf = buf;
    while (!UART1->STAT_b.TX_READY) {}
    if (cbuf[i] == '\n') {
      UART1->IO = '\r';
      while (!UART1->STAT_b.TX_READY) {}
    }
    UART1->IO = cbuf[i];
  }
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void TIMER_1_IRQHandler(void) {
  system_ticks++;
  SYSTMR->C1 += 977;
  SYSTMR->CS_b.M1 = 1;
}

uint32_t board_millis(void) {
  return system_ticks;
}

#endif

void HardFault_Handler(void) {
  // asm("bkpt");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {

}
