/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, hathach (tinyusb.org)
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
 */

#include "sam.h"

// Suppress warning caused by mcu driver
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#endif

#include "peripheral_clk_config.h"
#include "hal/include/hal_init.h"
#include "hal/include/hpl_usart_sync.h"
#include "hpl/pmc/hpl_pmc.h"
#include "hal/include/hal_gpio.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "bsp/board_api.h"
#include "board.h"


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
struct _usart_sync_device edbg_com;

//------------- IMPLEMENTATION -------------//
void board_init(void) {
  init_mcu();

  _pmc_enable_periph_clock(ID_PIOA);

  /* Disable Watchdog */
  hri_wdt_set_MR_WDDIS_bit(WDT);

  // LED
  gpio_set_pin_level(LED_PIN, false);
  gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(LED_PIN, GPIO_PIN_FUNCTION_OFF);

  // Button
  gpio_set_pin_direction(BUTTON_PIN, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BUTTON_PIN, GPIO_PULL_UP);
  gpio_set_pin_function(BUTTON_PIN, GPIO_PIN_FUNCTION_OFF);

  // Uart via EDBG Com
  _pmc_enable_periph_clock(ID_FLEXCOM7);
  gpio_set_pin_function(UART_RX_PIN, MUX_PA27B_FLEXCOM7_RXD);
  gpio_set_pin_function(UART_TX_PIN, MUX_PA28B_FLEXCOM7_TXD);

  _usart_sync_init(&edbg_com, FLEXCOM7);
  _usart_sync_set_baud_rate(&edbg_com, CFG_BOARD_UART_BAUDRATE);
  _usart_sync_set_mode(&edbg_com, USART_MODE_ASYNCHRONOUS);
  _usart_sync_enable(&edbg_com);

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer (samd SystemCoreClock may not correct)
  SysTick_Config(CONF_CPU_FREQUENCY / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  NVIC_SetPriority(UDP_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  // USB Pin, Clock init

  /* Clear SYSIO 10 & 11 for USB DM & DP */
  hri_matrix_clear_CCFG_SYSIO_reg(MATRIX, CCFG_SYSIO_SYSIO10 | CCFG_SYSIO_SYSIO11);

  // Enable clock
  _pmc_enable_periph_clock(ID_UDP);

  /* USB Device mode & Transceiver active */
  hri_matrix_write_CCFG_USBMR_reg(MATRIX, CCFG_USBMR_USBMODE);
}

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void UDP_Handler(void) {
  #if CFG_TUD_ENABLED
  tud_int_handler(0);
  #endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  gpio_set_pin_level(LED_PIN, state);
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == gpio_get_pin_level(BUTTON_PIN);
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  uint8_t const* buf8 = (uint8_t const*) buf;
  for (int i = 0; i < len; i++) {
    while (!_usart_sync_is_ready_to_send(&edbg_com)) {}
    _usart_sync_write_byte(&edbg_com, buf8[i]);
  }
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}

#endif

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {

}
