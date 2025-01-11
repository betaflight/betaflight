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
#include "bsp/board_api.h"

#include "peripheral_clk_config.h"
#include "hpl/usart/hpl_usart_base.h"
#include "hpl/pmc/hpl_pmc.h"
#include "hal/include/hal_init.h"
#include "hal/include/hal_usart_async.h"
#include "hal/include/hal_gpio.h"


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define LED_PIN               GPIO(GPIO_PORTC, 8)

#define BUTTON_PIN            GPIO(GPIO_PORTA, 11)
#define BUTTON_STATE_ACTIVE   0

#define UART_TX_PIN           GPIO(GPIO_PORTB, 4)
#define UART_RX_PIN           GPIO(GPIO_PORTA, 21)

static struct usart_async_descriptor edbg_com;
static uint8_t edbg_com_buffer[64];
static volatile bool uart_busy = false;

static void tx_cb_EDBG_COM(const struct usart_async_descriptor *const io_descr)
{
  (void) io_descr;
  uart_busy = false;
}

//------------- IMPLEMENTATION -------------//
void board_init(void)
{
  init_mcu();

  /* Disable Watchdog */
  hri_wdt_set_MR_WDDIS_bit(WDT);

  // LED
  _pmc_enable_periph_clock(ID_PIOC);
  gpio_set_pin_level(LED_PIN, false);
  gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(LED_PIN, GPIO_PIN_FUNCTION_OFF);

  // Button
  _pmc_enable_periph_clock(ID_PIOA);
  gpio_set_pin_direction(BUTTON_PIN, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BUTTON_PIN, GPIO_PULL_UP);
  gpio_set_pin_function(BUTTON_PIN, GPIO_PIN_FUNCTION_OFF);

  // Uart via EDBG Com
  _pmc_enable_periph_clock(ID_USART1);
  gpio_set_pin_function(UART_RX_PIN, MUX_PA21A_USART1_RXD1);
  gpio_set_pin_function(UART_TX_PIN, MUX_PB4D_USART1_TXD1);

  usart_async_init(&edbg_com, USART1, edbg_com_buffer, sizeof(edbg_com_buffer), _usart_get_usart_async());
  usart_async_set_baud_rate(&edbg_com, CFG_BOARD_UART_BAUDRATE);
  usart_async_register_callback(&edbg_com, USART_ASYNC_TXC_CB, tx_cb_EDBG_COM);
  usart_async_enable(&edbg_com);

#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer (samd SystemCoreClock may not correct)
  SysTick_Config(CONF_CPU_FREQUENCY / 1000);
#endif

  // Enable USB clock
  _pmc_enable_periph_clock(ID_USBHS);

}

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void USBHS_Handler(void)
{
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  gpio_set_pin_level(LED_PIN, state);
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
  // while until previous transfer is complete
  while(uart_busy) {}
  uart_busy = true;

  io_write(&edbg_com.io, buf, len);
  return len;
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

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}
