/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
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

#include "xmc_gpio.h"
#include "xmc_scu.h"
#include "xmc_uart.h"

#include "bsp/board_api.h"
#include "board.h"


//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB0_0_IRQHandler(void) {
  tud_int_handler(0);
}

void board_init(void) {
  board_clock_init();
  SystemCoreClockUpdate();

  // LED
  XMC_GPIO_CONFIG_t led_cfg = {0};
  led_cfg.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
  led_cfg.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
  led_cfg.output_strength = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM;
  XMC_GPIO_Init(LED_PIN, &led_cfg);

  // Button
  XMC_GPIO_CONFIG_t button_cfg = {0};
  button_cfg.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
  XMC_GPIO_Init(BUTTON_PIN, &button_cfg);

#ifdef UART_DEV
  XMC_UART_CH_CONFIG_t uart_cfg = {0};
  uart_cfg.baudrate = CFG_BOARD_UART_BAUDRATE;
  uart_cfg.data_bits = 8;
  uart_cfg.stop_bits = 1;
  XMC_UART_CH_Init(UART_DEV, &uart_cfg);

  XMC_GPIO_SetMode(UART_RX_PIN, XMC_GPIO_MODE_INPUT_PULL_UP);
  XMC_UART_CH_SetInputSource(UART_DEV, XMC_UART_CH_INPUT_RXD, UART_RX_INPUT);

  XMC_UART_CH_Start(UART_DEV);
  XMC_GPIO_SetMode(UART_TX_PIN, (XMC_GPIO_MODE_t)(XMC_GPIO_MODE_OUTPUT_PUSH_PULL | UART_TX_PIN_AF));
#endif

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // Explicitly disable systick to prevent its ISR runs before scheduler start
  SysTick->CTRL &= ~1U;

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  // USB Power Enable
#if(UC_SERIES != XMC45)
  XMC_SCU_CLOCK_UngatePeripheralClock(XMC_SCU_PERIPHERAL_CLOCK_USB0);
#endif
  XMC_SCU_RESET_DeassertPeripheralReset(XMC_SCU_PERIPHERAL_RESET_USB0);
  XMC_SCU_POWER_EnableUsb();
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  uint32_t is_high = state ? LED_STATE_ON : (1 - LED_STATE_ON);

  XMC_GPIO_SetOutputLevel(LED_PIN, is_high ? XMC_GPIO_OUTPUT_LEVEL_HIGH : XMC_GPIO_OUTPUT_LEVEL_LOW);
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == XMC_GPIO_GetInput(BUTTON_PIN);
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  uint8_t const len = tu_min8(16, max_len);
  memcpy(id, g_chipid, len);
  return len;
}

int board_uart_read(uint8_t* buf, int len) {
#ifdef UART_DEV
  for(int i=0;i<len;i++) {
    buf[i] = XMC_UART_CH_GetReceivedData(UART_DEV);
  }
  return len;
#else
  (void) buf;
  (void) len;
  return 0;
#endif
}

int board_uart_write(void const* buf, int len) {
#ifdef UART_DEV
  char const* bufch = (char const*) buf;
  for(int i=0;i<len;i++) {
    XMC_UART_CH_Transmit(UART_DEV, bufch[i]);
  }
  return len;
#else
  (void) buf;
  (void) len;
  return 0;
#endif
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
//void _init(void) {
//}
