/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
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
#include <hal/hal_gpio.h>
#include <mcu/mcu.h>

#define LED_STATE_OFF   (1-LED_STATE_ON)

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_IRQHandler(void) {
  tud_int_handler(0);
}

// DA146xx driver function that must be called whenever VBUS changes.
extern void tusb_vbus_changed(bool present);

#if defined(NEED_VBUS_MONITOR) && CFG_TUD_ENABLED
// VBUS change interrupt handler
void VBUS_IRQHandler(void) {
  bool present = (CRG_TOP->ANA_STATUS_REG & CRG_TOP_ANA_STATUS_REG_VBUS_AVAILABLE_Msk) != 0;
  // Clear VBUS interrupt
  CRG_TOP->VBUS_IRQ_CLEAR_REG = 1;

  tusb_vbus_changed(present);
}
#endif

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
void UnhandledIRQ(void) {
  CRG_TOP->SYS_CTRL_REG = 0x80;
  __BKPT(1);
  while (1);
}

void board_init(void) {
  // LED
  hal_gpio_init_out(LED_PIN, LED_STATE_ON);

  hal_gpio_init_out(1, 0);
  hal_gpio_init_out(2, 0);
  hal_gpio_init_out(3, 0);
  hal_gpio_init_out(4, 0);
  hal_gpio_init_out(5, 0);

  // Button
  hal_gpio_init_in(BUTTON_PIN, BUTTON_STATE_ACTIVE ? HAL_GPIO_PULL_DOWN : HAL_GPIO_PULL_UP);

  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

#if CFG_TUD_ENABLED
  #ifdef NEED_VBUS_MONITOR
  // Setup interrupt for both connect and disconnect
  CRG_TOP->VBUS_IRQ_MASK_REG = CRG_TOP_VBUS_IRQ_MASK_REG_VBUS_IRQ_EN_FALL_Msk |
                               CRG_TOP_VBUS_IRQ_MASK_REG_VBUS_IRQ_EN_RISE_Msk;
  NVIC_SetPriority(VBUS_IRQn, 2);
  // Trigger interrupt at the start to inform driver about VBUS state at start
  // otherwise it could go unnoticed.
  NVIC_SetPendingIRQ(VBUS_IRQn);
  NVIC_EnableIRQ(VBUS_IRQn);
  #else
  // This board is USB powered there is no need to monitor VBUS line.  Notify driver that VBUS is present.
  tusb_vbus_changed(true);
  #endif

  /* Setup USB IRQ */
  NVIC_SetPriority(USB_IRQn, 2);
  NVIC_EnableIRQ(USB_IRQn);

  /* Use PLL96 / 2 clock not HCLK */
  CRG_TOP->CLK_CTRL_REG &= ~CRG_TOP_CLK_CTRL_REG_USB_CLK_SRC_Msk;

  mcu_gpio_set_pin_function(14, MCU_GPIO_MODE_INPUT, MCU_GPIO_FUNC_USB);
  mcu_gpio_set_pin_function(15, MCU_GPIO_MODE_INPUT, MCU_GPIO_FUNC_USB);
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  hal_gpio_write(LED_PIN, state ? LED_STATE_ON : LED_STATE_OFF);
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == hal_gpio_read(BUTTON_PIN);
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  (void) buf;
  (void) len;

  return 0;
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
