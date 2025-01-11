/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 * Copyright (c) 2020, Koji Kitayama
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
 */

#include "bsp/board_api.h"
#include "board.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_uart.h"
#include "fsl_sysmpu.h"

#include "board/clock_config.h"
#include "board/pin_mux.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void) {
#if CFG_TUH_ENABLED
  tuh_int_handler(0);
#endif
#if CFG_TUD_ENABLED
  tud_int_handler(0);
#endif
}

void board_init(void) {
  BOARD_InitBootPins();
  BOARD_BootClockRUN();
  SystemCoreClockUpdate();
  SYSMPU_Enable(SYSMPU, 0);

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  // LED
  gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 0 };
  GPIO_PinInit(LED_PORT, LED_PIN, &led_config);
  board_led_write(false);

#if defined(BUTTON_PORT) && defined(BUTTON_PIN)
  gpio_pin_config_t button_config = { kGPIO_DigitalInput, 0 };
  GPIO_PinInit(BUTTON_PORT, BUTTON_PIN, &button_config);
#endif

#ifdef UART_DEV
  const uart_config_t uart_config = {
      .baudRate_Bps = 115200UL,
      .parityMode = kUART_ParityDisabled,
      .stopBitCount = kUART_OneStopBit,
      .txFifoWatermark = 0U,
      .rxFifoWatermark = 1U,
      .idleType = kUART_IdleTypeStartBit,
      .enableTx = true,
      .enableRx = true
  };
  UART_Init(UART_DEV, &uart_config, UART_CLOCK);
#endif

  // USB
  // USB clock is configured in BOARD_BootClockRUN()
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_PinWrite(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
}

uint32_t board_button_read(void) {
#if defined(BUTTON_PORT) && defined(BUTTON_PIN)
  return BUTTON_STATE_ACTIVE == GPIO_PinRead(BUTTON_PORT, BUTTON_PIN);
#else
  return 0;
#endif
}

int board_uart_read(uint8_t *buf, int len) {
  (void) buf;
  (void) len;
#ifdef UART_DEV
  // Read blocking will block until there is data
//  UART_ReadBlocking(UART_DEV, buf, len);
//  return len;
  return 0;
#else
  return 0;
#endif
}

int board_uart_write(void const *buf, int len) {
  (void) buf;
  (void) len;

#ifdef UART_DEV
  UART_WriteBlocking(UART_DEV, (uint8_t const*) buf, len);
  return len;
#else
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


#ifndef __ICCARM__
// Implement _start() since we use linker flag '-nostartfiles'.
// Requires defined __STARTUP_CLEAR_BSS,
extern int main(void);
TU_ATTR_UNUSED void _start(void) {
  // called by startup code
  main();
  while (1) {}
}

#ifdef __clang__
void	_exit (int __status) {
  while (1) {}
}
#endif

#endif
