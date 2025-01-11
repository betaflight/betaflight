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
 *
 * This file is part of the TinyUSB stack.
 */

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"

#include "clock_config.h"
#include "bsp/board_api.h"
#include "board.h"


//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void) {
  tud_int_handler(0);
}

void board_init(void) {
  /* Enable port clocks for GPIO pins */
  CLOCK_EnableClock(kCLOCK_PortA);
  CLOCK_EnableClock(kCLOCK_PortB);
  CLOCK_EnableClock(kCLOCK_PortC);
  CLOCK_EnableClock(kCLOCK_PortD);
  CLOCK_EnableClock(kCLOCK_PortE);

  BOARD_InitBootPins();
  BOARD_BootClockRUN();
  SystemCoreClockUpdate();

  gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};
  GPIO_PinInit(LED_GPIO, LED_PIN, &led_config);
  PORT_SetPinMux(LED_PORT, LED_PIN, kPORT_MuxAsGpio);

#ifdef BUTTON_PIN
  gpio_pin_config_t button_config = {kGPIO_DigitalInput, 0};
  GPIO_PinInit(BUTTON_GPIO, BUTTON_PIN, &button_config);
  const port_pin_config_t BUTTON_CFG = {
      kPORT_PullUp,
      kPORT_FastSlewRate,
      kPORT_PassiveFilterDisable,
#if defined(FSL_FEATURE_PORT_HAS_OPEN_DRAIN) && FSL_FEATURE_PORT_HAS_OPEN_DRAIN
      kPORT_OpenDrainDisable,
#endif
      kPORT_LowDriveStrength,
      kPORT_MuxAsGpio,
#if defined(FSL_FEATURE_PORT_HAS_PIN_CONTROL_LOCK) && FSL_FEATURE_PORT_HAS_PIN_CONTROL_LOCK
      kPORT_UnlockRegister
#endif
  };
  PORT_SetPinConfig(BUTTON_PORT, BUTTON_PIN, &BUTTON_CFG);
#endif

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  lpuart_config_t uart_config;
  LPUART_GetDefaultConfig(&uart_config);
  uart_config.baudRate_Bps = CFG_BOARD_UART_BAUDRATE;
  uart_config.enableTx = true;
  uart_config.enableRx = true;
  LPUART_Init(UART_PORT, &uart_config, UART_CLOCK_SOURCE_HZ);

  // USB
  CLOCK_EnableUsbfs0Clock(USB_CLOCK_SOURCE, 48000000U);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_PinWrite(LED_GPIO, LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
}

uint32_t board_button_read(void) {
#ifdef BUTTON_PIN
  return BUTTON_STATE_ACTIVE == GPIO_PinRead(BUTTON_GPIO, BUTTON_PIN);
#else
  return 0;
#endif
}

int board_uart_read(uint8_t* buf, int len) {
#if 0 /*
	Use this version if want the LED to blink during BOARD=board_test,
	without having to hit a key.
      */
  if( 0U != (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags( UART_PORT )) )
    {
      LPUART_ReadBlocking(UART_PORT, buf, len);
      return len;
    }

  return( 0 );
#else /* Wait for 'len' characters to come in */

  LPUART_ReadBlocking(UART_PORT, buf, len);
  return len;

#endif
}

int board_uart_write(void const* buf, int len) {
  LPUART_WriteBlocking(UART_PORT, (uint8_t const*) buf, len);
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
