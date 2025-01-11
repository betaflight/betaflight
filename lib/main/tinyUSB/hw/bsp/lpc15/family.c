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

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-prototypes"
  #pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include "chip.h"

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif

#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_IRQHandler(void)
{
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

/* System oscillator rate and RTC oscillator rate */
const uint32_t OscRateIn = XTAL_OscRateIn;
const uint32_t RTCOscRateIn = XTAL_RTCOscRateIn;

// Invoked by startup code
void SystemInit(void)
{
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SRAM1);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SRAM2);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_SYSCTL_PeriphReset(RESET_IOCON);

  board_lpc15_pinmux_swm_init();

  Chip_SetupXtalClocking();
}

void board_init(void)
{
  SystemCoreClockUpdate();

  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

#if CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  Chip_GPIO_Init(LPC_GPIO);

  // LED
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);

  // Button
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, BUTTON_PORT, BUTTON_PIN);

	// UART
	Chip_Clock_SetUARTBaseClockRate(Chip_Clock_GetMainClockRate(), false);
	Chip_UART_Init(UART_PORT);
	Chip_UART_ConfigData(UART_PORT, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(UART_PORT, CFG_BOARD_UART_BAUDRATE);
	Chip_UART_Enable(UART_PORT);
	Chip_UART_TXEnable(UART_PORT);

  // USB: Setup PLL clock, and power
  Chip_USB_Init();
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, state);
}

uint32_t board_button_read(void)
{
  // active low
  return Chip_GPIO_GetPinState(LPC_GPIO, BUTTON_PORT, BUTTON_PIN) ? 0 : 1;
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  return Chip_UART_SendBlocking(UART_PORT, buf, len);
}

#if CFG_TUSB_OS == OPT_OS_NONE
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
