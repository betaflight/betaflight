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

#include "chip.h"
#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void) {
  tusb_int_handler(0, true);
}

void USB1_IRQHandler(void) {
  tusb_int_handler(1, true);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

/* System configuration variables used by chip driver */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

// Invoked by startup code
void SystemInit(void) {
#ifdef __USE_LPCOPEN
  extern void (*const g_pfnVectors[])(void);
  unsigned int *pSCB_VTOR = (unsigned int *) 0xE000ED08;
  *pSCB_VTOR = (unsigned int) g_pfnVectors;
#endif

  board_lpc18_pinmux();

#ifdef TRACE_ETM
  // Trace clock is limited to 60MHz, limit CPU clock to 120MHz
  Chip_SetupCoreClock(CLKIN_CRYSTAL, 120000000UL, true);
#else
  // CPU clock max to 180 Mhz
  Chip_SetupCoreClock(CLKIN_CRYSTAL, MAX_CLOCK_FREQ, true);
#endif

}

void board_init(void) {
  SystemCoreClockUpdate();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_SetPriority(USB1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  Chip_GPIO_Init(LPC_GPIO_PORT);

  // LED
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED_PORT, LED_PIN);

  // Button
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, BUTTON_PORT, BUTTON_PIN);

  //------------- UART -------------//
  Chip_UART_Init(UART_DEV);
  Chip_UART_SetBaud(UART_DEV, CFG_BOARD_UART_BAUDRATE);
  Chip_UART_ConfigData(UART_DEV, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
  Chip_UART_TXEnable(UART_DEV);

  //------------- USB -------------//
  Chip_USB0_Init();
  Chip_USB1_Init();
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED_PORT, LED_PIN, state);
}

uint32_t board_button_read(void) {
  // active low
  return Chip_GPIO_GetPinState(LPC_GPIO_PORT, BUTTON_PORT, BUTTON_PIN) ? 0 : 1;
}

int board_uart_read(uint8_t *buf, int len) {
  return Chip_UART_Read(UART_DEV, buf, len);
}

int board_uart_write(void const *buf, int len) {
  uint8_t const *buf8 = (uint8_t const *) buf;
  for (int i = 0; i < len; i++) {
    while ((Chip_UART_ReadLineStatus(UART_DEV) & UART_LSR_THRE) == 0) {}
    Chip_UART_SendByte(UART_DEV, buf8[i]);
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
