/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Brent Kowal (Analog Devices, Inc)
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
#pragma GCC diagnostic ignored "-Wstrict-prototypes" // _mxc_crit_get_state()
#endif

#include "gpio.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_device.h"
#include "uart.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "board.h"
#include "bsp/board_api.h"


//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_IRQHandler(void) {
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(UART_NUM);

void board_init(void) {
#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif
  mxc_gpio_cfg_t gpioConfig;

  // LED
  gpioConfig.drvstr = MXC_GPIO_DRVSTR_0;
  gpioConfig.func = MXC_GPIO_FUNC_OUT;
  gpioConfig.mask = LED_PIN;
  gpioConfig.pad = MXC_GPIO_PAD_NONE;
  gpioConfig.port = LED_PORT;
  gpioConfig.vssel = LED_VDDIO;
  MXC_GPIO_Config(&gpioConfig);
  board_led_write(false);

  // Button
  gpioConfig.drvstr = MXC_GPIO_DRVSTR_0;
  gpioConfig.func = MXC_GPIO_FUNC_IN;
  gpioConfig.mask = BUTTON_PIN;
  gpioConfig.pad = BUTTON_PULL;
  gpioConfig.port = BUTTON_PORT;
  gpioConfig.vssel = MXC_GPIO_VSSEL_VDDIO;
  MXC_GPIO_Config(&gpioConfig);

  // UART
  MXC_UART_Init(ConsoleUart, CFG_BOARD_UART_BAUDRATE, MXC_UART_IBRO_CLK);

  //USB
  MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
  MXC_MCR->ldoctrl |= MXC_F_MCR_LDOCTRL_0P9EN;
  MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_USB);
  MXC_SYS_Reset_Periph(MXC_SYS_RESET0_USB);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
#if LED_STATE_ON
  state = !state;
#endif
  if (state) {
    MXC_GPIO_OutClr(LED_PORT, LED_PIN);
  } else {
    MXC_GPIO_OutSet(LED_PORT, LED_PIN);
  }
}

uint32_t board_button_read(void) {
  uint32_t state = MXC_GPIO_InGet(BUTTON_PORT, BUTTON_PIN) ? 1 : 0;
  return BUTTON_STATE_ACTIVE == state;
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  uint8_t hw_id[MXC_SYS_USN_CHECKSUM_LEN];//USN Buffer
                                          /* All other 2nd parameter is optional checksum buffer */
  MXC_SYS_GetUSN(hw_id, NULL);

  size_t act_len = TU_MIN(max_len, MXC_SYS_USN_LEN);
  memcpy(id, hw_id, act_len);
  return act_len;
}

int board_uart_read(uint8_t *buf, int len) {
  int uart_val;
  int act_len = 0;

  while (act_len < len) {
    if ((uart_val = MXC_UART_ReadCharacterRaw(ConsoleUart)) == E_UNDERFLOW) {
      break;
    } else {
      *buf++ = (uint8_t) uart_val;
      act_len++;
    }
  }
  return act_len;
}

int board_uart_write(void const *buf, int len) {
  int act_len = 0;
  const uint8_t *ch_ptr = (const uint8_t *) buf;
  while (act_len < len) {
    MXC_UART_WriteCharacter(ConsoleUart, *ch_ptr++);
    act_len++;
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

void HardFault_Handler(void) {
  __asm("BKPT #0\n");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {
}
