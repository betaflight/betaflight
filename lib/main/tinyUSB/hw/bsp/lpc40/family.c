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
void USB_IRQHandler(void) {
  #if CFG_TUD_ENABLED
  tud_int_handler(0);
  #endif

  #if CFG_TUH_ENABLED
  tuh_int_handler(0, true);
  #endif
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// Invoked by startup code
void SystemInit(void) {
#ifdef __USE_LPCOPEN
  extern void (*const g_pfnVectors[])(void);
  unsigned int *pSCB_VTOR = (unsigned int *) 0xE000ED08;
  *pSCB_VTOR = (unsigned int) g_pfnVectors;

  #if __FPU_USED == 1
  fpuInit();
  #endif
#endif // __USE_LPCOPEN

  Chip_IOCON_Init(LPC_IOCON);
  Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing, sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));

#ifdef TRACE_ETM
  const PINMUX_GRP_T trace_pinmux[] = {
      {2, 2, IOCON_FUNC5 | IOCON_FASTSLEW_EN },
      {2, 3, IOCON_FUNC5 | IOCON_FASTSLEW_EN },
      {2, 4, IOCON_FUNC5 | IOCON_FASTSLEW_EN },
      {2, 5, IOCON_FUNC5 | IOCON_FASTSLEW_EN },
      {2, 6, IOCON_FUNC5 | IOCON_FASTSLEW_EN },
  };
  Chip_IOCON_SetPinMuxing(LPC_IOCON, trace_pinmux, sizeof(trace_pinmux) / sizeof(PINMUX_GRP_T));
#endif

  /* CPU clock source starts with IRC */
  /* Enable PBOOST for CPU clock over 100MHz */
  Chip_SYSCTL_EnableBoost();

  Chip_SetupXtalClocking();
}

void board_init(void) {
  SystemCoreClockUpdate();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  Chip_GPIO_Init(LPC_GPIO);

  // LED
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);

  // Button
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, BUTTON_PORT, BUTTON_PIN);

  // UART

  //------------- USB -------------//

  // Port1 as Host, Port2: Device
  Chip_USB_Init();

  enum {
    USBCLK_DEVCIE = 0x12, // AHB + Device
    USBCLK_HOST = 0x19,  // AHB + OTG + Host
    USBCLK_ALL = 0x1B    // Host + Device + OTG + AHB
  };

  LPC_USB->OTGClkCtrl = USBCLK_ALL;
  while ( (LPC_USB->OTGClkSt & USBCLK_ALL) != USBCLK_ALL ) {}

  // set portfunc: USB1 = host, USB2 = device
  LPC_USB->StCtrl = 0x3;
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, state);
}

uint32_t board_button_read(void) {
  return BUTTON_ACTIV_STATE == Chip_GPIO_GetPinState(LPC_GPIO, BUTTON_PORT, BUTTON_PIN);
}

int board_uart_read(uint8_t *buf, int len) {
  //return UART_ReceiveByte(BOARD_UART_PORT);
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const *buf, int len) {
  //UART_Send(BOARD_UART_PORT, &c, 1, BLOCKING);
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
