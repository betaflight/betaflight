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
#include "fsl_lpsci.h"

#include "clock_config.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void)
{
#if CFG_TUH_ENABLED
  tuh_int_handler(0, true);
#endif
#if CFG_TUD_ENABLED
  tud_int_handler(0);
#endif
}

void board_init(void)
{
  BOARD_BootClockRUN();
  SystemCoreClockUpdate();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  // LED
  CLOCK_EnableClock(LED_PIN_CLOCK);
  PORT_SetPinMux(LED_PIN_PORT, LED_PIN, LED_PIN_FUNCTION);
  gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 0 };
  GPIO_PinInit(LED_PORT, LED_PIN, &led_config);
  board_led_write(false);

#if defined(BUTTON_PORT) && defined(BUTTON_PIN)
  // Button
  CLOCK_EnableClock(BUTTON_PIN_CLOCK);
  port_pin_config_t button_port = {
    .pullSelect = kPORT_PullUp,
    .mux = BUTTON_PIN_FUNCTION,
  };
  PORT_SetPinConfig(BUTTON_PIN_PORT, BUTTON_PIN, &button_port);
  gpio_pin_config_t button_config = { kGPIO_DigitalInput, 0 };
  GPIO_PinInit(BUTTON_PORT, BUTTON_PIN, &button_config);
#endif

  // UART
  CLOCK_EnableClock(UART_PIN_CLOCK);
  PORT_SetPinMux(UART_PIN_PORT, UART_PIN_RX, UART_PIN_FUNCTION);
  PORT_SetPinMux(UART_PIN_PORT, UART_PIN_TX, UART_PIN_FUNCTION);
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK)))
      | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)
      | SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX)
    );

  lpsci_config_t uart_config;
  CLOCK_SetLpsci0Clock(1);
  LPSCI_GetDefaultConfig(&uart_config);
  uart_config.baudRate_Bps = CFG_BOARD_UART_BAUDRATE;
  uart_config.enableTx = true;
  uart_config.enableRx = true;
  LPSCI_Init(UART_PORT, &uart_config, CLOCK_GetPllFllSelClkFreq());

  // USB
  CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  GPIO_PinWrite(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1-LED_STATE_ON));
}

uint32_t board_button_read(void)
{
#if defined(BUTTON_PORT) && defined(BUTTON_PIN)
  return BUTTON_STATE_ACTIVE == GPIO_PinRead(BUTTON_PORT, BUTTON_PIN);
#endif
  return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
  LPSCI_ReadBlocking(UART_PORT, buf, len);
  return len;
}

int board_uart_write(void const * buf, int len)
{
  LPSCI_WriteBlocking(UART_PORT, (uint8_t const*) buf, len);
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler(void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
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
