/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
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

#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_power.h"
#include "fsl_iocon.h"
#include "fsl_usart.h"

#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

// IOCON pin mux
#define IOCON_PIO_DIGITAL_EN     0x80u   // Enables digital function
#define IOCON_PIO_FUNC0          0x00u
#define IOCON_PIO_FUNC1          0x01u   // Selects pin function 1
#define IOCON_PIO_FUNC7          0x07u   // Selects pin function 7
#define IOCON_PIO_INPFILT_OFF    0x0100u // Input filter disabled
#define IOCON_PIO_INV_DI         0x00u   // Input function is not inverted
#define IOCON_PIO_MODE_INACT     0x00u   // No addition pin function
#define IOCON_PIO_MODE_PULLUP    0x10u
#define IOCON_PIO_OPENDRAIN_DI   0x00u   // Open drain is disabled
#define IOCON_PIO_SLEW_STANDARD  0x00u   // Standard mode, output slew rate control is enabled

// Digital pin function n enabled
#define IOCON_PIO_DIG_FUNC0_EN   (IOCON_PIO_DIGITAL_EN | IOCON_PIO_INPFILT_OFF | IOCON_PIO_FUNC0)
#define IOCON_PIO_DIG_FUNC1_EN   (IOCON_PIO_DIGITAL_EN | IOCON_PIO_INPFILT_OFF | IOCON_PIO_FUNC1)
#define IOCON_PIO_DIG_FUNC4_EN   (IOCON_PIO_DIGITAL_EN | IOCON_PIO_INPFILT_OFF | IOCON_PIO_FUNC4)
#define IOCON_PIO_DIG_FUNC7_EN   (IOCON_PIO_DIGITAL_EN | IOCON_PIO_INPFILT_OFF | IOCON_PIO_FUNC7)

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void) {
  tusb_int_handler(0, true);
}

void USB1_IRQHandler(void) {
  tusb_int_handler(1, true);
}

/****************************************************************
name: BOARD_BootClockFROHF96M
outputs:
- {id: SYSTICK_clock.outFreq, value: 96 MHz}
- {id: System_clock.outFreq, value: 96 MHz}
settings:
- {id: SYSCON.MAINCLKSELA.sel, value: SYSCON.fro_hf}
sources:
- {id: SYSCON.fro_hf.outFreq, value: 96 MHz}
******************************************************************/
void BootClockFROHF96M(void) {
  /*!< Set up the clock sources */
  /*!< Set up FRO */
  POWER_DisablePD(kPDRUNCFG_PD_FRO_EN); /*!< Ensure FRO is on  */
  CLOCK_AttachClk(kFRO12M_to_MAIN_CLK); /*!< Switch to FRO 12MHz first to ensure we can change voltage without
                                             accidentally being below the voltage for current speed */
  POWER_SetVoltageForFreq(96000000U); /*!< Set voltage for the one of the fastest clock outputs: System clock output */
  CLOCK_SetFLASHAccessCyclesForFreq(96000000U); /*!< Set FLASH wait states for core */

  CLOCK_SetupFROClocking(96000000U); /*!< Set up high frequency FRO output to selected frequency */

  /*!< Set up dividers */
  CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);     /*!< Set AHBCLKDIV divider to value 1 */

  /*!< Set up clock selectors - Attach clocks to the peripheries */
  CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK); /*!< Switch MAIN_CLK to FRO_HF */

  /*!< Set SystemCoreClock variable. */
  SystemCoreClock = 96000000U;
}

void board_init(void) {
  // Enable IOCON clock
  CLOCK_EnableClock(kCLOCK_Iocon);

  // Init 96 MHz clock
  BootClockFROHF96M();

  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

#if CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  // Init all GPIO ports 54114 only has 2 ports.
  GPIO_PortInit(GPIO, LED_PORT);
  GPIO_PortInit(GPIO, BUTTON_PORT);

  // LED
  IOCON_PinMuxSet(IOCON, LED_PORT, LED_PIN, IOCON_PIO_DIG_FUNC0_EN);
  gpio_pin_config_t const led_config = { kGPIO_DigitalOutput, 0};
  GPIO_PinInit(GPIO, LED_PORT, LED_PIN, &led_config);

  board_led_write(0);

  // Button
  IOCON_PinMuxSet(IOCON, BUTTON_PORT, BUTTON_PIN, IOCON_PIO_DIG_FUNC0_EN | IOCON_PIO_MODE_PULLUP);
  gpio_pin_config_t const button_config = { kGPIO_DigitalInput, 0};
  GPIO_PinInit(GPIO, BUTTON_PORT, BUTTON_PIN, &button_config);

#ifdef UART_DEV
  // UART
  IOCON_PinMuxSet(IOCON, UART_RX_PINMUX);
  IOCON_PinMuxSet(IOCON, UART_TX_PINMUX);

  // Enable UART when debug log is on
  CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
  usart_config_t uart_config;
  USART_GetDefaultConfig(&uart_config);
  uart_config.baudRate_Bps = CFG_BOARD_UART_BAUDRATE;
  uart_config.enableTx     = true;
  uart_config.enableRx     = true;
  USART_Init(UART_DEV, &uart_config, 12000000);
#endif

  // USB
  IOCON_PinMuxSet(IOCON, USB0_VBUS_PINMUX);

#if defined(FSL_FEATURE_SOC_USBHSD_COUNT) && FSL_FEATURE_SOC_USBHSD_COUNT
  // LPC546xx and LPC540xx has OTG 1 FS + 1 HS rhports
  #if defined(BOARD_TUD_RHPORT) && BOARD_TUD_RHPORT == 0
    // Port0 is Full Speed
    POWER_DisablePD(kPDRUNCFG_PD_USB0_PHY); /*< Turn on USB Phy */
    CLOCK_SetClkDiv(kCLOCK_DivUsb0Clk, 1, false);
    CLOCK_AttachClk(kFRO_HF_to_USB0_CLK);

    /*According to reference manual, device mode setting has to be set by access usb host register */
    CLOCK_EnableClock(kCLOCK_Usbhsl0); /* enable usb0 host clock */
    USBFSH->PORTMODE |= USBFSH_PORTMODE_DEV_ENABLE_MASK;
    CLOCK_DisableClock(kCLOCK_Usbhsl0); /* disable usb0 host clock */

    CLOCK_EnableUsbfs0DeviceClock(kCLOCK_UsbSrcFro, CLOCK_GetFroHfFreq());
  #endif

  #if defined(BOARD_TUD_RHPORT) && BOARD_TUD_RHPORT == 1
    // Port1 is High Speed
    POWER_DisablePD(kPDRUNCFG_PD_USB1_PHY);

    /*According to reference manual, device mode setting has to be set by access usb host register */
    CLOCK_EnableClock(kCLOCK_Usbh1); /* enable usb1 host clock */
    USBHSH->PORTMODE |= USBHSH_PORTMODE_DEV_ENABLE_MASK;
    CLOCK_DisableClock(kCLOCK_Usbh1); /* enable usb1 host clock */

    CLOCK_EnableUsbhs0DeviceClock(kCLOCK_UsbSrcUsbPll, 0U);
  #endif
#else
  // LPC5411x series only has full speed device
  POWER_DisablePD(kPDRUNCFG_PD_USB0_PHY); // Turn on USB Phy
  CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcFro, CLOCK_GetFreq(kCLOCK_FroHf)); /* enable USB IP clock */
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_PinWrite(GPIO, LED_PORT, LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
}

uint32_t board_button_read(void) {
  // active low
  return BUTTON_STATE_ACTIVE == GPIO_PinRead(GPIO, BUTTON_PORT, BUTTON_PIN);
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  USART_WriteBlocking(UART_DEV, (uint8_t const*) buf, len);
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
