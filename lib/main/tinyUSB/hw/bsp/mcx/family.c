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

#include "bsp/board_api.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

#if CFG_TUSB_MCU == OPT_MCU_MCXN9
void USB0_FS_IRQHandler(void) {
  tusb_int_handler(0, true);
}

void USB1_HS_IRQHandler(void) {
  tusb_int_handler(1, true);
}

#elif CFG_TUSB_MCU == OPT_MCU_MCXA15

void USB0_IRQHandler(void) {
  tusb_int_handler(0, true);
}

#endif


void board_init(void) {
  BOARD_InitPins();
  BOARD_InitBootClocks();
  CLOCK_SetupExtClocking(XTAL0_CLK_HZ);

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  #if CFG_TUSB_MCU == OPT_MCU_MCXN9
  NVIC_SetPriority(USB0_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_SetPriority(USB1_HS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  #else
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  #endif
#endif

  // LED
  CLOCK_EnableClock(LED_CLK);
  gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0};
  GPIO_PinInit(LED_GPIO, LED_PIN, &led_config);
  board_led_write(0);

#ifdef NEOPIXEL_PIN
  // Neopixel
  static uint32_t pixelData[NEOPIXEL_NUMBER];
  IOCON_PinMuxSet(IOCON, NEOPIXEL_PORT, NEOPIXEL_PIN, IOCON_PIO_DIG_FUNC4_EN);

  sctpix_init(NEOPIXEL_TYPE);
  sctpix_addCh(NEOPIXEL_CH, pixelData, NEOPIXEL_NUMBER);
  sctpix_setPixel(NEOPIXEL_CH, 0, 0x100010);
  sctpix_setPixel(NEOPIXEL_CH, 1, 0x100010);
  sctpix_show();
#endif

  // Button
#ifdef BUTTON_GPIO
  CLOCK_EnableClock(BUTTON_CLK);
  gpio_pin_config_t const button_config = {kGPIO_DigitalInput, 0};
  GPIO_PinInit(BUTTON_GPIO, BUTTON_PIN, &button_config);
#endif

#ifdef UART_DEV
  // UART
//  IOCON_PinMuxSet(IOCON, UART_RX_PINMUX);
//  IOCON_PinMuxSet(IOCON, UART_TX_PINMUX);

  // Enable UART when debug log is on
  board_uart_init_clock();

  lpuart_config_t uart_config;
  LPUART_GetDefaultConfig(&uart_config);
  uart_config.baudRate_Bps = CFG_BOARD_UART_BAUDRATE;
  uart_config.enableTx = true;
  uart_config.enableRx = true;
  LPUART_Init(UART_DEV, &uart_config, 12000000u);
#endif

  // USB VBUS
  /* PORT0 PIN22 configured as USB0_VBUS */

#if defined(BOARD_TUD_RHPORT) && BOARD_TUD_RHPORT == 0
  // Port0 is Full Speed

  #if CFG_TUSB_MCU == OPT_MCU_MCXA15
  RESET_PeripheralReset(kUSB0_RST_SHIFT_RSTn);
  #elif CFG_TUSB_MCU == OPT_MCU_MCXN9
  CLOCK_AttachClk(kCLK_48M_to_USB0);
  CLOCK_EnableClock(kCLOCK_Usb0Ram);
  CLOCK_EnableClock(kCLOCK_Usb0Fs);
  #endif

  CLOCK_EnableUsbfsClock();
#endif

#if defined(BOARD_TUD_RHPORT) && BOARD_TUD_RHPORT == 1 && (CFG_TUSB_MCU == OPT_MCU_MCXN9)
  // Port1 is High Speed

  // Power
  SPC0->ACTIVE_VDELAY = 0x0500;
  /* Change the power DCDC to 1.8v (By default, DCDC is 1.8V), CORELDO to 1.1v (By default, CORELDO is 1.0V) */
  SPC0->ACTIVE_CFG &= ~SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK;
  SPC0->ACTIVE_CFG |= SPC_ACTIVE_CFG_DCDC_VDD_LVL(0x3) | SPC_ACTIVE_CFG_CORELDO_VDD_LVL(0x3) |
                      SPC_ACTIVE_CFG_SYSLDO_VDD_DS_MASK | SPC_ACTIVE_CFG_DCDC_VDD_DS(0x2u);
  /* Wait until it is done */
  while (SPC0->SC & SPC_SC_BUSY_MASK) {}
  if (0u == (SCG0->LDOCSR & SCG_LDOCSR_LDOEN_MASK)) {
    SCG0->TRIM_LOCK = 0x5a5a0001U;
    SCG0->LDOCSR |= SCG_LDOCSR_LDOEN_MASK;
    /* wait LDO ready */
    while (0U == (SCG0->LDOCSR & SCG_LDOCSR_VOUT_OK_MASK));
  }
  SYSCON->AHBCLKCTRLSET[2] |= SYSCON_AHBCLKCTRL2_USB_HS_MASK | SYSCON_AHBCLKCTRL2_USB_HS_PHY_MASK;
  SCG0->SOSCCFG &= ~(SCG_SOSCCFG_RANGE_MASK | SCG_SOSCCFG_EREFS_MASK);
  /* xtal = 20 ~ 30MHz */
  SCG0->SOSCCFG = (1U << SCG_SOSCCFG_RANGE_SHIFT) | (1U << SCG_SOSCCFG_EREFS_SHIFT);
  SCG0->SOSCCSR |= SCG_SOSCCSR_SOSCEN_MASK;
  while (1) {
    if (SCG0->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) {
      break;
    }
  }

  // Clock
  SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK | SYSCON_CLOCK_CTRL_CLKIN_ENA_FM_USBH_LPT_MASK;
  CLOCK_EnableClock(kCLOCK_UsbHs);
  CLOCK_EnableClock(kCLOCK_UsbHsPhy);
  CLOCK_EnableUsbhsPhyPllClock(kCLOCK_Usbphy480M, 24000000U);
  CLOCK_EnableUsbhsClock();

  // USB PHY
#if ((!(defined FSL_FEATURE_SOC_CCM_ANALOG_COUNT)) && (!(defined FSL_FEATURE_SOC_ANATOP_COUNT)))
  USBPHY->TRIM_OVERRIDE_EN = 0x001fU; /* override IFR value */
#endif

  // Enable PHY support for Low speed device + LS via FS Hub
  USBPHY->CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL2_MASK | USBPHY_CTRL_SET_ENUTMILEVEL3_MASK;

  // Enable all power for normal operation
  USBPHY->PWD = 0;

  // TX Timing
  uint32_t phytx = USBPHY->TX;
  phytx &= ~(USBPHY_TX_D_CAL_MASK | USBPHY_TX_TXCAL45DM_MASK | USBPHY_TX_TXCAL45DP_MASK);
  phytx |= USBPHY_TX_D_CAL(0x04) | USBPHY_TX_TXCAL45DP(0x07) | USBPHY_TX_TXCAL45DM(0x07);
  //phytx |= USBPHY_TX_D_CAL(0x0C) | USBPHY_TX_TXCAL45DP(0x06) | USBPHY_TX_TXCAL45DM(0x06);
  USBPHY->TX = phytx;
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_PinWrite(LED_GPIO, LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));

#ifdef NEOPIXEL_PIN
  if (state) {
    sctpix_setPixel(NEOPIXEL_CH, 0, 0x100000);
    sctpix_setPixel(NEOPIXEL_CH, 1, 0x101010);
  } else {
    sctpix_setPixel(NEOPIXEL_CH, 0, 0x001000);
    sctpix_setPixel(NEOPIXEL_CH, 1, 0x000010);
  }
  sctpix_show();
#endif
}

uint32_t board_button_read(void) {
#ifdef BUTTON_GPIO
  return BUTTON_STATE_ACTIVE == GPIO_PinRead(BUTTON_GPIO, BUTTON_PIN);
#endif
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
#ifdef UART_DEV
  LPUART_WriteBlocking(UART_DEV, (uint8_t const*) buf, len);
  return len;
#else
  (void) buf; (void) len;
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
