/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
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

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define LED_PIN               P1_1
#define LED_STATE_ON          1

#define BUTTON_PIN            P1_14
#define BUTTON_STATE_ACTIVE   0

//#define UART_DEV              USART6
//#define UART_CLK_EN           __HAL_RCC_USART6_CLK_ENABLE
//#define UART_GPIO_AF          GPIO_AF8_USART6
//
//#define UART_TX_PORT          GPIOC
//#define UART_TX_PIN           GPIO_PIN_6
//
//#define UART_RX_PORT          GPIOC
//#define UART_RX_PIN           GPIO_PIN_7

static inline void board_clock_init(void)
{
  /* Clock configuration */
  /* fPLL = 120MHz */
  /* fSYS = 120MHz */
  /* fUSBPLL = 192MHz */
  /* fUSB = 48MHz */
  const XMC_SCU_CLOCK_CONFIG_t clock_config =
  {
    .syspll_config.p_div  = 2,
    .syspll_config.n_div  = 80,
    .syspll_config.k_div  = 4,
    .syspll_config.mode   = XMC_SCU_CLOCK_SYSPLL_MODE_NORMAL,
    .syspll_config.clksrc = XMC_SCU_CLOCK_SYSPLLCLKSRC_OSCHP,
    .enable_oschp         = true,
    .calibration_mode     = XMC_SCU_CLOCK_FOFI_CALIBRATION_MODE_FACTORY,
    .fsys_clksrc          = XMC_SCU_CLOCK_SYSCLKSRC_PLL,
    .fsys_clkdiv          = 1,
    .fcpu_clkdiv          = 1,
    .fccu_clkdiv          = 1,
    .fperipheral_clkdiv   = 1
  };

  /* Setup settings for USB clock */
  XMC_SCU_CLOCK_Init(&clock_config);

  XMC_SCU_CLOCK_EnableUsbPll();
  XMC_SCU_CLOCK_StartUsbPll(2, 64);
  XMC_SCU_CLOCK_SetUsbClockDivider(4);
  XMC_SCU_CLOCK_SetUsbClockSource(XMC_SCU_CLOCK_USBCLKSRC_USBPLL);
  XMC_SCU_CLOCK_EnableClock(XMC_SCU_CLOCK_USB);
}


#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
