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

#define UART_DEV              USART3
#define UART_CLK_EN           __HAL_RCC_USART3_CLK_ENABLE

// VBUS Sense detection
#define OTG_FS_VBUS_SENSE     1
#define OTG_HS_VBUS_SENSE     0

// STM32F723 has only one USB HS peripheral
// Nucleo board does not have ULPI so USB will operate in FS mode only
// For the rest of the synopsys driver it is FS device however there
// is only USB_OTG_HS defined. Here are required conversions to
// make peripheral FS.
#define __HAL_RCC_USB2_OTG_FS_CLK_ENABLE __HAL_RCC_USB1_OTG_HS_CLK_ENABLE
#define GPIO_AF10_OTG2_HS               GPIO_AF10_OTG1_HS
#define USB_OTG_FS                      USB_OTG_HS

#define PINID_LED      0
#define PINID_BUTTON   1
#define PINID_UART_TX  2
#define PINID_UART_RX  3
#define PINID_VBUS0_EN 4

static board_pindef_t board_pindef[] = {
  { // LED
    .port = GPIOB,
    .pin_init = { .Pin = GPIO_PIN_0, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 },
    .active_state = 1
  },
  { // Button
    .port = GPIOC,
    .pin_init = { .Pin = GPIO_PIN_13, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 },
    .active_state = 1
  },
  { // UART TX
    .port = GPIOD,
    .pin_init = { .Pin = GPIO_PIN_8, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF7_USART3 },
    .active_state = 0
  },
  { // UART RX
    .port = GPIOD,
    .pin_init = { .Pin = GPIO_PIN_9, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF7_USART3 },
    .active_state = 0
  },
  { // VBUS0 EN
    .port = GPIOD,
    .pin_init = { .Pin = GPIO_PIN_10, .Mode = GPIO_MODE_OUTPUT_OD, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 },
    .active_state = 0
  }
};

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
static inline void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

  /* The PWR block is always enabled on the H7 series- there is no clock
     enable. For now, use the default VOS3 scale mode (lowest) and limit clock
     frequencies to avoid potential current draw problems from bus
     power when using the max clock speeds throughout the chip. */

  /* Enable HSE Oscillator and activate PLL1 with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = HSE_VALUE/1000000;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2; /* Unused */
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
    RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | \
    RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;

  /* Unlike on the STM32F4 family, it appears the maximum APB frequencies are
     device-dependent- 120 MHz for this board according to Figure 2 of
     the datasheet. Dividing by half will be safe for now. */
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  /* 4 wait states required for 168MHz and VOS3. */
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  /* Like on F4, on H7, USB's actual peripheral clock and bus clock are
     separate. However, the main system PLL (PLL1) doesn't have a direct
     connection to the USB peripheral clock to generate 48 MHz, so we do this
     dance. This will connect PLL1's Q output to the USB peripheral clock. */
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = { 0 };

  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  RCC_PeriphCLKInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
}

static inline void board_init2(void) {
  // For this board does nothing
}

void board_vbus_set(uint8_t rhport, bool state) {
  if (rhport == 0) {
    board_pindef_t* pindef = &board_pindef[PINID_VBUS0_EN];
    HAL_GPIO_WritePin(pindef->port, pindef->pin_init.Pin, state == pindef->active_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

#ifdef __cplusplus
 }
#endif

#endif
