/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021
 *    Ha Thach (tinyusb.org)
 *    Benjamin Evans
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

/* ** BOARD SETUP **
 *
 * NOTE: This board has bad signal integrity so you may experience some problems.
 * This setup assumes you have an openh743i-c Core and breakout board. For the HS
 * examples it also assumes you have a waveshare USB3300 breakout board plugged
 * into the ULPI PMOD header on the openh743i-c.
 *
 * UART Debugging:
 * Due to pin conflicts in the HS configuration, this BSP uses USART3 (PD8, PD9).
 * As such, you won't be able to use the UART to USB converter on board and will
 * require an external UART to USB converter. You could use the waveshare FT232
 * USB UART Board (micro) but any 3.3V UART to USB converter will be fine.
 *
 * Fullspeed:
 * If VBUS sense is enabled, ensure the PA9-VBUS jumper is connected on the core
 * board. Connect the PB6 jumper for the LED and the Wakeup - PA0 jumper for the
 * button. Connect the USB cable to the USB connector on the core board.
 *
 * High Speed:
 * Remove all jumpers from the openh743i-c (especially the USART1 jumpers as the
 * pins conflict). Connect the PB6 jumper for the LED and the Wakeup - PA0
 * jumper for the button.
 *
 * The reset pin on the ULPI PMOD port is not connected to the MCU. You'll need
 * to solder a wire from the RST pin on the USB3300 to a pin of your choosing on
 * the openh743i-c board (this example assumes you've used PD14 as specified with
 * the ULPI_RST_PORT and ULPI_RST_PIN defines below).
 *
 * Preferably power the board using the external 5VDC jack. Connect the USB cable
 * to the USB connector on the ULPI board. Adjust delays in this file as required.
 *
 * If you're having trouble, ask a question on the tinyUSB Github Discussion boards.
 *
 * Have fun!
 *
*/

#ifdef __cplusplus
 extern "C" {
#endif

// Need to change jumper setting J7 and J8 from RS-232 to STLink
#define UART_DEV              USART3
#define UART_CLK_EN           __HAL_RCC_USART3_CLK_ENABLE

// VBUS Sense detection
#define OTG_FS_VBUS_SENSE     1
#define OTG_HS_VBUS_SENSE     0

 // USB HS External PHY Pin: CLK, STP, DIR, NXT, D0-D7
#define ULPI_PINS \
  {GPIOA, GPIO_PIN_3 }, {GPIOA, GPIO_PIN_5 }, {GPIOB, GPIO_PIN_0 }, {GPIOB, GPIO_PIN_1 }, \
  {GPIOB, GPIO_PIN_5 }, {GPIOB, GPIO_PIN_10}, {GPIOB, GPIO_PIN_11}, {GPIOB, GPIO_PIN_12}, \
  {GPIOB, GPIO_PIN_13}, {GPIOC, GPIO_PIN_0 }, {GPIOC, GPIO_PIN_2 }, {GPIOC, GPIO_PIN_3 }

// ULPI PHY reset pin used by walkaround
#define ULPI_RST_PORT GPIOD
#define ULPI_RST_PIN GPIO_PIN_14

#define PINID_LED      0
#define PINID_BUTTON   1
#define PINID_UART_TX  2
#define PINID_UART_RX  3

static board_pindef_t board_pindef[] = {
  { // LED
    .port = GPIOB,
    .pin_init = { .Pin = GPIO_PIN_6, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 },
    .active_state = 1
  },
  { // Button
    .port = GPIOA,
    .pin_init = { .Pin = GPIO_PIN_0, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 },
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

  { // I2C SCL for MFX VBUS
    .port = GPIOB,
    .pin_init = { .Pin = GPIO_PIN_6, .Mode = GPIO_MODE_AF_OD, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF4_I2C1 },
    .active_state = 0
  },
  { // I2C SDA for MFX VBUS
    .port = GPIOB,
    .pin_init = { .Pin = GPIO_PIN_7, .Mode = GPIO_MODE_AF_OD, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF4_I2C1 },
    .active_state = 1
  },
};

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
static inline void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  // Supply configuration update enable
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  // Configure the main internal regulator output voltage
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }
  // Macro to configure the PLL clock source
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  // Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.PLL3.PLL3M = 8;
  PeriphClkInitStruct.PLL3.PLL3N = 336;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 7;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  __HAL_RCC_CSI_ENABLE();

  // Enable SYSCFG clock mondatory for I/O Compensation Cell
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  // Enables the I/O Compensation Cell
  HAL_EnableCompensationCell();

  // Enable voltage detector
  HAL_PWREx_EnableUSBVoltageDetector();
}

static inline void timer_board_delay(TIM_HandleTypeDef* tim_hdl, uint32_t ms)
{
  uint32_t startMs = __HAL_TIM_GET_COUNTER(tim_hdl);
  while ((__HAL_TIM_GET_COUNTER(tim_hdl) - startMs) < ms) {
    asm("nop"); //do nothing
  }
}

static inline void board_init2(void)
{
  // walkaround for resetting the ULPI PHY using Timer since systick is not
  // available when RTOS is used.

  // Init timer
  TIM_HandleTypeDef tim2Handle;
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  GPIO_InitTypeDef  GPIO_InitStruct;

  // ULPI_RST
  GPIO_InitStruct.Pin   = ULPI_RST_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = 0;
  HAL_GPIO_Init(ULPI_RST_PORT, &GPIO_InitStruct);

  __HAL_RCC_TIM2_CLK_ENABLE();

  //Assuming timer clock is running at 260Mhz this should configure the timer counter to 1000Hz
  tim2Handle.Instance = TIM2;
  tim2Handle.Init.Prescaler = 60000U - 1U;
  tim2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim2Handle.Init.Period = 0xFFFFFFFFU;
  tim2Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  tim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&tim2Handle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&tim2Handle, &sClockSourceConfig);

  //Start the timer
  HAL_TIM_Base_Start(&tim2Handle);

  // Reset PHY, change the delays as you see fit
  timer_board_delay(&tim2Handle, 5U);
  HAL_GPIO_WritePin(ULPI_RST_PORT, ULPI_RST_PIN, GPIO_PIN_SET);
  timer_board_delay(&tim2Handle, 20U);
  HAL_GPIO_WritePin(ULPI_RST_PORT, ULPI_RST_PIN, GPIO_PIN_RESET);
  timer_board_delay(&tim2Handle, 20U);

  //Disable the timer used for delays
  HAL_TIM_Base_Stop(&tim2Handle);
  __HAL_RCC_TIM2_CLK_DISABLE();
}

// need to short a jumper
void board_vbus_set(uint8_t rhport, bool state) {
  (void) rhport; (void) state;
}

#ifdef __cplusplus
 }
#endif

#endif
