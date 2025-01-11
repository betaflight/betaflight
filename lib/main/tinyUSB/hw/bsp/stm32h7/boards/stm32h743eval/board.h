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

#include "mfxstm32l152.h"

// Need to change jumper setting J7 and J8 from RS-232 to STLink
#define UART_DEV              USART1
#define UART_CLK_EN           __HAL_RCC_USART1_CLK_ENABLE

// VBUS Sense detection
#define OTG_FS_VBUS_SENSE     1
#define OTG_HS_VBUS_SENSE     0

// USB HS External PHY Pin: CLK, STP, DIR, NXT, D0-D7
#define ULPI_PINS \
  {GPIOA, GPIO_PIN_3 }, {GPIOA, GPIO_PIN_5 }, {GPIOB, GPIO_PIN_0 }, {GPIOB, GPIO_PIN_1 }, \
  {GPIOB, GPIO_PIN_5 }, {GPIOB, GPIO_PIN_10}, {GPIOB, GPIO_PIN_11}, {GPIOB, GPIO_PIN_12}, \
  {GPIOB, GPIO_PIN_13}, {GPIOC, GPIO_PIN_0 }, {GPIOH, GPIO_PIN_4 }, {GPIOI, GPIO_PIN_11}

#define PINID_LED      0
#define PINID_BUTTON   1
#define PINID_UART_TX  2
#define PINID_UART_RX  3

static board_pindef_t board_pindef[] = {
  { // LED
    .port = GPIOA,
    .pin_init = { .Pin = GPIO_PIN_4, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 },
    .active_state = 1
  },
  { // Button
    .port = GPIOC,
    .pin_init = { .Pin = GPIO_PIN_13, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 },
    .active_state = 0
  },
  { // UART TX
    .port = GPIOB,
    .pin_init = { .Pin = GPIO_PIN_14, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF4_USART1 },
    .active_state = 0
  },
  { // UART RX
    .port = GPIOB,
    .pin_init = { .Pin = GPIO_PIN_15, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF4_USART1 },
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
static inline void SystemClock_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ( (PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY ) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  // PLL1 for System Clock (400Mhz)
  // From H743 eval manual ETM can only work at 50 MHz clock by default because ETM signals
  // are shared with other peripherals. Trace CLK = PLL1R.
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 6; // Trace clock is 400/6 = 66.67 MHz (larger than 50 MHz but work well)
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);


  /* Select PLL as system clock source and configure bus clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  /* PLL3 for USB Clock */
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 336;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 7;
  PeriphClkInitStruct.PLL3.PLL3R = 2;

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /*activate CSI clock mondatory for I/O Compensation Cell*/
  __HAL_RCC_CSI_ENABLE();

  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();
}

//--------------------------------------------------------------------+
// MFX
//--------------------------------------------------------------------+
static I2C_HandleTypeDef i2c_handle = {
  .Instance = I2C1,
  .Init = {
    .Timing = 0x10C0ECFF,
    .OwnAddress1 = 0,
    .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    .DualAddressMode = I2C_DUALADDRESS_DISABLE,
    .OwnAddress2 = 0,
    .OwnAddress2Masks = I2C_OA2_NOMASK,
    .GeneralCallMode = I2C_GENERALCALL_DISABLE,
    .NoStretchMode = I2C_NOSTRETCH_DISABLE,
  }
};
static MFXSTM32L152_Object_t  mfx_obj = { 0 };
static MFXSTM32L152_IO_Mode_t* mfx_io = NULL;
static uint32_t mfx_vbus_pin[2] = { MFXSTM32L152_GPIO_PIN_7, MFXSTM32L152_GPIO_PIN_9 };

int32_t board_i2c_init(void) {
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  if (HAL_I2C_Init(&i2c_handle) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&i2c_handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&i2c_handle, 0) != HAL_OK) {
    return HAL_ERROR;
  }
  return 0;
}

int32_t board_i2c_deinit(void) {
  return 0;
}

int32_t i2c_readreg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length) {
  TU_ASSERT (HAL_OK == HAL_I2C_Mem_Read(&i2c_handle, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 10000));
  return 0;
}

int32_t i2c_writereg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length) {
  TU_ASSERT(HAL_OK == HAL_I2C_Mem_Write(&i2c_handle, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 10000));
  return 0;
}

static inline void board_init2(void) {
  // IO control via MFX
  MFXSTM32L152_IO_t io_ctx;
  io_ctx.Init        = board_i2c_init;
  io_ctx.DeInit      = board_i2c_deinit;
  io_ctx.ReadReg     = i2c_readreg;
  io_ctx.WriteReg    = i2c_writereg;
  io_ctx.GetTick     = (MFXSTM32L152_GetTick_Func) HAL_GetTick;

  uint16_t i2c_addr[] = { 0x84, 0x86 };
  for(uint8_t i = 0U; i < 2U; i++) {
    uint32_t mfx_id;
    io_ctx.Address = i2c_addr[i];
    TU_ASSERT(MFXSTM32L152_RegisterBusIO(&mfx_obj, &io_ctx) == MFXSTM32L152_OK, );
    TU_ASSERT(MFXSTM32L152_ReadID(&mfx_obj, &mfx_id) == MFXSTM32L152_OK, );
    if ((mfx_id == MFXSTM32L152_ID) || (mfx_id == MFXSTM32L152_ID_2)) {
      TU_ASSERT(MFXSTM32L152_Init(&mfx_obj) == MFXSTM32L152_OK, );
      break;
    }
  }

  mfx_io = &MFXSTM32L152_IO_Driver;
  mfx_io->IO_Start(&mfx_obj, MFXSTM32L152_GPIO_PINS_ALL);

  for(uint32_t i=0; i<2; i++) {
    MFXSTM32L152_IO_Init_t io_init = {
      .Pin = mfx_vbus_pin[i],
      .Mode = MFXSTM32L152_GPIO_MODE_OUTPUT_PP,
      .Pull = MFXSTM32L152_GPIO_PULLUP,
    };
    mfx_io->Init(&mfx_obj, &io_init);
  }
}

// VBUS1 is actually controlled by USB3320C PHY (using dwc2 drivebus signal)
void board_vbus_set(uint8_t rhport, bool state) {
  if (mfx_io) {
    mfx_io->IO_WritePin(&mfx_obj, mfx_vbus_pin[rhport], state);
  }
}

#ifdef __cplusplus
 }
#endif

#endif
