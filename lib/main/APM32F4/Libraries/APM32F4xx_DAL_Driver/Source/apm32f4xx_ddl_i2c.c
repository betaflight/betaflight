/**
  *
  * @file    apm32f4xx_ddl_i2c.c
  * @brief   I2C DDL module driver.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_i2c.h"
#include "apm32f4xx_ddl_bus.h"
#include "apm32f4xx_ddl_rcm.h"
#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (I2C1) || defined (I2C2) || defined (I2C3)

/** @defgroup I2C_DDL I2C
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup I2C_DDL_Private_Macros
  * @{
  */

#define IS_DDL_I2C_PERIPHERAL_MODE(__VALUE__)    (((__VALUE__) == DDL_I2C_MODE_I2C)          || \
                                                 ((__VALUE__) == DDL_I2C_MODE_SMBUS_HOST)   || \
                                                 ((__VALUE__) == DDL_I2C_MODE_SMBUS_DEVICE) || \
                                                 ((__VALUE__) == DDL_I2C_MODE_SMBUS_DEVICE_ARP))

#define IS_DDL_I2C_CLOCK_SPEED(__VALUE__)           (((__VALUE__) > 0U) && ((__VALUE__) <= DDL_I2C_MAX_SPEED_FAST))

#define IS_DDL_I2C_DUTY_CYCLE(__VALUE__)            (((__VALUE__) == DDL_I2C_DUTYCYCLE_2) || \
                                                 ((__VALUE__) == DDL_I2C_DUTYCYCLE_16_9))

#if  defined(I2C_FILTER_ANFDIS)&&defined(I2C_FILTER_DNFCFG)
#define IS_DDL_I2C_ANALOG_FILTER(__VALUE__)      (((__VALUE__) == DDL_I2C_ANALOGFILTER_ENABLE) || \
                                                 ((__VALUE__) == DDL_I2C_ANALOGFILTER_DISABLE))

#define IS_DDL_I2C_DIGITAL_FILTER(__VALUE__)     ((__VALUE__) <= 0x0000000FU)

#endif
#define IS_DDL_I2C_OWN_ADDRESS1(__VALUE__)       ((__VALUE__) <= 0x000003FFU)

#define IS_DDL_I2C_TYPE_ACKNOWLEDGE(__VALUE__)   (((__VALUE__) == DDL_I2C_ACK) || \
                                                 ((__VALUE__) == DDL_I2C_NACK))

#define IS_DDL_I2C_OWN_ADDRSIZE(__VALUE__)       (((__VALUE__) == DDL_I2C_OWNADDRESS1_7BIT) || \
                                                 ((__VALUE__) == DDL_I2C_OWNADDRESS1_10BIT))
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_DDL_Exported_Functions
  * @{
  */

/** @addtogroup I2C_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the I2C registers to their default reset values.
  * @param  I2Cx I2C Instance.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS  I2C registers are de-initialized
  *          - ERROR  I2C registers are not de-initialized
  */
uint32_t DDL_I2C_DeInit(I2C_TypeDef *I2Cx)
{
  ErrorStatus status = SUCCESS;

  /* Check the I2C Instance I2Cx */
  ASSERT_PARAM(IS_I2C_ALL_INSTANCE(I2Cx));

  if (I2Cx == I2C1)
  {
    /* Force reset of I2C clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_I2C1);

    /* Release reset of I2C clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_I2C1);
  }
  else if (I2Cx == I2C2)
  {
    /* Force reset of I2C clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_I2C2);

    /* Release reset of I2C clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_I2C2);

  }
#if defined(I2C3)
  else if (I2Cx == I2C3)
  {
    /* Force reset of I2C clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_I2C3);

    /* Release reset of I2C clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_I2C3);
  }
#endif
  else
  {
    status = ERROR;
  }

  return status;
}

/**
  * @brief  Initialize the I2C registers according to the specified parameters in I2C_InitStruct.
  * @param  I2Cx I2C Instance.
  * @param  I2C_InitStruct pointer to a @ref DDL_I2C_InitTypeDef structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS  I2C registers are initialized
  *          - ERROR  Not applicable
  */
uint32_t DDL_I2C_Init(I2C_TypeDef *I2Cx, DDL_I2C_InitTypeDef *I2C_InitStruct)
{
  DDL_RCM_ClocksTypeDef rcc_clocks;

  /* Check the I2C Instance I2Cx */
  ASSERT_PARAM(IS_I2C_ALL_INSTANCE(I2Cx));

  /* Check the I2C parameters from I2C_InitStruct */
  ASSERT_PARAM(IS_DDL_I2C_PERIPHERAL_MODE(I2C_InitStruct->PeripheralMode));
  ASSERT_PARAM(IS_DDL_I2C_CLOCK_SPEED(I2C_InitStruct->ClockSpeed));
  ASSERT_PARAM(IS_DDL_I2C_DUTY_CYCLE(I2C_InitStruct->DutyCycle));
#if  defined(I2C_FILTER_ANFDIS)&&defined(I2C_FILTER_DNFCFG)
  ASSERT_PARAM(IS_DDL_I2C_ANALOG_FILTER(I2C_InitStruct->AnalogFilter));
  ASSERT_PARAM(IS_DDL_I2C_DIGITAL_FILTER(I2C_InitStruct->DigitalFilter));
#endif
  ASSERT_PARAM(IS_DDL_I2C_OWN_ADDRESS1(I2C_InitStruct->OwnAddress1));
  ASSERT_PARAM(IS_DDL_I2C_TYPE_ACKNOWLEDGE(I2C_InitStruct->TypeAcknowledge));
  ASSERT_PARAM(IS_DDL_I2C_OWN_ADDRSIZE(I2C_InitStruct->OwnAddrSize));

  /* Disable the selected I2Cx Peripheral */
  DDL_I2C_Disable(I2Cx);

  /* Retrieve Clock frequencies */
  DDL_RCM_GetSystemClocksFreq(&rcc_clocks);

#if  defined(I2C_FILTER_ANFDIS)&&defined(I2C_FILTER_DNFCFG)
  /*---------------------------- I2Cx FLTR Configuration -----------------------
   * Configure the analog and digital noise filters with parameters :
   * - AnalogFilter: I2C_FLTR_ANFOFF bit
   * - DigitalFilter: I2C_FILTER_DNFCFG[3:0] bits
   */
  DDL_I2C_ConfigFilters(I2Cx, I2C_InitStruct->AnalogFilter, I2C_InitStruct->DigitalFilter);

#endif
  /*---------------------------- I2Cx SCL Clock Speed Configuration ------------
   * Configure the SCL speed :
   * - ClockSpeed: I2C_CTRL2_CLKFCFG[5:0], I2C_RISETMAX_RISETMAX[5:0], I2C_CLKCTRL_SPEEDCFG,
   *           and I2C_CLKCTRL_CLKS[11:0] bits
   * - DutyCycle: I2C_CLKCTRL_FDUTYCFG[7:0] bits
   */
  DDL_I2C_ConfigSpeed(I2Cx, rcc_clocks.PCLK1_Frequency, I2C_InitStruct->ClockSpeed, I2C_InitStruct->DutyCycle);

  /*---------------------------- I2Cx OAR1 Configuration -----------------------
   * Disable, Configure and Enable I2Cx device own address 1 with parameters :
   * - OwnAddress1:  I2C_SADDR1_ADDR[9:8], I2C_SADDR1_ADDR[7:1] and I2C_SADDR1_ADDR0 bits
   * - OwnAddrSize:  I2C_SADDR1_ADDRLEN bit
   */
  DDL_I2C_SetOwnAddress1(I2Cx, I2C_InitStruct->OwnAddress1, I2C_InitStruct->OwnAddrSize);

  /*---------------------------- I2Cx MODE Configuration -----------------------
  * Configure I2Cx peripheral mode with parameter :
   * - PeripheralMode: I2C_CTRL1_SMBEN, I2C_CTRL1_SMBTCFG and I2C_CTRL1_ARPEN bits
   */
  DDL_I2C_SetMode(I2Cx, I2C_InitStruct->PeripheralMode);

  /* Enable the selected I2Cx Peripheral */
  DDL_I2C_Enable(I2Cx);

  /*---------------------------- I2Cx CTRL2 Configuration ------------------------
   * Configure the ACKnowledge or Non ACKnowledge condition
   * after the address receive match code or next received byte with parameter :
   * - TypeAcknowledge: I2C_CTRL2_NACK bit
   */
  DDL_I2C_AcknowledgeNextData(I2Cx, I2C_InitStruct->TypeAcknowledge);

  return SUCCESS;
}

/**
  * @brief  Set each @ref DDL_I2C_InitTypeDef field to default value.
  * @param  I2C_InitStruct Pointer to a @ref DDL_I2C_InitTypeDef structure.
  * @retval None
  */
void DDL_I2C_StructInit(DDL_I2C_InitTypeDef *I2C_InitStruct)
{
  /* Set I2C_InitStruct fields to default values */
  I2C_InitStruct->PeripheralMode  = DDL_I2C_MODE_I2C;
  I2C_InitStruct->ClockSpeed      = 5000U;
  I2C_InitStruct->DutyCycle       = DDL_I2C_DUTYCYCLE_2;
#if  defined(I2C_FILTER_ANFDIS)&&defined(I2C_FILTER_DNFCFG)
  I2C_InitStruct->AnalogFilter    = DDL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct->DigitalFilter   = 0U;
#endif
  I2C_InitStruct->OwnAddress1     = 0U;
  I2C_InitStruct->TypeAcknowledge = DDL_I2C_NACK;
  I2C_InitStruct->OwnAddrSize     = DDL_I2C_OWNADDRESS1_7BIT;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* I2C1 || I2C2 || I2C3 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

