/**
  *
  * @file    apm32f4xx_ddl_tmr.c
  * @brief   TMR DDL module driver.
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
#include "apm32f4xx_ddl_tmr.h"
#include "apm32f4xx_ddl_bus.h"

#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif /* USE_FULL_ASSERT */

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (TMR1) || defined (TMR2) || defined (TMR3) || defined (TMR4) || defined (TMR5) || defined (TMR6) || defined (TMR7) || defined (TMR8) || defined (TMR9) || defined (TMR10) || defined (TMR11) || defined (TMR12) || defined (TMR13) || defined (TMR14)

/** @addtogroup TMR_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup TMR_DDL_Private_Macros
  * @{
  */
#define IS_DDL_TMR_COUNTERMODE(__VALUE__) (((__VALUE__) == DDL_TMR_COUNTERMODE_UP) \
                                          || ((__VALUE__) == DDL_TMR_COUNTERMODE_DOWN) \
                                          || ((__VALUE__) == DDL_TMR_COUNTERMODE_CENTER_UP) \
                                          || ((__VALUE__) == DDL_TMR_COUNTERMODE_CENTER_DOWN) \
                                          || ((__VALUE__) == DDL_TMR_COUNTERMODE_CENTER_UP_DOWN))

#define IS_DDL_TMR_CLOCKDIVISION(__VALUE__) (((__VALUE__) == DDL_TMR_CLOCKDIVISION_DIV1) \
                                            || ((__VALUE__) == DDL_TMR_CLOCKDIVISION_DIV2) \
                                            || ((__VALUE__) == DDL_TMR_CLOCKDIVISION_DIV4))

#define IS_DDL_TMR_OCMODE(__VALUE__) (((__VALUE__) == DDL_TMR_OCMODE_FROZEN) \
                                     || ((__VALUE__) == DDL_TMR_OCMODE_ACTIVE) \
                                     || ((__VALUE__) == DDL_TMR_OCMODE_INACTIVE) \
                                     || ((__VALUE__) == DDL_TMR_OCMODE_TOGGLE) \
                                     || ((__VALUE__) == DDL_TMR_OCMODE_FORCED_INACTIVE) \
                                     || ((__VALUE__) == DDL_TMR_OCMODE_FORCED_ACTIVE) \
                                     || ((__VALUE__) == DDL_TMR_OCMODE_PWM1) \
                                     || ((__VALUE__) == DDL_TMR_OCMODE_PWM2))

#define IS_DDL_TMR_OCSTATE(__VALUE__) (((__VALUE__) == DDL_TMR_OCSTATE_DISABLE) \
                                      || ((__VALUE__) == DDL_TMR_OCSTATE_ENABLE))

#define IS_DDL_TMR_OCPOLARITY(__VALUE__) (((__VALUE__) == DDL_TMR_OCPOLARITY_HIGH) \
                                         || ((__VALUE__) == DDL_TMR_OCPOLARITY_LOW))

#define IS_DDL_TMR_OCIDLESTATE(__VALUE__) (((__VALUE__) == DDL_TMR_OCIDLESTATE_LOW) \
                                          || ((__VALUE__) == DDL_TMR_OCIDLESTATE_HIGH))

#define IS_DDL_TMR_ACTIVEINPUT(__VALUE__) (((__VALUE__) == DDL_TMR_ACTIVEINPUT_DIRECTTI) \
                                          || ((__VALUE__) == DDL_TMR_ACTIVEINPUT_INDIRECTTI) \
                                          || ((__VALUE__) == DDL_TMR_ACTIVEINPUT_TRC))

#define IS_DDL_TMR_ICPSC(__VALUE__) (((__VALUE__) == DDL_TMR_ICPSC_DIV1) \
                                    || ((__VALUE__) == DDL_TMR_ICPSC_DIV2) \
                                    || ((__VALUE__) == DDL_TMR_ICPSC_DIV4) \
                                    || ((__VALUE__) == DDL_TMR_ICPSC_DIV8))

#define IS_DDL_TMR_IC_FILTER(__VALUE__) (((__VALUE__) == DDL_TMR_IC_FILTER_FDIV1) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV1_N2) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV1_N4) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV1_N8) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV2_N6) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV2_N8) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV4_N6) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV4_N8) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV8_N6) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV8_N8) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV16_N5) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV16_N6) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV16_N8) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV32_N5) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV32_N6) \
                                        || ((__VALUE__) == DDL_TMR_IC_FILTER_FDIV32_N8))

#define IS_DDL_TMR_IC_POLARITY(__VALUE__) (((__VALUE__) == DDL_TMR_IC_POLARITY_RISING) \
                                          || ((__VALUE__) == DDL_TMR_IC_POLARITY_FALLING) \
                                          || ((__VALUE__) == DDL_TMR_IC_POLARITY_BOTHEDGE))

#define IS_DDL_TMR_ENCODERMODE(__VALUE__) (((__VALUE__) == DDL_TMR_ENCODERMODE_X2_TI1) \
                                          || ((__VALUE__) == DDL_TMR_ENCODERMODE_X2_TI2) \
                                          || ((__VALUE__) == DDL_TMR_ENCODERMODE_X4_TI12))

#define IS_DDL_TMR_IC_POLARITY_ENCODER(__VALUE__) (((__VALUE__) == DDL_TMR_IC_POLARITY_RISING) \
                                                  || ((__VALUE__) == DDL_TMR_IC_POLARITY_FALLING))

#define IS_DDL_TMR_OSSR_STATE(__VALUE__) (((__VALUE__) == DDL_TMR_OSSR_DISABLE) \
                                         || ((__VALUE__) == DDL_TMR_OSSR_ENABLE))

#define IS_DDL_TMR_OSSI_STATE(__VALUE__) (((__VALUE__) == DDL_TMR_OSSI_DISABLE) \
                                         || ((__VALUE__) == DDL_TMR_OSSI_ENABLE))

#define IS_DDL_TMR_LOCK_LEVEL(__VALUE__) (((__VALUE__) == DDL_TMR_LOCKLEVEL_OFF) \
                                         || ((__VALUE__) == DDL_TMR_LOCKLEVEL_1)   \
                                         || ((__VALUE__) == DDL_TMR_LOCKLEVEL_2)   \
                                         || ((__VALUE__) == DDL_TMR_LOCKLEVEL_3))

#define IS_DDL_TMR_BREAK_STATE(__VALUE__) (((__VALUE__) == DDL_TMR_BREAK_DISABLE) \
                                          || ((__VALUE__) == DDL_TMR_BREAK_ENABLE))

#define IS_DDL_TMR_BREAK_POLARITY(__VALUE__) (((__VALUE__) == DDL_TMR_BREAK_POLARITY_LOW) \
                                             || ((__VALUE__) == DDL_TMR_BREAK_POLARITY_HIGH))

#define IS_DDL_TMR_AUTOMATIC_OUTPUT_STATE(__VALUE__) (((__VALUE__) == DDL_TMR_AUTOMATICOUTPUT_DISABLE) \
                                                     || ((__VALUE__) == DDL_TMR_AUTOMATICOUTPUT_ENABLE))
/**
  * @}
  */


/* Private function prototypes -----------------------------------------------*/
/** @defgroup TMR_DDL_Private_Functions TMR Private Functions
  * @{
  */
static ErrorStatus OC1Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC2Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC3Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus OC4Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct);
static ErrorStatus IC1Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct);
static ErrorStatus IC2Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct);
static ErrorStatus IC3Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct);
static ErrorStatus IC4Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup TMR_DDL_Exported_Functions
  * @{
  */

/** @addtogroup TMR_DDL_EF_Init
  * @{
  */

/**
  * @brief  Set TMRx registers to their reset values.
  * @param  TMRx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: invalid TMRx instance
  */
ErrorStatus DDL_TMR_DeInit(TMR_TypeDef *TMRx)
{
  ErrorStatus result = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(TMRx));

  if (TMRx == TMR1)
  {
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_TMR1);
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_TMR1);
  }
#if defined(TMR2)
  else if (TMRx == TMR2)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR2);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR2);
  }
#endif /* TMR2 */
#if defined(TMR3)
  else if (TMRx == TMR3)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR3);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR3);
  }
#endif /* TMR3 */
#if defined(TMR4)
  else if (TMRx == TMR4)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR4);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR4);
  }
#endif /* TMR4 */
#if defined(TMR5)
  else if (TMRx == TMR5)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR5);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR5);
  }
#endif /* TMR5 */
#if defined(TMR6)
  else if (TMRx == TMR6)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR6);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR6);
  }
#endif /* TMR6 */
#if defined (TMR7)
  else if (TMRx == TMR7)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR7);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR7);
  }
#endif /* TMR7 */
#if defined(TMR8)
  else if (TMRx == TMR8)
  {
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_TMR8);
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_TMR8);
  }
#endif /* TMR8 */
#if defined(TMR9)
  else if (TMRx == TMR9)
  {
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_TMR9);
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_TMR9);
  }
#endif /* TMR9 */
#if defined(TMR10)
  else if (TMRx == TMR10)
  {
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_TMR10);
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_TMR10);
  }
#endif /* TMR10 */
#if defined(TMR11)
  else if (TMRx == TMR11)
  {
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_TMR11);
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_TMR11);
  }
#endif /* TMR11 */
#if defined(TMR12)
  else if (TMRx == TMR12)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR12);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR12);
  }
#endif /* TMR12 */
#if defined(TMR13)
  else if (TMRx == TMR13)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR13);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR13);
  }
#endif /* TMR13 */
#if defined(TMR14)
  else if (TMRx == TMR14)
  {
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_TMR14);
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_TMR14);
  }
#endif /* TMR14 */
  else
  {
    result = ERROR;
  }

  return result;
}

/**
  * @brief  Set the fields of the time base unit configuration data structure
  *         to their default values.
  * @param  TMR_InitStruct pointer to a @ref DDL_TMR_InitTypeDef structure (time base unit configuration data structure)
  * @retval None
  */
void DDL_TMR_StructInit(DDL_TMR_InitTypeDef *TMR_InitStruct)
{
  /* Set the default configuration */
  TMR_InitStruct->Prescaler         = (uint16_t)0x0000;
  TMR_InitStruct->CounterMode       = DDL_TMR_COUNTERMODE_UP;
  TMR_InitStruct->Autoreload        = 0xFFFFFFFFU;
  TMR_InitStruct->ClockDivision     = DDL_TMR_CLOCKDIVISION_DIV1;
  TMR_InitStruct->RepetitionCounter = 0x00000000U;
}

/**
  * @brief  Configure the TMRx time base unit.
  * @param  TMRx Timer Instance
  * @param  TMR_InitStruct pointer to a @ref DDL_TMR_InitTypeDef structure
  *         (TMRx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_TMR_Init(TMR_TypeDef *TMRx, DDL_TMR_InitTypeDef *TMR_InitStruct)
{
  uint32_t tmpcr1;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_COUNTERMODE(TMR_InitStruct->CounterMode));
  ASSERT_PARAM(IS_DDL_TMR_CLOCKDIVISION(TMR_InitStruct->ClockDivision));

  tmpcr1 = DDL_TMR_ReadReg(TMRx, CTRL1);

  if (IS_TMR_COUNTER_MODE_SELECT_INSTANCE(TMRx))
  {
    /* Select the Counter Mode */
    MODIFY_REG(tmpcr1, (TMR_CTRL1_CNTDIR | TMR_CTRL1_CAMSEL), TMR_InitStruct->CounterMode);
  }

  if (IS_TMR_CLOCK_DIVISION_INSTANCE(TMRx))
  {
    /* Set the clock division */
    MODIFY_REG(tmpcr1, TMR_CTRL1_CLKDIV, TMR_InitStruct->ClockDivision);
  }

  /* Write to TMRx CTRL1 */
  DDL_TMR_WriteReg(TMRx, CTRL1, tmpcr1);

  /* Set the Autoreload value */
  DDL_TMR_SetAutoReload(TMRx, TMR_InitStruct->Autoreload);

  /* Set the Prescaler value */
  DDL_TMR_SetPrescaler(TMRx, TMR_InitStruct->Prescaler);

  if (IS_TMR_REPETITION_COUNTER_INSTANCE(TMRx))
  {
    /* Set the Repetition Counter value */
    DDL_TMR_SetRepetitionCounter(TMRx, TMR_InitStruct->RepetitionCounter);
  }

  /* Generate an update event to reload the Prescaler
     and the repetition counter value (if applicable) immediately */
  DDL_TMR_GenerateEvent_UPDATE(TMRx);

  return SUCCESS;
}

/**
  * @brief  Set the fields of the TMRx output channel configuration data
  *         structure to their default values.
  * @param  TMR_OC_InitStruct pointer to a @ref DDL_TMR_OC_InitTypeDef structure
  *         (the output channel configuration data structure)
  * @retval None
  */
void DDL_TMR_OC_StructInit(DDL_TMR_OC_InitTypeDef *TMR_OC_InitStruct)
{
  /* Set the default configuration */
  TMR_OC_InitStruct->OCMode       = DDL_TMR_OCMODE_FROZEN;
  TMR_OC_InitStruct->OCState      = DDL_TMR_OCSTATE_DISABLE;
  TMR_OC_InitStruct->OCNState     = DDL_TMR_OCSTATE_DISABLE;
  TMR_OC_InitStruct->CompareValue = 0x00000000U;
  TMR_OC_InitStruct->OCPolarity   = DDL_TMR_OCPOLARITY_HIGH;
  TMR_OC_InitStruct->OCNPolarity  = DDL_TMR_OCPOLARITY_HIGH;
  TMR_OC_InitStruct->OCIdleState  = DDL_TMR_OCIDLESTATE_LOW;
  TMR_OC_InitStruct->OCNIdleState = DDL_TMR_OCIDLESTATE_LOW;
}

/**
  * @brief  Configure the TMRx output channel.
  * @param  TMRx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  TMR_OC_InitStruct pointer to a @ref DDL_TMR_OC_InitTypeDef structure (TMRx output channel configuration
  *         data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx output channel is initialized
  *          - ERROR: TMRx output channel is not initialized
  */
ErrorStatus DDL_TMR_OC_Init(TMR_TypeDef *TMRx, uint32_t Channel, DDL_TMR_OC_InitTypeDef *TMR_OC_InitStruct)
{
  ErrorStatus result = ERROR;

  switch (Channel)
  {
    case DDL_TMR_CHANNEL_CH1:
      result = OC1Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_TMR_CHANNEL_CH2:
      result = OC2Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_TMR_CHANNEL_CH3:
      result = OC3Config(TMRx, TMR_OC_InitStruct);
      break;
    case DDL_TMR_CHANNEL_CH4:
      result = OC4Config(TMRx, TMR_OC_InitStruct);
      break;
    default:
      break;
  }

  return result;
}

/**
  * @brief  Set the fields of the TMRx input channel configuration data
  *         structure to their default values.
  * @param  TMR_ICInitStruct pointer to a @ref DDL_TMR_IC_InitTypeDef structure (the input channel configuration
  *         data structure)
  * @retval None
  */
void DDL_TMR_IC_StructInit(DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Set the default configuration */
  TMR_ICInitStruct->ICPolarity    = DDL_TMR_IC_POLARITY_RISING;
  TMR_ICInitStruct->ICActiveInput = DDL_TMR_ACTIVEINPUT_DIRECTTI;
  TMR_ICInitStruct->ICPrescaler   = DDL_TMR_ICPSC_DIV1;
  TMR_ICInitStruct->ICFilter      = DDL_TMR_IC_FILTER_FDIV1;
}

/**
  * @brief  Configure the TMRx input channel.
  * @param  TMRx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_TMR_CHANNEL_CH1
  *         @arg @ref DDL_TMR_CHANNEL_CH2
  *         @arg @ref DDL_TMR_CHANNEL_CH3
  *         @arg @ref DDL_TMR_CHANNEL_CH4
  * @param  TMR_IC_InitStruct pointer to a @ref DDL_TMR_IC_InitTypeDef structure (TMRx input channel configuration data
  *         structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx output channel is initialized
  *          - ERROR: TMRx output channel is not initialized
  */
ErrorStatus DDL_TMR_IC_Init(TMR_TypeDef *TMRx, uint32_t Channel, DDL_TMR_IC_InitTypeDef *TMR_IC_InitStruct)
{
  ErrorStatus result = ERROR;

  switch (Channel)
  {
    case DDL_TMR_CHANNEL_CH1:
      result = IC1Config(TMRx, TMR_IC_InitStruct);
      break;
    case DDL_TMR_CHANNEL_CH2:
      result = IC2Config(TMRx, TMR_IC_InitStruct);
      break;
    case DDL_TMR_CHANNEL_CH3:
      result = IC3Config(TMRx, TMR_IC_InitStruct);
      break;
    case DDL_TMR_CHANNEL_CH4:
      result = IC4Config(TMRx, TMR_IC_InitStruct);
      break;
    default:
      break;
  }

  return result;
}

/**
  * @brief  Fills each TMR_EncoderInitStruct field with its default value
  * @param  TMR_EncoderInitStruct pointer to a @ref DDL_TMR_ENCODER_InitTypeDef structure (encoder interface
  *         configuration data structure)
  * @retval None
  */
void DDL_TMR_ENCODER_StructInit(DDL_TMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct)
{
  /* Set the default configuration */
  TMR_EncoderInitStruct->EncoderMode    = DDL_TMR_ENCODERMODE_X2_TI1;
  TMR_EncoderInitStruct->IC1Polarity    = DDL_TMR_IC_POLARITY_RISING;
  TMR_EncoderInitStruct->IC1ActiveInput = DDL_TMR_ACTIVEINPUT_DIRECTTI;
  TMR_EncoderInitStruct->IC1Prescaler   = DDL_TMR_ICPSC_DIV1;
  TMR_EncoderInitStruct->IC1Filter      = DDL_TMR_IC_FILTER_FDIV1;
  TMR_EncoderInitStruct->IC2Polarity    = DDL_TMR_IC_POLARITY_RISING;
  TMR_EncoderInitStruct->IC2ActiveInput = DDL_TMR_ACTIVEINPUT_DIRECTTI;
  TMR_EncoderInitStruct->IC2Prescaler   = DDL_TMR_ICPSC_DIV1;
  TMR_EncoderInitStruct->IC2Filter      = DDL_TMR_IC_FILTER_FDIV1;
}

/**
  * @brief  Configure the encoder interface of the timer instance.
  * @param  TMRx Timer Instance
  * @param  TMR_EncoderInitStruct pointer to a @ref DDL_TMR_ENCODER_InitTypeDef structure (TMRx encoder interface
  *         configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_TMR_ENCODER_Init(TMR_TypeDef *TMRx, DDL_TMR_ENCODER_InitTypeDef *TMR_EncoderInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_ENCODERMODE(TMR_EncoderInitStruct->EncoderMode));
  ASSERT_PARAM(IS_DDL_TMR_IC_POLARITY_ENCODER(TMR_EncoderInitStruct->IC1Polarity));
  ASSERT_PARAM(IS_DDL_TMR_ACTIVEINPUT(TMR_EncoderInitStruct->IC1ActiveInput));
  ASSERT_PARAM(IS_DDL_TMR_ICPSC(TMR_EncoderInitStruct->IC1Prescaler));
  ASSERT_PARAM(IS_DDL_TMR_IC_FILTER(TMR_EncoderInitStruct->IC1Filter));
  ASSERT_PARAM(IS_DDL_TMR_IC_POLARITY_ENCODER(TMR_EncoderInitStruct->IC2Polarity));
  ASSERT_PARAM(IS_DDL_TMR_ACTIVEINPUT(TMR_EncoderInitStruct->IC2ActiveInput));
  ASSERT_PARAM(IS_DDL_TMR_ICPSC(TMR_EncoderInitStruct->IC2Prescaler));
  ASSERT_PARAM(IS_DDL_TMR_IC_FILTER(TMR_EncoderInitStruct->IC2Filter));

  /* Disable the CC1 and CC2: Reset the CC1EN and CC2EN Bits */
  TMRx->CCEN &= (uint32_t)~(TMR_CCEN_CC1EN | TMR_CCEN_CC2EN);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_TMR_ReadReg(TMRx, CCM1);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_TMR_ReadReg(TMRx, CCEN);

  /* Configure TI1 */
  tmpccmr1 &= (uint32_t)~(TMR_CCM1_CC1SEL | TMR_CCM1_IC1F  | TMR_CCM1_IC1PSC);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC1ActiveInput >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC1Filter >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC1Prescaler >> 16U);

  /* Configure TI2 */
  tmpccmr1 &= (uint32_t)~(TMR_CCM1_CC2SEL | TMR_CCM1_IC2F  | TMR_CCM1_IC2PSC);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC2ActiveInput >> 8U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC2Filter >> 8U);
  tmpccmr1 |= (uint32_t)(TMR_EncoderInitStruct->IC2Prescaler >> 8U);

  /* Set TI1 and TI2 polarity and enable TI1 and TI2 */
  tmpccer &= (uint32_t)~(TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL | TMR_CCEN_CC2POL | TMR_CCEN_CC2NPOL);
  tmpccer |= (uint32_t)(TMR_EncoderInitStruct->IC1Polarity);
  tmpccer |= (uint32_t)(TMR_EncoderInitStruct->IC2Polarity << 4U);
  tmpccer |= (uint32_t)(TMR_CCEN_CC1EN | TMR_CCEN_CC2EN);

  /* Set encoder mode */
  DDL_TMR_SetEncoderMode(TMRx, TMR_EncoderInitStruct->EncoderMode);

  /* Write to TMRx CCM1 */
  DDL_TMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Write to TMRx CCEN */
  DDL_TMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Set the fields of the TMRx Hall sensor interface configuration data
  *         structure to their default values.
  * @param  TMR_HallSensorInitStruct pointer to a @ref DDL_TMR_HALLSENSOR_InitTypeDef structure (HALL sensor interface
  *         configuration data structure)
  * @retval None
  */
void DDL_TMR_HALLSENSOR_StructInit(DDL_TMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct)
{
  /* Set the default configuration */
  TMR_HallSensorInitStruct->IC1Polarity       = DDL_TMR_IC_POLARITY_RISING;
  TMR_HallSensorInitStruct->IC1Prescaler      = DDL_TMR_ICPSC_DIV1;
  TMR_HallSensorInitStruct->IC1Filter         = DDL_TMR_IC_FILTER_FDIV1;
  TMR_HallSensorInitStruct->CommutationDelay  = 0U;
}

/**
  * @brief  Configure the Hall sensor interface of the timer instance.
  * @note TMRx CH1, CH2 and CH3 inputs connected through a XOR
  *       to the TI1 input channel
  * @note TMRx slave mode controller is configured in reset mode.
          Selected internal trigger is TI1F_ED.
  * @note Channel 1 is configured as input, IC1 is mapped on TRC.
  * @note Captured value stored in TMRx_CCR1 correspond to the time elapsed
  *       between 2 changes on the inputs. It gives information about motor speed.
  * @note Channel 2 is configured in output PWM 2 mode.
  * @note Compare value stored in TMRx_CCR2 corresponds to the commutation delay.
  * @note OC2REF is selected as trigger output on TRGO.
  * @note DDL_TMR_IC_POLARITY_BOTHEDGE must not be used for TI1 when it is used
  *       when TMRx operates in Hall sensor interface mode.
  * @param  TMRx Timer Instance
  * @param  TMR_HallSensorInitStruct pointer to a @ref DDL_TMR_HALLSENSOR_InitTypeDef structure (TMRx DALL sensor
  *         interface configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_TMR_HALLSENSOR_Init(TMR_TypeDef *TMRx, DDL_TMR_HALLSENSOR_InitTypeDef *TMR_HallSensorInitStruct)
{
  uint32_t tmpcr2;
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_IC_POLARITY_ENCODER(TMR_HallSensorInitStruct->IC1Polarity));
  ASSERT_PARAM(IS_DDL_TMR_ICPSC(TMR_HallSensorInitStruct->IC1Prescaler));
  ASSERT_PARAM(IS_DDL_TMR_IC_FILTER(TMR_HallSensorInitStruct->IC1Filter));

  /* Disable the CC1 and CC2: Reset the CC1E and CC2E Bits */
  TMRx->CCEN &= (uint32_t)~(TMR_CCEN_CC1EN | TMR_CCEN_CC2EN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_TMR_ReadReg(TMRx, CTRL2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_TMR_ReadReg(TMRx, CCM1);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_TMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx SMCTRL register value */
  tmpsmcr = DDL_TMR_ReadReg(TMRx, SMCTRL);

  /* Connect TMRx_CH1, CH2 and CH3 pins to the TI1 input */
  tmpcr2 |= TMR_CTRL2_TI1SEL;

  /* OC2REF signal is used as trigger output (TRGO) */
  tmpcr2 |= DDL_TMR_TRGO_OC2REF;

  /* Configure the slave mode controller */
  tmpsmcr &= (uint32_t)~(TMR_SMCTRL_TRGSEL | TMR_SMCTRL_SMFSEL);
  tmpsmcr |= DDL_TMR_TS_TI1F_ED;
  tmpsmcr |= DDL_TMR_SLAVEMODE_RESET;

  /* Configure input channel 1 */
  tmpccmr1 &= (uint32_t)~(TMR_CCM1_CC1SEL | TMR_CCM1_IC1F  | TMR_CCM1_IC1PSC);
  tmpccmr1 |= (uint32_t)(DDL_TMR_ACTIVEINPUT_TRC >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_HallSensorInitStruct->IC1Filter >> 16U);
  tmpccmr1 |= (uint32_t)(TMR_HallSensorInitStruct->IC1Prescaler >> 16U);

  /* Configure input channel 2 */
  tmpccmr1 &= (uint32_t)~(TMR_CCM1_OC2MOD | TMR_CCM1_OC2FEN  | TMR_CCM1_OC2PEN  | TMR_CCM1_OC2CEN);
  tmpccmr1 |= (uint32_t)(DDL_TMR_OCMODE_PWM2 << 8U);

  /* Set Channel 1 polarity and enable Channel 1 and Channel2 */
  tmpccer &= (uint32_t)~(TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL | TMR_CCEN_CC2POL | TMR_CCEN_CC2NPOL);
  tmpccer |= (uint32_t)(TMR_HallSensorInitStruct->IC1Polarity);
  tmpccer |= (uint32_t)(TMR_CCEN_CC1EN | TMR_CCEN_CC2EN);

  /* Write to TMRx CTRL2 */
  DDL_TMR_WriteReg(TMRx, CTRL2, tmpcr2);

  /* Write to TMRx SMCTRL */
  DDL_TMR_WriteReg(TMRx, SMCTRL, tmpsmcr);

  /* Write to TMRx CCM1 */
  DDL_TMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Write to TMRx CCEN */
  DDL_TMR_WriteReg(TMRx, CCEN, tmpccer);

  /* Write to TMRx CC2 */
  DDL_TMR_OC_SetCompareCH2(TMRx, TMR_HallSensorInitStruct->CommutationDelay);

  return SUCCESS;
}

/**
  * @brief  Set the fields of the Break and Dead Time configuration data structure
  *         to their default values.
  * @param  TMR_BDTInitStruct pointer to a @ref DDL_TMR_BDT_InitTypeDef structure (Break and Dead Time configuration
  *         data structure)
  * @retval None
  */
void DDL_TMR_BDT_StructInit(DDL_TMR_BDT_InitTypeDef *TMR_BDTInitStruct)
{
  /* Set the default configuration */
  TMR_BDTInitStruct->OSSRState       = DDL_TMR_OSSR_DISABLE;
  TMR_BDTInitStruct->OSSIState       = DDL_TMR_OSSI_DISABLE;
  TMR_BDTInitStruct->LockLevel       = DDL_TMR_LOCKLEVEL_OFF;
  TMR_BDTInitStruct->DeadTime        = (uint8_t)0x00;
  TMR_BDTInitStruct->BreakState      = DDL_TMR_BREAK_DISABLE;
  TMR_BDTInitStruct->BreakPolarity   = DDL_TMR_BREAK_POLARITY_LOW;
  TMR_BDTInitStruct->AutomaticOutput = DDL_TMR_AUTOMATICOUTPUT_DISABLE;
}

/**
  * @brief  Configure the Break and Dead Time feature of the timer instance.
  * @note As the bits AOE, BKP, BKE, OSSR, OSSI and DTG[7:0] can be write-locked
  *  depending on the LOCK configuration, it can be necessary to configure all of
  *  them during the first write access to the TMRx_BDTR register.
  * @note Macro IS_TMR_BREAK_INSTANCE(TMRx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @param  TMRx Timer Instance
  * @param  TMR_BDTInitStruct pointer to a @ref DDL_TMR_BDT_InitTypeDef structure (Break and Dead Time configuration
  *         data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Break and Dead Time is initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_TMR_BDT_Init(TMR_TypeDef *TMRx, DDL_TMR_BDT_InitTypeDef *TMR_BDTInitStruct)
{
  uint32_t tmpbdtr = 0;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_BREAK_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_OSSR_STATE(TMR_BDTInitStruct->OSSRState));
  ASSERT_PARAM(IS_DDL_TMR_OSSI_STATE(TMR_BDTInitStruct->OSSIState));
  ASSERT_PARAM(IS_DDL_TMR_LOCK_LEVEL(TMR_BDTInitStruct->LockLevel));
  ASSERT_PARAM(IS_DDL_TMR_BREAK_STATE(TMR_BDTInitStruct->BreakState));
  ASSERT_PARAM(IS_DDL_TMR_BREAK_POLARITY(TMR_BDTInitStruct->BreakPolarity));
  ASSERT_PARAM(IS_DDL_TMR_AUTOMATIC_OUTPUT_STATE(TMR_BDTInitStruct->AutomaticOutput));

  /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
  the OSSI State, the dead time value and the Automatic Output Enable Bit */

  /* Set the BDT bits */
  MODIFY_REG(tmpbdtr, TMR_BDT_DTS, TMR_BDTInitStruct->DeadTime);
  MODIFY_REG(tmpbdtr, TMR_BDT_LOCKCFG, TMR_BDTInitStruct->LockLevel);
  MODIFY_REG(tmpbdtr, TMR_BDT_IMOS, TMR_BDTInitStruct->OSSIState);
  MODIFY_REG(tmpbdtr, TMR_BDT_RMOS, TMR_BDTInitStruct->OSSRState);
  MODIFY_REG(tmpbdtr, TMR_BDT_BRKEN, TMR_BDTInitStruct->BreakState);
  MODIFY_REG(tmpbdtr, TMR_BDT_BRKPOL, TMR_BDTInitStruct->BreakPolarity);
  MODIFY_REG(tmpbdtr, TMR_BDT_AOEN, TMR_BDTInitStruct->AutomaticOutput);
  MODIFY_REG(tmpbdtr, TMR_BDT_MOEN, TMR_BDTInitStruct->AutomaticOutput);

  /* Set TMRx_BDT */
  DDL_TMR_WriteReg(TMRx, BDT, tmpbdtr);

  return SUCCESS;
}
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup TMR_DDL_Private_Functions TMR Private Functions
  *  @brief   Private functions
  * @{
  */
/**
  * @brief  Configure the TMRx output channel 1.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC1Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC1_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCNState));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));

  /* Disable the Channel 1: Reset the CC1E Bit */
  CLEAR_BIT(TMRx->CCEN, TMR_CCEN_CC1EN);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_TMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_TMR_ReadReg(TMRx, CTRL2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_TMR_ReadReg(TMRx, CCM1);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr1, TMR_CCM1_CC1SEL);

  /* Set the Output Compare Mode */
  MODIFY_REG(tmpccmr1, TMR_CCM1_OC1MOD, TMR_OCInitStruct->OCMode);

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, TMR_CCEN_CC1POL, TMR_OCInitStruct->OCPolarity);

  /* Set the Output State */
  MODIFY_REG(tmpccer, TMR_CCEN_CC1EN, TMR_OCInitStruct->OCState);

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the complementary output Polarity */
    MODIFY_REG(tmpccer, TMR_CCEN_CC1NPOL, TMR_OCInitStruct->OCNPolarity << 2U);

    /* Set the complementary output State */
    MODIFY_REG(tmpccer, TMR_CCEN_CC1NEN, TMR_OCInitStruct->OCNState << 2U);

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, TMR_CTRL2_OC1OIS, TMR_OCInitStruct->OCIdleState);

    /* Set the complementary output Idle state */
    MODIFY_REG(tmpcr2, TMR_CTRL2_OC1NOIS, TMR_OCInitStruct->OCNIdleState << 1U);
  }

  /* Write to TMRx CTRL2 */
  DDL_TMR_WriteReg(TMRx, CTRL2, tmpcr2);

  /* Write to TMRx CCM1 */
  DDL_TMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Set the Capture Compare Register value */
  DDL_TMR_OC_SetCompareCH1(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_TMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx output channel 2.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 2 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC2Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC2_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCNState));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));

  /* Disable the Channel 2: Reset the CC2E Bit */
  CLEAR_BIT(TMRx->CCEN, TMR_CCEN_CC2EN);

  /* Get the TMRx CCEN register value */
  tmpccer =  DDL_TMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_TMR_ReadReg(TMRx, CTRL2);

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = DDL_TMR_ReadReg(TMRx, CCM1);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr1, TMR_CCM1_CC2SEL);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr1, TMR_CCM1_OC2MOD, TMR_OCInitStruct->OCMode << 8U);

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, TMR_CCEN_CC2POL, TMR_OCInitStruct->OCPolarity << 4U);

  /* Set the Output State */
  MODIFY_REG(tmpccer, TMR_CCEN_CC2EN, TMR_OCInitStruct->OCState << 4U);

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the complementary output Polarity */
    MODIFY_REG(tmpccer, TMR_CCEN_CC2NPOL, TMR_OCInitStruct->OCNPolarity << 6U);

    /* Set the complementary output State */
    MODIFY_REG(tmpccer, TMR_CCEN_CC2NEN, TMR_OCInitStruct->OCNState << 6U);

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, TMR_CTRL2_OC2OIS, TMR_OCInitStruct->OCIdleState << 2U);

    /* Set the complementary output Idle state */
    MODIFY_REG(tmpcr2, TMR_CTRL2_OC2NOIS, TMR_OCInitStruct->OCNIdleState << 3U);
  }

  /* Write to TMRx CTRL2 */
  DDL_TMR_WriteReg(TMRx, CTRL2, tmpcr2);

  /* Write to TMRx CCM1 */
  DDL_TMR_WriteReg(TMRx, CCM1, tmpccmr1);

  /* Set the Capture Compare Register value */
  DDL_TMR_OC_SetCompareCH2(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_TMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx output channel 3.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 3 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC3Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC3_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCNState));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));

  /* Disable the Channel 3: Reset the CC3E Bit */
  CLEAR_BIT(TMRx->CCEN, TMR_CCEN_CC3EN);

  /* Get the TMRx CCEN register value */
  tmpccer =  DDL_TMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = DDL_TMR_ReadReg(TMRx, CTRL2);

  /* Get the TMRx CCM2 register value */
  tmpccmr2 = DDL_TMR_ReadReg(TMRx, CCM2);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr2, TMR_CCM2_CC3SEL);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr2, TMR_CCM2_OC3MOD, TMR_OCInitStruct->OCMode);

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, TMR_CCEN_CC3POL, TMR_OCInitStruct->OCPolarity << 8U);

  /* Set the Output State */
  MODIFY_REG(tmpccer, TMR_CCEN_CC3EN, TMR_OCInitStruct->OCState << 8U);

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the complementary output Polarity */
    MODIFY_REG(tmpccer, TMR_CCEN_CC3NPOL, TMR_OCInitStruct->OCNPolarity << 10U);

    /* Set the complementary output State */
    MODIFY_REG(tmpccer, TMR_CCEN_CC3NEN, TMR_OCInitStruct->OCNState << 10U);

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, TMR_CTRL2_OC3OIS, TMR_OCInitStruct->OCIdleState << 4U);

    /* Set the complementary output Idle state */
    MODIFY_REG(tmpcr2, TMR_CTRL2_OC3NOIS, TMR_OCInitStruct->OCNIdleState << 5U);
  }

  /* Write to TMRx CTRL2 */
  DDL_TMR_WriteReg(TMRx, CTRL2, tmpcr2);

  /* Write to TMRx CCM2 */
  DDL_TMR_WriteReg(TMRx, CCM2, tmpccmr2);

  /* Set the Capture Compare Register value */
  DDL_TMR_OC_SetCompareCH3(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_TMR_WriteReg(TMRx, CCEN, tmpccer);

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx output channel 4.
  * @param  TMRx Timer Instance
  * @param  TMR_OCInitStruct pointer to the the TMRx output channel 4 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus OC4Config(TMR_TypeDef *TMRx, DDL_TMR_OC_InitTypeDef *TMR_OCInitStruct)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC4_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_OCMODE(TMR_OCInitStruct->OCMode));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCState));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCPolarity));
  ASSERT_PARAM(IS_DDL_TMR_OCPOLARITY(TMR_OCInitStruct->OCNPolarity));
  ASSERT_PARAM(IS_DDL_TMR_OCSTATE(TMR_OCInitStruct->OCNState));

  /* Disable the Channel 4: Reset the CC4E Bit */
  CLEAR_BIT(TMRx->CCEN, TMR_CCEN_CC4EN);

  /* Get the TMRx CCEN register value */
  tmpccer = DDL_TMR_ReadReg(TMRx, CCEN);

  /* Get the TMRx CTRL2 register value */
  tmpcr2 =  DDL_TMR_ReadReg(TMRx, CTRL2);

  /* Get the TMRx CCM2 register value */
  tmpccmr2 = DDL_TMR_ReadReg(TMRx, CCM2);

  /* Reset Capture/Compare selection Bits */
  CLEAR_BIT(tmpccmr2, TMR_CCM2_CC4SEL);

  /* Select the Output Compare Mode */
  MODIFY_REG(tmpccmr2, TMR_CCM2_OC4MOD, TMR_OCInitStruct->OCMode << 8U);

  /* Set the Output Compare Polarity */
  MODIFY_REG(tmpccer, TMR_CCEN_CC4POL, TMR_OCInitStruct->OCPolarity << 12U);

  /* Set the Output State */
  MODIFY_REG(tmpccer, TMR_CCEN_CC4EN, TMR_OCInitStruct->OCState << 12U);

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCNIdleState));
    ASSERT_PARAM(IS_DDL_TMR_OCIDLESTATE(TMR_OCInitStruct->OCIdleState));

    /* Set the Output Idle state */
    MODIFY_REG(tmpcr2, TMR_CTRL2_OC4OIS, TMR_OCInitStruct->OCIdleState << 6U);
  }

  /* Write to TMRx CTRL2 */
  DDL_TMR_WriteReg(TMRx, CTRL2, tmpcr2);

  /* Write to TMRx CCM2 */
  DDL_TMR_WriteReg(TMRx, CCM2, tmpccmr2);

  /* Set the Capture Compare Register value */
  DDL_TMR_OC_SetCompareCH4(TMRx, TMR_OCInitStruct->CompareValue);

  /* Write to TMRx CCEN */
  DDL_TMR_WriteReg(TMRx, CCEN , tmpccer);

  return SUCCESS;
}


/**
  * @brief  Configure the TMRx input channel 1.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC1Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC1_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_TMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_TMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_TMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 1: Reset the CC1E Bit */
  TMRx->CCEN &= (uint32_t)~TMR_CCEN_CC1EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM1,
             (TMR_CCM1_CC1SEL | TMR_CCM1_IC1F | TMR_CCM1_IC1PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 16U);

  /* Select the Polarity and set the CC1E Bit */
  MODIFY_REG(TMRx->CCEN,
             (TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL),
             (TMR_ICInitStruct->ICPolarity | TMR_CCEN_CC1EN));

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx input channel 2.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 2 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC2Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC2_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_TMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_TMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_TMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 2: Reset the CC2E Bit */
  TMRx->CCEN &= (uint32_t)~TMR_CCEN_CC2EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM1,
             (TMR_CCM1_CC2SEL | TMR_CCM1_IC2F | TMR_CCM1_IC2PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 8U);

  /* Select the Polarity and set the CC2E Bit */
  MODIFY_REG(TMRx->CCEN,
             (TMR_CCEN_CC2POL | TMR_CCEN_CC2NPOL),
             ((TMR_ICInitStruct->ICPolarity << 4U) | TMR_CCEN_CC2EN));

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx input channel 3.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 3 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC3Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC3_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_TMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_TMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_TMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 3: Reset the CC3E Bit */
  TMRx->CCEN &= (uint32_t)~TMR_CCEN_CC3EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM2,
             (TMR_CCM2_CC3SEL | TMR_CCM2_IC3F | TMR_CCM2_IC3PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 16U);

  /* Select the Polarity and set the CC3E Bit */
  MODIFY_REG(TMRx->CCEN,
             (TMR_CCEN_CC3POL | TMR_CCEN_CC3NPOL),
             ((TMR_ICInitStruct->ICPolarity << 8U) | TMR_CCEN_CC3EN));

  return SUCCESS;
}

/**
  * @brief  Configure the TMRx input channel 4.
  * @param  TMRx Timer Instance
  * @param  TMR_ICInitStruct pointer to the the TMRx input channel 4 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TMRx registers are de-initialized
  *          - ERROR: not applicable
  */
static ErrorStatus IC4Config(TMR_TypeDef *TMRx, DDL_TMR_IC_InitTypeDef *TMR_ICInitStruct)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC4_INSTANCE(TMRx));
  ASSERT_PARAM(IS_DDL_TMR_IC_POLARITY(TMR_ICInitStruct->ICPolarity));
  ASSERT_PARAM(IS_DDL_TMR_ACTIVEINPUT(TMR_ICInitStruct->ICActiveInput));
  ASSERT_PARAM(IS_DDL_TMR_ICPSC(TMR_ICInitStruct->ICPrescaler));
  ASSERT_PARAM(IS_DDL_TMR_IC_FILTER(TMR_ICInitStruct->ICFilter));

  /* Disable the Channel 4: Reset the CC4E Bit */
  TMRx->CCEN &= (uint32_t)~TMR_CCEN_CC4EN;

  /* Select the Input and set the filter and the prescaler value */
  MODIFY_REG(TMRx->CCM2,
             (TMR_CCM2_CC4SEL | TMR_CCM2_IC4F | TMR_CCM2_IC4PSC),
             (TMR_ICInitStruct->ICActiveInput | TMR_ICInitStruct->ICFilter | TMR_ICInitStruct->ICPrescaler) >> 8U);

  /* Select the Polarity and set the CC4E Bit */
  MODIFY_REG(TMRx->CCEN,
             (TMR_CCEN_CC4POL | TMR_CCEN_CC4NPOL),
             ((TMR_ICInitStruct->ICPolarity << 12U) | TMR_CCEN_CC4EN));

  return SUCCESS;
}


/**
  * @}
  */

/**
  * @}
  */

#endif /* TMR1 || TMR2 || TMR3 || TMR4 || TMR5 || TMR6 || TMR7 || TMR8 || TMR9 || TMR10 || TMR11 || TMR12 || TMR13 || TMR14 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

