/**
  *
  * @file    apm32f4xx_ddl_dac.c
  * @brief   DAC DDL module driver
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
#include "apm32f4xx_ddl_dac.h"
#include "apm32f4xx_ddl_bus.h"

#if (USE_FULL_ASSERT == 1U)
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_)                         ((void)(_PARAM_))
#endif /* USE_FULL_ASSERT */

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(DAC)

/** @addtogroup DAC_DDL DAC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/** @addtogroup DAC_DDL_Private_Macros
  * @{
  */
#if defined(DAC_CHANNEL2_SUPPORT)
#define IS_DDL_DAC_CHANNEL(__DACX__, __DAC_CHANNEL__)                           \
  (                                                                            \
      ((__DAC_CHANNEL__) == DDL_DAC_CHANNEL_1)                                  \
   || ((__DAC_CHANNEL__) == DDL_DAC_CHANNEL_2)                                  \
  )
#else
#define IS_DDL_DAC_CHANNEL(__DACX__, __DAC_CHANNEL__)                           \
  (                                                                            \
   ((__DAC_CHANNEL__) == DDL_DAC_CHANNEL_1)                                     \
  )
#endif /* DAC_CHANNEL2_SUPPORT */

#define IS_DDL_DAC_TRIGGER_SOURCE(__TRIGGER_SOURCE__)                           \
  (   ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_SOFTWARE)                           \
   || ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_EXT_TMR2_TRGO)                      \
   || ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_EXT_TMR4_TRGO)                      \
   || ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_EXT_TMR5_TRGO)                      \
   || ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_EXT_TMR6_TRGO)                      \
   || ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_EXT_TMR7_TRGO)                      \
   || ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_EXT_TMR8_TRGO)                      \
   || ((__TRIGGER_SOURCE__) == DDL_DAC_TRIG_EXT_EINT_LINE9)                     \
  )

#define IS_DDL_DAC_WAVE_AUTO_GENER_MODE(__WAVE_AUTO_GENERATION_MODE__)              \
  (   ((__WAVE_AUTO_GENERATION_MODE__) == DDL_DAC_WAVE_AUTO_GENERATION_NONE)        \
      || ((__WAVE_AUTO_GENERATION_MODE__) == DDL_DAC_WAVE_AUTO_GENERATION_NOISE)    \
      || ((__WAVE_AUTO_GENERATION_MODE__) == DDL_DAC_WAVE_AUTO_GENERATION_TRIANGLE) \
  )

#define IS_DDL_DAC_WAVE_AUTO_GENER_CONFIG(__WAVE_AUTO_GENERATION_MODE__, __WAVE_AUTO_GENERATION_CONFIG__)  \
  ( (((__WAVE_AUTO_GENERATION_MODE__) == DDL_DAC_WAVE_AUTO_GENERATION_NOISE)                               \
     && (  ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BIT0)                           \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS1_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS2_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS3_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS4_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS5_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS6_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS7_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS8_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS9_0)                     \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS10_0)                    \
           || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_NOISE_LFSR_UNMASK_BITS11_0))                   \
    )                                                                                                     \
    ||(((__WAVE_AUTO_GENERATION_MODE__) == DDL_DAC_WAVE_AUTO_GENERATION_TRIANGLE)                          \
       && (  ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_1)                           \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_3)                        \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_7)                        \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_15)                       \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_31)                       \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_63)                       \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_127)                      \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_255)                      \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_511)                      \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_1023)                     \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_2047)                     \
             || ((__WAVE_AUTO_GENERATION_CONFIG__) == DDL_DAC_TRIANGLE_AMPLITUDE_4095))                    \
      )                                                                                                   \
  )

#define IS_DDL_DAC_OUTPUT_BUFFER(__OUTPUT_BUFFER__)                             \
  (   ((__OUTPUT_BUFFER__) == DDL_DAC_OUTPUT_BUFFER_ENABLE)                     \
      || ((__OUTPUT_BUFFER__) == DDL_DAC_OUTPUT_BUFFER_DISABLE)                 \
  )

/**
  * @}
  */


/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup DAC_DDL_Exported_Functions
  * @{
  */

/** @addtogroup DAC_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize registers of the selected DAC instance
  *         to their default reset values.
  * @param  DACx DAC instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DAC registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_DAC_DeInit(DAC_TypeDef *DACx)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_ALL_INSTANCE(DACx));

  /* Force reset of DAC clock */
  DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_DAC1);

  /* Release reset of DAC clock */
  DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_DAC1);

  return SUCCESS;
}

/**
  * @brief  Initialize some features of DAC channel.
  * @note   @ref DDL_DAC_Init() aims to ease basic configuration of a DAC channel.
  *         Leaving it ready to be enabled and output:
  *         a level by calling one of
  *           @ref DDL_DAC_ConvertData12RightAligned
  *           @ref DDL_DAC_ConvertData12LeftAligned
  *           @ref DDL_DAC_ConvertData8RightAligned
  *         or one of the supported autogenerated wave.
  * @note   This function allows configuration of:
  *          - Output mode
  *          - Trigger
  *          - Wave generation
  * @note   The setting of these parameters by function @ref DDL_DAC_Init()
  *         is conditioned to DAC state:
  *         DAC channel must be disabled.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *         
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  DAC_InitStruct Pointer to a @ref DDL_DAC_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DAC registers are initialized
  *          - ERROR: DAC registers are not initialized
  */
ErrorStatus DDL_DAC_Init(DAC_TypeDef *DACx, uint32_t DAC_Channel, DDL_DAC_InitTypeDef *DAC_InitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_ALL_INSTANCE(DACx));
  ASSERT_PARAM(IS_DDL_DAC_CHANNEL(DACx, DAC_Channel));
  ASSERT_PARAM(IS_DDL_DAC_TRIGGER_SOURCE(DAC_InitStruct->TriggerSource));
  ASSERT_PARAM(IS_DDL_DAC_OUTPUT_BUFFER(DAC_InitStruct->OutputBuffer));
  ASSERT_PARAM(IS_DDL_DAC_WAVE_AUTO_GENER_MODE(DAC_InitStruct->WaveAutoGeneration));
  if (DAC_InitStruct->WaveAutoGeneration != DDL_DAC_WAVE_AUTO_GENERATION_NONE)
  {
    ASSERT_PARAM(IS_DDL_DAC_WAVE_AUTO_GENER_CONFIG(DAC_InitStruct->WaveAutoGeneration,
                                                  DAC_InitStruct->WaveAutoGenerationConfig));
  }

  /* Note: Hardware constraint (refer to description of this function)        */
  /*       DAC instance must be disabled.                                     */
  if (DDL_DAC_IsEnabled(DACx, DAC_Channel) == 0UL)
  {
    /* Configuration of DAC channel:                                          */
    /*  - TriggerSource                                                       */
    /*  - WaveAutoGeneration                                                  */
    /*  - OutputBuffer                                                        */
    /*  - OutputMode                                                          */
    if (DAC_InitStruct->WaveAutoGeneration != DDL_DAC_WAVE_AUTO_GENERATION_NONE)
    {
      MODIFY_REG(DACx->CTRL,
                 (DAC_CTRL_TRGSELCH1
                  | DAC_CTRL_WAVENCH1
                  | DAC_CTRL_MAMPSELCH1
                  | DAC_CTRL_BUFFDCH1
                 ) << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                 ,
                 (DAC_InitStruct->TriggerSource
                  | DAC_InitStruct->WaveAutoGeneration
                  | DAC_InitStruct->WaveAutoGenerationConfig
                  | DAC_InitStruct->OutputBuffer
                 ) << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                );
    }
    else
    {
      MODIFY_REG(DACx->CTRL,
                 (DAC_CTRL_TRGSELCH1
                  | DAC_CTRL_WAVENCH1
                  | DAC_CTRL_BUFFDCH1
                 ) << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                 ,
                 (DAC_InitStruct->TriggerSource
                  | DDL_DAC_WAVE_AUTO_GENERATION_NONE
                  | DAC_InitStruct->OutputBuffer
                 ) << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                );
    }
  }
  else
  {
    /* Initialization error: DAC instance is not disabled.                    */
    status = ERROR;
  }
  return status;
}

/**
  * @brief Set each @ref DDL_DAC_InitTypeDef field to default value.
  * @param DAC_InitStruct pointer to a @ref DDL_DAC_InitTypeDef structure
  *                       whose fields will be set to default values.
  * @retval None
  */
void DDL_DAC_StructInit(DDL_DAC_InitTypeDef *DAC_InitStruct)
{
  /* Set DAC_InitStruct fields to default values */
  DAC_InitStruct->TriggerSource            = DDL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct->WaveAutoGeneration       = DDL_DAC_WAVE_AUTO_GENERATION_NONE;
  /* Note: Parameter discarded if wave auto generation is disabled,           */
  /*       set anyway to its default value.                                   */
  DAC_InitStruct->WaveAutoGenerationConfig = DDL_DAC_NOISE_LFSR_UNMASK_BIT0;
  DAC_InitStruct->OutputBuffer             = DDL_DAC_OUTPUT_BUFFER_ENABLE;
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

#endif /* DAC */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

