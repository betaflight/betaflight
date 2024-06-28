/**
  *
  * @file    apm32f4xx_ddl_adc.c
  * @brief   ADC DDL module driver
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_adc.h"
#include "apm32f4xx_ddl_bus.h"

#ifdef  USE_FULL_ASSERT
  #include "apm32_assert.h"
#else
  #define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (ADC1) || defined (ADC2) || defined (ADC3)

/** @addtogroup ADC_DDL ADC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/** @addtogroup ADC_DDL_Private_Macros
  * @{
  */

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* common to several ADC instances.                                           */
#define IS_DDL_ADC_COMMON_CLOCK(__CLOCK__)                                      \
  (   ((__CLOCK__) == DDL_ADC_CLOCK_SYNC_PCLK_DIV2)                             \
   || ((__CLOCK__) == DDL_ADC_CLOCK_SYNC_PCLK_DIV4)                             \
   || ((__CLOCK__) == DDL_ADC_CLOCK_SYNC_PCLK_DIV6)                             \
   || ((__CLOCK__) == DDL_ADC_CLOCK_SYNC_PCLK_DIV8)                             \
  )

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC instance.                                                              */
#define IS_DDL_ADC_RESOLUTION(__RESOLUTION__)                                   \
  (   ((__RESOLUTION__) == DDL_ADC_RESOLUTION_12B)                              \
   || ((__RESOLUTION__) == DDL_ADC_RESOLUTION_10B)                              \
   || ((__RESOLUTION__) == DDL_ADC_RESOLUTION_8B)                               \
   || ((__RESOLUTION__) == DDL_ADC_RESOLUTION_6B)                               \
  )

#define IS_DDL_ADC_DATA_ALIGN(__DATA_ALIGN__)                                   \
  (   ((__DATA_ALIGN__) == DDL_ADC_DATA_ALIGN_RIGHT)                            \
   || ((__DATA_ALIGN__) == DDL_ADC_DATA_ALIGN_LEFT)                             \
  )

#define IS_DDL_ADC_SCAN_SELECTION(__SCAN_SELECTION__)                           \
  (   ((__SCAN_SELECTION__) == DDL_ADC_SEQ_SCAN_DISABLE)                        \
   || ((__SCAN_SELECTION__) == DDL_ADC_SEQ_SCAN_ENABLE)                         \
  )

#define IS_DDL_ADC_SEQ_SCAN_MODE(__SEQ_SCAN_MODE__)                             \
  (   ((__SCAN_MODE__) == DDL_ADC_SEQ_SCAN_DISABLE)                             \
   || ((__SCAN_MODE__) == DDL_ADC_SEQ_SCAN_ENABLE)                              \
  )

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC group regular                                                          */
#define IS_DDL_ADC_REG_TRIG_SOURCE(__REG_TRIG_SOURCE__)                         \
  (   ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_SOFTWARE)                      \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR1_CH1)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR1_CH2)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR1_CH3)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR2_CH2)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR2_CH3)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR2_CH4)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR2_TRGO)                 \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR3_CH1)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR3_TRGO)                 \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR4_CH4)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR5_CH1)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR5_CH2)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR5_CH3)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR8_CH1)                  \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_TMR8_TRGO)                 \
   || ((__REG_TRIG_SOURCE__) == DDL_ADC_REG_TRIG_EXT_EINT_LINE11)               \
  )
#define IS_DDL_ADC_REG_CONTINUOUS_MODE(__REG_CONTINUOUS_MODE__)                 \
  (   ((__REG_CONTINUOUS_MODE__) == DDL_ADC_REG_CONV_SINGLE)                    \
   || ((__REG_CONTINUOUS_MODE__) == DDL_ADC_REG_CONV_CONTINUOUS)                \
  )

#define IS_DDL_ADC_REG_DMA_TRANSFER(__REG_DMA_TRANSFER__)                       \
  (   ((__REG_DMA_TRANSFER__) == DDL_ADC_REG_DMA_TRANSFER_NONE)                 \
   || ((__REG_DMA_TRANSFER__) == DDL_ADC_REG_DMA_TRANSFER_LIMITED)              \
   || ((__REG_DMA_TRANSFER__) == DDL_ADC_REG_DMA_TRANSFER_UNLIMITED)            \
  )

#define IS_DDL_ADC_REG_FLAG_EOC_SELECTION(__REG_FLAG_EOC_SELECTION__)           \
  (   ((__REG_FLAG_EOC_SELECTION__) == DDL_ADC_REG_FLAG_EOC_SEQUENCE_CONV)      \
   || ((__REG_FLAG_EOC_SELECTION__) == DDL_ADC_REG_FLAG_EOC_UNITARY_CONV)       \
  )

#define IS_DDL_ADC_REG_SEQ_SCAN_LENGTH(__REG_SEQ_SCAN_LENGTH__)                 \
  (   ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_DISABLE)               \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS)         \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS)        \
   || ((__REG_SEQ_SCAN_LENGTH__) == DDL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS)        \
  )

#define IS_DDL_ADC_REG_SEQ_SCAN_DISCONT_MODE(__REG_SEQ_DISCONT_MODE__)          \
  (   ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_DISABLE)           \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_1RANK)             \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_2RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_3RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_4RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_5RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_6RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_7RANKS)            \
   || ((__REG_SEQ_DISCONT_MODE__) == DDL_ADC_REG_SEQ_DISCONT_8RANKS)            \
  )

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC group injected                                                         */
#define IS_DDL_ADC_INJ_TRIG_SOURCE(__INJ_TRIG_SOURCE__)                         \
  (   ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_SOFTWARE)                      \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR1_CH4)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR1_TRGO)                 \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR2_CH1)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR2_TRGO)                 \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR3_CH2)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR3_CH4)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR4_CH1)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR4_CH2)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR4_CH3)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR4_TRGO)                 \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR5_CH4)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR5_TRGO)                 \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR8_CH2)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR8_CH3)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_TMR8_CH4)                  \
   || ((__INJ_TRIG_SOURCE__) == DDL_ADC_INJ_TRIG_EXT_EXTI_LINE15)               \
  )

#define IS_DDL_ADC_INJ_TRIG_EXT_EDGE(__INJ_TRIG_EXT_EDGE__)                     \
  (   ((__INJ_TRIG_EXT_EDGE__) == DDL_ADC_INJ_TRIG_EXT_RISING)                  \
   || ((__INJ_TRIG_EXT_EDGE__) == DDL_ADC_INJ_TRIG_EXT_FALLING)                 \
   || ((__INJ_TRIG_EXT_EDGE__) == DDL_ADC_INJ_TRIG_EXT_RISINGFALLING)           \
  )

#define IS_DDL_ADC_INJ_TRIG_AUTO(__INJ_TRIG_AUTO__)                             \
  (   ((__INJ_TRIG_AUTO__) == DDL_ADC_INJ_TRIG_INDEPENDENT)                     \
   || ((__INJ_TRIG_AUTO__) == DDL_ADC_INJ_TRIG_FROM_GRP_REGULAR)                \
  )

#define IS_DDL_ADC_INJ_SEQ_SCAN_LENGTH(__INJ_SEQ_SCAN_LENGTH__)                 \
  (   ((__INJ_SEQ_SCAN_LENGTH__) == DDL_ADC_INJ_SEQ_SCAN_DISABLE)               \
   || ((__INJ_SEQ_SCAN_LENGTH__) == DDL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS)         \
   || ((__INJ_SEQ_SCAN_LENGTH__) == DDL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS)         \
   || ((__INJ_SEQ_SCAN_LENGTH__) == DDL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS)         \
  )

#define IS_DDL_ADC_INJ_SEQ_SCAN_DISCONT_MODE(__INJ_SEQ_DISCONT_MODE__)          \
  (   ((__INJ_SEQ_DISCONT_MODE__) == DDL_ADC_INJ_SEQ_DISCONT_DISABLE)           \
   || ((__INJ_SEQ_DISCONT_MODE__) == DDL_ADC_INJ_SEQ_DISCONT_1RANK)             \
  )

#if defined(ADC_MULTIMODE_SUPPORT)
/* Check of parameters for configuration of ADC hierarchical scope:           */
/* multimode.                                                                 */
#if defined(ADC3)
#define IS_DDL_ADC_MULTI_MODE(__MULTI_MODE__)                                   \
  (   ((__MULTI_MODE__) == DDL_ADC_MULTI_INDEPENDENT)                           \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_SIMULT)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_INTERL)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_INJ_SIMULT)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_INJ_ALTERN)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_SIM_INJ_SIM)                  \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_SIM_INJ_ALT)                  \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_INT_INJ_SIM)                  \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_TRIPLE_REG_SIM_INJ_SIM)                \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_TRIPLE_REG_SIM_INJ_ALT)                \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_TRIPLE_INJ_SIMULT)                     \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_TRIPLE_REG_SIMULT)                     \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_TRIPLE_REG_INTERL)                     \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_TRIPLE_INJ_ALTERN)                     \
  )
#else
#define IS_DDL_ADC_MULTI_MODE(__MULTI_MODE__)                                   \
  (   ((__MULTI_MODE__) == DDL_ADC_MULTI_INDEPENDENT)                           \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_SIMULT)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_INTERL)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_INJ_SIMULT)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_INJ_ALTERN)                       \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_SIM_INJ_SIM)                  \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_SIM_INJ_ALT)                  \
   || ((__MULTI_MODE__) == DDL_ADC_MULTI_DUAL_REG_INT_INJ_SIM)                  \
  )
#endif

#define IS_DDL_ADC_MULTI_DMA_TRANSFER(__MULTI_DMA_TRANSFER__)                   \
  (   ((__MULTI_DMA_TRANSFER__) == DDL_ADC_MULTI_REG_DMA_EACH_ADC)              \
   || ((__MULTI_DMA_TRANSFER__) == DDL_ADC_MULTI_REG_DMA_LIMIT_1)               \
   || ((__MULTI_DMA_TRANSFER__) == DDL_ADC_MULTI_REG_DMA_LIMIT_2)               \
   || ((__MULTI_DMA_TRANSFER__) == DDL_ADC_MULTI_REG_DMA_LIMIT_3)               \
   || ((__MULTI_DMA_TRANSFER__) == DDL_ADC_MULTI_REG_DMA_UNLMT_1)               \
   || ((__MULTI_DMA_TRANSFER__) == DDL_ADC_MULTI_REG_DMA_UNLMT_2)               \
   || ((__MULTI_DMA_TRANSFER__) == DDL_ADC_MULTI_REG_DMA_UNLMT_3)               \
  )

#define IS_DDL_ADC_MULTI_TWOSMP_DELAY(__MULTI_TWOSMP_DELAY__)                   \
  (   ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_5CYCLES)          \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_6CYCLES)          \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_7CYCLES)          \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_8CYCLES)          \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_9CYCLES)          \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_10CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_11CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_12CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_13CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_14CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_15CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_16CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_17CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_18CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_19CYCLES)         \
   || ((__MULTI_TWOSMP_DELAY__) == DDL_ADC_MULTI_TWOSMP_DELAY_20CYCLES)         \
  )

#define IS_DDL_ADC_MULTI_MASTER_SLAVE(__MULTI_MASTER_SLAVE__)                   \
  (   ((__MULTI_MASTER_SLAVE__) == DDL_ADC_MULTI_MASTER)                        \
   || ((__MULTI_MASTER_SLAVE__) == DDL_ADC_MULTI_SLAVE)                         \
   || ((__MULTI_MASTER_SLAVE__) == DDL_ADC_MULTI_MASTER_SLAVE)                  \
  )

#endif /* ADC_MULTIMODE_SUPPORT */
/**
  * @}
  */


/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_DDL_Exported_Functions
  * @{
  */

/** @addtogroup ADC_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize registers of all ADC instances belonging to
  *         the same ADC common instance to their default reset values.
  * @param  ADCxy_COMMON ADC common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __DDL_ADC_COMMON_INSTANCE() )
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC common registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_ADC_CommonDeInit(ADC_Common_TypeDef *ADCxy_COMMON)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_COMMON_INSTANCE(ADCxy_COMMON));
  

  /* Force reset of ADC clock (core clock) */
  DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_ADC);
  
  /* Release reset of ADC clock (core clock) */
  DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_ADC);
  
  return SUCCESS;
}

/**
  * @brief  Initialize some features of ADC common parameters
  *         (all ADC instances belonging to the same ADC common instance)
  *         and multimode (for devices with several ADC instances available).
  * @note   The setting of ADC common parameters is conditioned to
  *         ADC instances state:
  *         All ADC instances belonging to the same ADC common instance
  *         must be disabled.
  * @param  ADCxy_COMMON ADC common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __DDL_ADC_COMMON_INSTANCE() )
  * @param  ADC_CommonInitStruct Pointer to a @ref DDL_ADC_CommonInitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC common registers are initialized
  *          - ERROR: ADC common registers are not initialized
  */
ErrorStatus DDL_ADC_CommonInit(ADC_Common_TypeDef *ADCxy_COMMON, DDL_ADC_CommonInitTypeDef *ADC_CommonInitStruct)
{
  ErrorStatus status = SUCCESS;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_COMMON_INSTANCE(ADCxy_COMMON));
  ASSERT_PARAM(IS_DDL_ADC_COMMON_CLOCK(ADC_CommonInitStruct->CommonClock));
  
#if defined(ADC_MULTIMODE_SUPPORT)
  ASSERT_PARAM(IS_DDL_ADC_MULTI_MODE(ADC_CommonInitStruct->Multimode));
  if(ADC_CommonInitStruct->Multimode != DDL_ADC_MULTI_INDEPENDENT)
  {
    ASSERT_PARAM(IS_DDL_ADC_MULTI_DMA_TRANSFER(ADC_CommonInitStruct->MultiDMATransfer));
    ASSERT_PARAM(IS_DDL_ADC_MULTI_TWOSMP_DELAY(ADC_CommonInitStruct->MultiTwoSamplingDelay));
  }
#endif /* ADC_MULTIMODE_SUPPORT */

  /* Note: Hardware constraint (refer to description of functions             */
  /*       "DDL_ADC_SetCommonXXX()" and "DDL_ADC_SetMultiXXX()"):               */
  /*       On this APM32 series, setting of these features is conditioned to  */
  /*       ADC state:                                                         */
  /*       All ADC instances of the ADC common group must be disabled.        */
  if(__DDL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(ADCxy_COMMON) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - common to several ADC                                               */
    /*    (all ADC instances belonging to the same ADC common instance)       */
    /*    - Set ADC clock (conversion clock)                                  */
    /*  - multimode (if several ADC instances available on the                */
    /*    selected device)                                                    */
    /*    - Set ADC multimode configuration                                   */
    /*    - Set ADC multimode DMA transfer                                    */
    /*    - Set ADC multimode: delay between 2 sampling phases                */
#if defined(ADC_MULTIMODE_SUPPORT)
    if(ADC_CommonInitStruct->Multimode != DDL_ADC_MULTI_INDEPENDENT)
    {
      MODIFY_REG(ADCxy_COMMON->CCTRL,
                   ADC_CCTRL_ADCPRE
                 | ADC_CCTRL_ADCMSEL
                 | ADC_CCTRL_DMAMODE
                 | ADC_CCTRL_DMAMODEDISSEL
                 | ADC_CCTRL_SMPDEL2
                ,
                   ADC_CommonInitStruct->CommonClock
                 | ADC_CommonInitStruct->Multimode
                 | ADC_CommonInitStruct->MultiDMATransfer
                 | ADC_CommonInitStruct->MultiTwoSamplingDelay
                );
    }
    else
    {
      MODIFY_REG(ADCxy_COMMON->CCTRL,
                   ADC_CCTRL_ADCPRE
                 | ADC_CCTRL_ADCMSEL
                 | ADC_CCTRL_DMAMODE
                 | ADC_CCTRL_DMAMODEDISSEL
                 | ADC_CCTRL_SMPDEL2
                ,
                   ADC_CommonInitStruct->CommonClock
                 | DDL_ADC_MULTI_INDEPENDENT
                );
    }
#else
    DDL_ADC_SetCommonClock(ADCxy_COMMON, ADC_CommonInitStruct->CommonClock);
#endif
  }
  else
  {
    /* Initialization error: One or several ADC instances belonging to        */
    /* the same ADC common instance are not disabled.                         */
    status = ERROR;
  }
  
  return status;
}

/**
  * @brief  Set each @ref DDL_ADC_CommonInitTypeDef field to default value.
  * @param  ADC_CommonInitStruct Pointer to a @ref DDL_ADC_CommonInitTypeDef structure
  *                              whose fields will be set to default values.
  * @retval None
  */
void DDL_ADC_CommonStructInit(DDL_ADC_CommonInitTypeDef *ADC_CommonInitStruct)
{
  /* Set ADC_CommonInitStruct fields to default values */
  /* Set fields of ADC common */
  /* (all ADC instances belonging to the same ADC common instance) */
  ADC_CommonInitStruct->CommonClock = DDL_ADC_CLOCK_SYNC_PCLK_DIV2;
  
#if defined(ADC_MULTIMODE_SUPPORT)
  /* Set fields of ADC multimode */
  ADC_CommonInitStruct->Multimode             = DDL_ADC_MULTI_INDEPENDENT;
    ADC_CommonInitStruct->MultiDMATransfer      = DDL_ADC_MULTI_REG_DMA_EACH_ADC;
  ADC_CommonInitStruct->MultiTwoSamplingDelay = DDL_ADC_MULTI_TWOSMP_DELAY_5CYCLES;
#endif /* ADC_MULTIMODE_SUPPORT */
}

/**
  * @brief  De-initialize registers of the selected ADC instance
  *         to their default reset values.
  * @note   To reset all ADC instances quickly (perform a hard reset),
  *         use function @ref DDL_ADC_CommonDeInit().
  * @param  ADCx ADC instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR: ADC registers are not de-initialized
  */
ErrorStatus DDL_ADC_DeInit(ADC_TypeDef *ADCx)
{
  ErrorStatus status = SUCCESS;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));
  
  /* Disable ADC instance if not already disabled.                            */
  if(DDL_ADC_IsEnabled(ADCx) == 1UL)
  {
    /* Set ADC group regular trigger source to SW start to ensure to not      */
    /* have an external trigger event occurring during the conversion stop    */
    /* ADC disable process.                                                   */
    DDL_ADC_REG_SetTriggerSource(ADCx, DDL_ADC_REG_TRIG_SOFTWARE);
    
    /* Set ADC group injected trigger source to SW start to ensure to not     */
    /* have an external trigger event occurring during the conversion stop    */
    /* ADC disable process.                                                   */
    DDL_ADC_INJ_SetTriggerSource(ADCx, DDL_ADC_INJ_TRIG_SOFTWARE);
    
    /* Disable the ADC instance */
    DDL_ADC_Disable(ADCx);
  }
  
  /* Check whether ADC state is compliant with expected state */
  /* (hardware requirements of bits state to reset registers below) */
  if(READ_BIT(ADCx->CTRL2, ADC_CTRL2_ADCEN) == 0UL)
  {
    /* ========== Reset ADC registers ========== */
    /* Reset register STS */
    CLEAR_BIT(ADCx->STS,
              (  DDL_ADC_FLAG_STRT
               | DDL_ADC_FLAG_JSTRT
               | DDL_ADC_FLAG_EOCS
               | DDL_ADC_FLAG_OVR
               | DDL_ADC_FLAG_JEOS
               | DDL_ADC_FLAG_AWD1 )
             );
    
    /* Reset register CTRL1 */
    CLEAR_BIT(ADCx->CTRL1,
              (  ADC_CTRL1_OVRIEN   | ADC_CTRL1_RESSEL     | ADC_CTRL1_REGAWDEN
               | ADC_CTRL1_INJAWDEN
               | ADC_CTRL1_DISCNUMCFG | ADC_CTRL1_INJDISCEN | ADC_CTRL1_REGDISCEN
               | ADC_CTRL1_INJGACEN   | ADC_CTRL1_AWDSGLEN  | ADC_CTRL1_SCANEN
               | ADC_CTRL1_INJEOCIEN  | ADC_CTRL1_AWDIEN   | ADC_CTRL1_EOCIEN
               | ADC_CTRL1_AWDCHSEL                                     )
             );
    
    /* Reset register CTRL2 */
    CLEAR_BIT(ADCx->CTRL2,
              (  ADC_CTRL2_REGCHSC  | ADC_CTRL2_REGEXTTRGEN  | ADC_CTRL2_REGEXTTRGSEL
               | ADC_CTRL2_INJSWSC | ADC_CTRL2_INJEXTTRGEN | ADC_CTRL2_INJGEXTTRGSEL
               | ADC_CTRL2_DALIGNCFG    | ADC_CTRL2_EOCSEL
               | ADC_CTRL2_DMADISSEL      | ADC_CTRL2_DMAEN
               | ADC_CTRL2_CONTCEN     | ADC_CTRL2_ADCEN                    )
             );
    
    /* Reset register SMPTIM1 */
    CLEAR_BIT(ADCx->SMPTIM1,
              (  ADC_SMPTIM1_SMPCYCCFG18 | ADC_SMPTIM1_SMPCYCCFG17 | ADC_SMPTIM1_SMPCYCCFG16
               | ADC_SMPTIM1_SMPCYCCFG15 | ADC_SMPTIM1_SMPCYCCFG14 | ADC_SMPTIM1_SMPCYCCFG13
               | ADC_SMPTIM1_SMPCYCCFG12 | ADC_SMPTIM1_SMPCYCCFG11 | ADC_SMPTIM1_SMPCYCCFG10)
             );
    
    /* Reset register SMPTIM2 */
    CLEAR_BIT(ADCx->SMPTIM2,
              (  ADC_SMPTIM2_SMPCYCCFG9
               | ADC_SMPTIM2_SMPCYCCFG8 | ADC_SMPTIM2_SMPCYCCFG7 | ADC_SMPTIM2_SMPCYCCFG6
               | ADC_SMPTIM2_SMPCYCCFG5 | ADC_SMPTIM2_SMPCYCCFG4 | ADC_SMPTIM2_SMPCYCCFG3
               | ADC_SMPTIM2_SMPCYCCFG2 | ADC_SMPTIM2_SMPCYCCFG1 | ADC_SMPTIM2_SMPCYCCFG0)
             );
    
    /* Reset register INJDOF1 */
    CLEAR_BIT(ADCx->INJDOF1, ADC_INJDOF1_INJDOF1);
    /* Reset register INJDOF2 */
    CLEAR_BIT(ADCx->INJDOF2, ADC_INJDOF2_INJDOF2);
    /* Reset register INJDOF3 */
    CLEAR_BIT(ADCx->INJDOF3, ADC_INJDOF3_INJDOF3);
    /* Reset register INJDOF4 */
    CLEAR_BIT(ADCx->INJDOF4, ADC_INJDOF4_INJDOF4);
    
    /* Reset register AWDHT */
    SET_BIT(ADCx->AWDHT, ADC_AWDHT_AWDHT);
    /* Reset register AWDLT */
    CLEAR_BIT(ADCx->AWDLT, ADC_AWDLT_AWDLT);
    
    /* Reset register REGSEQ1 */
    CLEAR_BIT(ADCx->REGSEQ1,
              (  ADC_REGSEQ1_REGSEQLEN
               | ADC_REGSEQ1_REGSEQC16
               | ADC_REGSEQ1_REGSEQC15 | ADC_REGSEQ1_REGSEQC14 | ADC_REGSEQ1_REGSEQC13)
             );
             
    /* Reset register REGSEQ2 */
    CLEAR_BIT(ADCx->REGSEQ2,
              (  ADC_REGSEQ2_REGSEQC12 | ADC_REGSEQ2_REGSEQC11 | ADC_REGSEQ2_REGSEQC10
               | ADC_REGSEQ2_REGSEQC9 | ADC_REGSEQ2_REGSEQC8 | ADC_REGSEQ2_REGSEQC7)
             );

    /* Reset register REGSEQ3 */
    CLEAR_BIT(ADCx->REGSEQ3,
              (  ADC_REGSEQ3_REGSEQC6 | ADC_REGSEQ3_REGSEQC5 | ADC_REGSEQ3_REGSEQC4
               | ADC_REGSEQ3_REGSEQC3 | ADC_REGSEQ3_REGSEQC2 | ADC_REGSEQ3_REGSEQC1)
             );

    /* Reset register INJSEQ */
    CLEAR_BIT(ADCx->INJSEQ,
              (  ADC_INJSEQ_INJSEQLEN
               | ADC_INJSEQ_INJSEQC4 | ADC_INJSEQ_INJSEQC3
               | ADC_INJSEQ_INJSEQC2 | ADC_INJSEQ_INJSEQC1  )
             );
    
    /* Reset register REGDATA */
    /* bits in access mode read only, no direct reset applicable */
    
    /* Reset registers INJDATA1, INJDATA2, INJDATA3, INJDATA4 */
    /* bits in access mode read only, no direct reset applicable */
    
    /* Reset register CCTRL */
#if defined(ADC1) && defined(ADC2) && defined(ADC3)
    CLEAR_BIT(ADC->CCTRL, ADC_CCTRL_TSVREFEN | ADC_CCTRL_ADCPRE);
#else
    CLEAR_BIT(ADC1_C->CCTRL, ADC_CCTRL_TSVREFEN | ADC_CCTRL_ADCPRE);
    CLEAR_BIT(ADC2_C->CCTRL, ADC_CCTRL_TSVREFEN | ADC_CCTRL_ADCPRE);
#endif /* ADC2 || ADC3 */
  }
  
  return status;
}

/**
  * @brief  Initialize some features of ADC instance.
  * @note   These parameters have an impact on ADC scope: ADC instance.
  *         Affects both group regular and group injected (availability
  *         of ADC group injected depends on APM32 families).
  *         Refer to corresponding unitary functions into
  *         @ref ADC_DDL_EF_Configuration_ADC_Instance .
  * @note   The setting of these parameters by function @ref DDL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all APM32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @note   After using this function, some other features must be configured
  *         using LL unitary functions.
  *         The minimum configuration remaining to be done is:
  *          - Set ADC group regular or group injected sequencer:
  *            map channel on the selected sequencer rank.
  *            Refer to function @ref DDL_ADC_REG_SetSequencerRanks().
  *          - Set ADC channel sampling time
  *            Refer to function DDL_ADC_SetChannelSamplingTime();
  * @param  ADCx ADC instance
  * @param  ADC_InitStruct Pointer to a @ref DDL_ADC_REG_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus DDL_ADC_Init(ADC_TypeDef *ADCx, DDL_ADC_InitTypeDef *ADC_InitStruct)
{
  ErrorStatus status = SUCCESS;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));
  
  ASSERT_PARAM(IS_DDL_ADC_RESOLUTION(ADC_InitStruct->Resolution));
  ASSERT_PARAM(IS_DDL_ADC_DATA_ALIGN(ADC_InitStruct->DataAlignment));
  ASSERT_PARAM(IS_DDL_ADC_SCAN_SELECTION(ADC_InitStruct->SequencersScanMode));
  
  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if(DDL_ADC_IsEnabled(ADCx) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC instance                                                        */
    /*    - Set ADC data resolution                                           */
    /*    - Set ADC conversion data alignment                                 */
    MODIFY_REG(ADCx->CTRL1,
                 ADC_CTRL1_RESSEL
               | ADC_CTRL1_SCANEN
              ,
                 ADC_InitStruct->Resolution
               | ADC_InitStruct->SequencersScanMode
              );
    
    MODIFY_REG(ADCx->CTRL2,
                 ADC_CTRL2_DALIGNCFG
              ,
                 ADC_InitStruct->DataAlignment
              );

  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_ADC_InitTypeDef field to default value.
  * @param  ADC_InitStruct Pointer to a @ref DDL_ADC_InitTypeDef structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void DDL_ADC_StructInit(DDL_ADC_InitTypeDef *ADC_InitStruct)
{
  /* Set ADC_InitStruct fields to default values */
  /* Set fields of ADC instance */
  ADC_InitStruct->Resolution    = DDL_ADC_RESOLUTION_12B;
  ADC_InitStruct->DataAlignment = DDL_ADC_DATA_ALIGN_RIGHT;
  
  /* Enable scan mode to have a generic behavior with ADC of other            */
  /* APM32 families, without this setting available:                          */
  /* ADC group regular sequencer and ADC group injected sequencer depend      */
  /* only of their own configuration.                                         */
  ADC_InitStruct->SequencersScanMode      = DDL_ADC_SEQ_SCAN_ENABLE;
  
}

/**
  * @brief  Initialize some features of ADC group regular.
  * @note   These parameters have an impact on ADC scope: ADC group regular.
  *         Refer to corresponding unitary functions into
  *         @ref ADC_DDL_EF_Configuration_ADC_Group_Regular
  *         (functions with prefix "REG").
  * @note   The setting of these parameters by function @ref DDL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all APM32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @note   After using this function, other features must be configured
  *         using LL unitary functions.
  *         The minimum configuration remaining to be done is:
  *          - Set ADC group regular or group injected sequencer:
  *            map channel on the selected sequencer rank.
  *            Refer to function @ref DDL_ADC_REG_SetSequencerRanks().
  *          - Set ADC channel sampling time
  *            Refer to function DDL_ADC_SetChannelSamplingTime();
  * @param  ADCx ADC instance
  * @param  ADC_REG_InitStruct Pointer to a @ref DDL_ADC_REG_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus DDL_ADC_REG_Init(ADC_TypeDef *ADCx, DDL_ADC_REG_InitTypeDef *ADC_REG_InitStruct)
{
  ErrorStatus status = SUCCESS;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));
  ASSERT_PARAM(IS_DDL_ADC_REG_TRIG_SOURCE(ADC_REG_InitStruct->TriggerSource));
  ASSERT_PARAM(IS_DDL_ADC_REG_SEQ_SCAN_LENGTH(ADC_REG_InitStruct->SequencerLength));
  if(ADC_REG_InitStruct->SequencerLength != DDL_ADC_REG_SEQ_SCAN_DISABLE)
  {
    ASSERT_PARAM(IS_DDL_ADC_REG_SEQ_SCAN_DISCONT_MODE(ADC_REG_InitStruct->SequencerDiscont));
  }
  ASSERT_PARAM(IS_DDL_ADC_REG_CONTINUOUS_MODE(ADC_REG_InitStruct->ContinuousMode));
  ASSERT_PARAM(IS_DDL_ADC_REG_DMA_TRANSFER(ADC_REG_InitStruct->DMATransfer));
  
  /* ADC group regular continuous mode and discontinuous mode                 */
  /* can not be enabled simultenaeously                                       */
  ASSERT_PARAM((ADC_REG_InitStruct->ContinuousMode == DDL_ADC_REG_CONV_SINGLE)
               || (ADC_REG_InitStruct->SequencerDiscont == DDL_ADC_REG_SEQ_DISCONT_DISABLE));
  
  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if(DDL_ADC_IsEnabled(ADCx) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC group regular                                                   */
    /*    - Set ADC group regular trigger source                              */
    /*    - Set ADC group regular sequencer length                            */
    /*    - Set ADC group regular sequencer discontinuous mode                */
    /*    - Set ADC group regular continuous mode                             */
    /*    - Set ADC group regular conversion data transfer: no transfer or    */
    /*      transfer by DMA, and DMA requests mode                            */
    /* Note: On this APM32 series, ADC trigger edge is set when starting      */
    /*       ADC conversion.                                                  */
    /*       Refer to function @ref DDL_ADC_REG_StartConversionExtTrig().      */
    if(ADC_REG_InitStruct->SequencerLength != DDL_ADC_REG_SEQ_SCAN_DISABLE)
    {
      MODIFY_REG(ADCx->CTRL1,
                   ADC_CTRL1_REGDISCEN
                 | ADC_CTRL1_DISCNUMCFG
                ,
                   ADC_REG_InitStruct->SequencerDiscont
                );
    }
    else
    {
      MODIFY_REG(ADCx->CTRL1,
                   ADC_CTRL1_REGDISCEN
                 | ADC_CTRL1_DISCNUMCFG
                ,
                   DDL_ADC_REG_SEQ_DISCONT_DISABLE
                );
    }
    
    MODIFY_REG(ADCx->CTRL2,
                 ADC_CTRL2_REGEXTTRGSEL
               | ADC_CTRL2_REGEXTTRGEN
               | ADC_CTRL2_CONTCEN
               | ADC_CTRL2_DMAEN
               | ADC_CTRL2_DMADISSEL
              ,
                (ADC_REG_InitStruct->TriggerSource & ADC_CTRL2_REGEXTTRGSEL)
               | ADC_REG_InitStruct->ContinuousMode
               | ADC_REG_InitStruct->DMATransfer
              );

    /* Set ADC group regular sequencer length and scan direction */
    /* Note: Hardware constraint (refer to description of this function):     */
    /* Note: If ADC instance feature scan mode is disabled                    */
    /*       (refer to  ADC instance initialization structure                 */
    /*       parameter @ref SequencersScanMode                                */
    /*       or function @ref DDL_ADC_SetSequencersScanMode() ),               */
    /*       this parameter is discarded.                                     */
    DDL_ADC_REG_SetSequencerLength(ADCx, ADC_REG_InitStruct->SequencerLength);
  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_ADC_REG_InitTypeDef field to default value.
  * @param  ADC_REG_InitStruct Pointer to a @ref DDL_ADC_REG_InitTypeDef structure
  *                            whose fields will be set to default values.
  * @retval None
  */
void DDL_ADC_REG_StructInit(DDL_ADC_REG_InitTypeDef *ADC_REG_InitStruct)
{
  /* Set ADC_REG_InitStruct fields to default values */
  /* Set fields of ADC group regular */
  /* Note: On this APM32 series, ADC trigger edge is set when starting        */
  /*       ADC conversion.                                                    */
  /*       Refer to function @ref DDL_ADC_REG_StartConversionExtTrig().        */
  ADC_REG_InitStruct->TriggerSource    = DDL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct->SequencerLength  = DDL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct->SequencerDiscont = DDL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct->ContinuousMode   = DDL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct->DMATransfer      = DDL_ADC_REG_DMA_TRANSFER_NONE;
}

/**
  * @brief  Initialize some features of ADC group injected.
  * @note   These parameters have an impact on ADC scope: ADC group injected.
  *         Refer to corresponding unitary functions into
  *         @ref ADC_DDL_EF_Configuration_ADC_Group_Regular
  *         (functions with prefix "INJ").
  * @note   The setting of these parameters by function @ref DDL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all APM32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @note   After using this function, other features must be configured
  *         using LL unitary functions.
  *         The minimum configuration remaining to be done is:
  *          - Set ADC group injected sequencer:
  *            map channel on the selected sequencer rank.
  *            Refer to function @ref DDL_ADC_INJ_SetSequencerRanks().
  *          - Set ADC channel sampling time
  *            Refer to function DDL_ADC_SetChannelSamplingTime();
  * @param  ADCx ADC instance
  * @param  ADC_INJ_InitStruct Pointer to a @ref DDL_ADC_INJ_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus DDL_ADC_INJ_Init(ADC_TypeDef *ADCx, DDL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct)
{
  ErrorStatus status = SUCCESS;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(ADCx));
  ASSERT_PARAM(IS_DDL_ADC_INJ_TRIG_SOURCE(ADC_INJ_InitStruct->TriggerSource));
  ASSERT_PARAM(IS_DDL_ADC_INJ_SEQ_SCAN_LENGTH(ADC_INJ_InitStruct->SequencerLength));
  if(ADC_INJ_InitStruct->SequencerLength != DDL_ADC_INJ_SEQ_SCAN_DISABLE)
  {
    ASSERT_PARAM(IS_DDL_ADC_INJ_SEQ_SCAN_DISCONT_MODE(ADC_INJ_InitStruct->SequencerDiscont));
  }
  ASSERT_PARAM(IS_DDL_ADC_INJ_TRIG_AUTO(ADC_INJ_InitStruct->TrigAuto));
  
  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if(DDL_ADC_IsEnabled(ADCx) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC group injected                                                  */
    /*    - Set ADC group injected trigger source                             */
    /*    - Set ADC group injected sequencer length                           */
    /*    - Set ADC group injected sequencer discontinuous mode               */
    /*    - Set ADC group injected conversion trigger: independent or         */
    /*      from ADC group regular                                            */
    /* Note: On this APM32 series, ADC trigger edge is set when starting      */
    /*       ADC conversion.                                                  */
    /*       Refer to function @ref DDL_ADC_INJ_StartConversionExtTrig().      */
    if(ADC_INJ_InitStruct->SequencerLength != DDL_ADC_REG_SEQ_SCAN_DISABLE)
    {
      MODIFY_REG(ADCx->CTRL1,
                   ADC_CTRL1_INJDISCEN
                 | ADC_CTRL1_INJGACEN
                ,
                   ADC_INJ_InitStruct->SequencerDiscont
                 | ADC_INJ_InitStruct->TrigAuto
                );
    }
    else
    {
      MODIFY_REG(ADCx->CTRL1,
                   ADC_CTRL1_INJDISCEN
                 | ADC_CTRL1_INJGACEN
                ,
                   DDL_ADC_REG_SEQ_DISCONT_DISABLE
                 | ADC_INJ_InitStruct->TrigAuto
                );
    }
    
    MODIFY_REG(ADCx->CTRL2,
                 ADC_CTRL2_INJGEXTTRGSEL
               | ADC_CTRL2_INJEXTTRGEN
              ,
                (ADC_INJ_InitStruct->TriggerSource & ADC_CTRL2_INJGEXTTRGSEL)
              );
    
    /* Note: Hardware constraint (refer to description of this function):     */
    /* Note: If ADC instance feature scan mode is disabled                    */
    /*       (refer to  ADC instance initialization structure                 */
    /*       parameter @ref SequencersScanMode                                */
    /*       or function @ref DDL_ADC_SetSequencersScanMode() ),               */
    /*       this parameter is discarded.                                     */
    DDL_ADC_INJ_SetSequencerLength(ADCx, ADC_INJ_InitStruct->SequencerLength);
  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_ADC_INJ_InitTypeDef field to default value.
  * @param  ADC_INJ_InitStruct Pointer to a @ref DDL_ADC_INJ_InitTypeDef structure
  *                            whose fields will be set to default values.
  * @retval None
  */
void DDL_ADC_INJ_StructInit(DDL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct)
{
  /* Set ADC_INJ_InitStruct fields to default values */
  /* Set fields of ADC group injected */
  ADC_INJ_InitStruct->TriggerSource    = DDL_ADC_INJ_TRIG_SOFTWARE;
  ADC_INJ_InitStruct->SequencerLength  = DDL_ADC_INJ_SEQ_SCAN_DISABLE;
  ADC_INJ_InitStruct->SequencerDiscont = DDL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct->TrigAuto         = DDL_ADC_INJ_TRIG_INDEPENDENT;
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

#endif /* ADC1 || ADC2 || ADC3 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

