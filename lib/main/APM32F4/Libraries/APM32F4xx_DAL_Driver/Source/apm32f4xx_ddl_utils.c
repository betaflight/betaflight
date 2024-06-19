/**
  *
  * @file    apm32f4xx_ddl_utils.c
  * @brief   UTILS DDL module driver.
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
  *
  */
/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_utils.h"
#include "apm32f4xx_ddl_rcm.h"
#include "apm32f4xx_ddl_system.h"
#include "apm32f4xx_ddl_pmu.h"
#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif /* USE_FULL_ASSERT */

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

/** @addtogroup UTILS_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup UTILS_DDL_Private_Constants
  * @{
  */
#if defined(RCM_MAX_FREQUENCY_SCALE1)
#define UTILS_MAX_FREQUENCY_SCALE1  RCM_MAX_FREQUENCY           /*!< Maximum frequency for system clock at power scale1, in Hz */
#endif /*RCM_MAX_FREQUENCY_SCALE1 */
#define UTILS_MAX_FREQUENCY_SCALE2  RCM_MAX_FREQUENCY_SCALE2    /*!< Maximum frequency for system clock at power scale2, in Hz */
#if defined(RCM_MAX_FREQUENCY_SCALE3)
#define UTILS_MAX_FREQUENCY_SCALE3  RCM_MAX_FREQUENCY_SCALE3    /*!< Maximum frequency for system clock at power scale3, in Hz */
#endif /* MAX_FREQUENCY_SCALE3 */

/* Defines used for PLL range */
#define UTILS_PLLVCO_INPUT_MIN      RCM_PLLVCO_INPUT_MIN        /*!< Frequency min for PLLVCO input, in Hz   */
#define UTILS_PLLVCO_INPUT_MAX      RCM_PLLVCO_INPUT_MAX        /*!< Frequency max for PLLVCO input, in Hz   */
#define UTILS_PLLVCO_OUTPUT_MIN     RCM_PLLVCO_OUTPUT_MIN       /*!< Frequency min for PLLVCO output, in Hz  */
#define UTILS_PLLVCO_OUTPUT_MAX     RCM_PLLVCO_OUTPUT_MAX       /*!< Frequency max for PLLVCO output, in Hz  */

/* Defines used for HSE range */
#define UTILS_HSE_FREQUENCY_MIN      4000000U        /*!< Frequency min for HSE frequency, in Hz   */
#define UTILS_HSE_FREQUENCY_MAX     26000000U        /*!< Frequency max for HSE frequency, in Hz   */

/* Defines used for FLASH latency according to HCLK Frequency */
#if defined(FLASH_SCALE1_LATENCY1_FREQ)
#define UTILS_SCALE1_LATENCY1_FREQ  FLASH_SCALE1_LATENCY1_FREQ /*!< HCLK frequency to set FLASH latency 1 in power scale 1 */
#endif
#if defined(FLASH_SCALE1_LATENCY2_FREQ)
#define UTILS_SCALE1_LATENCY2_FREQ  FLASH_SCALE1_LATENCY2_FREQ /*!< HCLK frequency to set FLASH latency 2 in power scale 1 */
#endif
#if defined(FLASH_SCALE1_LATENCY3_FREQ)
#define UTILS_SCALE1_LATENCY3_FREQ  FLASH_SCALE1_LATENCY3_FREQ /*!< HCLK frequency to set FLASH latency 3 in power scale 1 */
#endif
#if defined(FLASH_SCALE1_LATENCY4_FREQ)
#define UTILS_SCALE1_LATENCY4_FREQ  FLASH_SCALE1_LATENCY4_FREQ /*!< HCLK frequency to set FLASH latency 4 in power scale 1 */
#endif
#if defined(FLASH_SCALE1_LATENCY5_FREQ)
#define UTILS_SCALE1_LATENCY5_FREQ  FLASH_SCALE1_LATENCY5_FREQ /*!< HCLK frequency to set FLASH latency 5 in power scale 1 */
#endif
#define UTILS_SCALE2_LATENCY1_FREQ  FLASH_SCALE2_LATENCY1_FREQ /*!< HCLK frequency to set FLASH latency 1 in power scale 2 */
#define UTILS_SCALE2_LATENCY2_FREQ  FLASH_SCALE2_LATENCY2_FREQ /*!< HCLK frequency to set FLASH latency 2 in power scale 2 */
#if defined(FLASH_SCALE2_LATENCY3_FREQ)
#define UTILS_SCALE2_LATENCY3_FREQ  FLASH_SCALE2_LATENCY3_FREQ /*!< HCLK frequency to set FLASH latency 2 in power scale 2 */
#endif
#if defined(FLASH_SCALE2_LATENCY4_FREQ)
#define UTILS_SCALE2_LATENCY4_FREQ  FLASH_SCALE2_LATENCY4_FREQ /*!< HCLK frequency to set FLASH latency 4 in power scale 2 */
#endif
#if defined(FLASH_SCALE2_LATENCY5_FREQ)
#define UTILS_SCALE2_LATENCY5_FREQ  FLASH_SCALE2_LATENCY5_FREQ /*!< HCLK frequency to set FLASH latency 5 in power scale 2 */
#endif
#if defined(FLASH_SCALE3_LATENCY1_FREQ)
#define UTILS_SCALE3_LATENCY1_FREQ  FLASH_SCALE3_LATENCY1_FREQ /*!< HCLK frequency to set FLASH latency 1 in power scale 3 */
#endif
#if defined(FLASH_SCALE3_LATENCY2_FREQ)
#define UTILS_SCALE3_LATENCY2_FREQ  FLASH_SCALE3_LATENCY2_FREQ /*!< HCLK frequency to set FLASH latency 2 in power scale 3 */
#endif
#if defined(FLASH_SCALE3_LATENCY3_FREQ)
#define UTILS_SCALE3_LATENCY3_FREQ  FLASH_SCALE3_LATENCY3_FREQ /*!< HCLK frequency to set FLASH latency 3 in power scale 3 */
#endif
#if defined(FLASH_SCALE3_LATENCY4_FREQ)
#define UTILS_SCALE3_LATENCY4_FREQ  FLASH_SCALE3_LATENCY4_FREQ /*!< HCLK frequency to set FLASH latency 4 in power scale 3 */
#endif
#if defined(FLASH_SCALE3_LATENCY5_FREQ)
#define UTILS_SCALE3_LATENCY5_FREQ  FLASH_SCALE3_LATENCY5_FREQ /*!< HCLK frequency to set FLASH latency 5 in power scale 3 */
#endif
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup UTILS_DDL_Private_Macros
  * @{
  */
#define IS_DDL_UTILS_SYSCLK_DIV(__VALUE__) (((__VALUE__) == DDL_RCM_SYSCLK_DIV_1)   \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_2)   \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_4)   \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_8)   \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_16)  \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_64)  \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_128) \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_256) \
                                        || ((__VALUE__) == DDL_RCM_SYSCLK_DIV_512))

#define IS_DDL_UTILS_APB1_DIV(__VALUE__) (((__VALUE__) == DDL_RCM_APB1_DIV_1) \
                                      || ((__VALUE__) == DDL_RCM_APB1_DIV_2) \
                                      || ((__VALUE__) == DDL_RCM_APB1_DIV_4) \
                                      || ((__VALUE__) == DDL_RCM_APB1_DIV_8) \
                                      || ((__VALUE__) == DDL_RCM_APB1_DIV_16))

#define IS_DDL_UTILS_APB2_DIV(__VALUE__) (((__VALUE__) == DDL_RCM_APB2_DIV_1) \
                                      || ((__VALUE__) == DDL_RCM_APB2_DIV_2) \
                                      || ((__VALUE__) == DDL_RCM_APB2_DIV_4) \
                                      || ((__VALUE__) == DDL_RCM_APB2_DIV_8) \
                                      || ((__VALUE__) == DDL_RCM_APB2_DIV_16))

#define IS_DDL_UTILS_PLLB_VALUE(__VALUE__) (((__VALUE__) == DDL_RCM_PLLB_DIV_2)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_3)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_4)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_5)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_6)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_7)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_8)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_9)  \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_10) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_11) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_12) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_13) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_14) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_15) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_16) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_17) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_18) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_19) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_20) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_21) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_22) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_23) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_24) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_25) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_26) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_27) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_28) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_29) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_30) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_31) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_32) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_33) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_34) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_35) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_36) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_37) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_38) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_39) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_40) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_41) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_42) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_43) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_44) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_45) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_46) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_47) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_48) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_49) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_50) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_51) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_52) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_53) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_54) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_55) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_56) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_57) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_58) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_59) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_60) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_61) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_62) \
                                        || ((__VALUE__) == DDL_RCM_PLLB_DIV_63))

#define IS_DDL_UTILS_PLL1A_VALUE(__VALUE__) ((RCM_PLL1A_MIN_VALUE <= (__VALUE__)) && ((__VALUE__) <= RCM_PLL1A_MAX_VALUE))

#define IS_DDL_UTILS_PLL1C_VALUE(__VALUE__) (((__VALUE__) == DDL_RCM_PLL1C_DIV_2) \
                                        || ((__VALUE__) == DDL_RCM_PLL1C_DIV_4) \
                                        || ((__VALUE__) == DDL_RCM_PLL1C_DIV_6) \
                                        || ((__VALUE__) == DDL_RCM_PLL1C_DIV_8))

#define IS_DDL_UTILS_PLLVCO_INPUT(__VALUE__)  ((UTILS_PLLVCO_INPUT_MIN <= (__VALUE__)) && ((__VALUE__) <= UTILS_PLLVCO_INPUT_MAX))

#define IS_DDL_UTILS_PLLVCO_OUTPUT(__VALUE__) ((UTILS_PLLVCO_OUTPUT_MIN <= (__VALUE__)) && ((__VALUE__) <= UTILS_PLLVCO_OUTPUT_MAX))

#if !defined(RCM_MAX_FREQUENCY_SCALE1)
#define IS_DDL_UTILS_PLL_FREQUENCY(__VALUE__) ((DDL_PMU_GetRegulVoltageScaling() == DDL_PMU_REGU_VOLTAGE_SCALE2) ? ((__VALUE__) <= UTILS_MAX_FREQUENCY_SCALE2) : \
                                             ((__VALUE__) <= UTILS_MAX_FREQUENCY_SCALE3))
                                             
#elif defined(RCM_MAX_FREQUENCY_SCALE3) 
#define IS_DDL_UTILS_PLL_FREQUENCY(__VALUE__) ((DDL_PMU_GetRegulVoltageScaling() == DDL_PMU_REGU_VOLTAGE_SCALE1) ? ((__VALUE__) <= UTILS_MAX_FREQUENCY_SCALE1) : \
                                              (DDL_PMU_GetRegulVoltageScaling() == DDL_PMU_REGU_VOLTAGE_SCALE2) ? ((__VALUE__) <= UTILS_MAX_FREQUENCY_SCALE2) : \
                                              ((__VALUE__) <= UTILS_MAX_FREQUENCY_SCALE3))

#else
#define IS_DDL_UTILS_PLL_FREQUENCY(__VALUE__) ((DDL_PMU_GetRegulVoltageScaling() == DDL_PMU_REGU_VOLTAGE_SCALE1) ? ((__VALUE__) <= UTILS_MAX_FREQUENCY_SCALE1) : \
                                             ((__VALUE__) <= UTILS_MAX_FREQUENCY_SCALE2))

#endif /* RCM_MAX_FREQUENCY_SCALE1*/
#define IS_DDL_UTILS_HSE_BYPASS(__STATE__) (((__STATE__) == DDL_UTILS_HSEBYPASS_ON) \
                                        || ((__STATE__) == DDL_UTILS_HSEBYPASS_OFF))

#define IS_DDL_UTILS_HSE_FREQUENCY(__FREQUENCY__) (((__FREQUENCY__) >= UTILS_HSE_FREQUENCY_MIN) && ((__FREQUENCY__) <= UTILS_HSE_FREQUENCY_MAX))
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/** @defgroup UTILS_DDL_Private_Functions UTILS Private functions
  * @{
  */
static uint32_t    UTILS_GetPLLOutputFrequency(uint32_t PLL_InputFrequency,
                                               DDL_UTILS_PLLInitTypeDef *UTILS_PLLInitStruct);
static ErrorStatus UTILS_EnablePLLAndSwitchSystem(uint32_t SYSCLK_Frequency, DDL_UTILS_ClkInitTypeDef *UTILS_ClkInitStruct);
static ErrorStatus UTILS_PLL_IsBusy(void);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup UTILS_DDL_Exported_Functions
  * @{
  */

/** @addtogroup UTILS_DDL_EF_DELAY
  * @{
  */

/**
  * @brief  This function configures the Cortex-M SysTick source to have 1ms time base.
  * @note   When a RTOS is used, it is recommended to avoid changing the Systick
  *         configuration by calling this function, for a delay use rather osDelay RTOS service.
  * @param  HCLKFrequency HCLK frequency in Hz
  * @note   HCLK frequency can be calculated thanks to RCM helper macro or function @ref DDL_RCM_GetSystemClocksFreq
  * @retval None
  */
void DDL_Init1msTick(uint32_t HCLKFrequency)
{
  /* Use frequency provided in argument */
  DDL_InitTick(HCLKFrequency, 1000U);
}

/**
  * @brief  This function provides accurate delay (in milliseconds) based
  *         on SysTick counter flag
  * @note   When a RTOS is used, it is recommended to avoid using blocking delay
  *         and use rather osDelay service.
  * @note   To respect 1ms timebase, user should call @ref DDL_Init1msTick function which
  *         will configure Systick to 1ms
  * @param  Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void DDL_mDelay(uint32_t Delay)
{
  __IO uint32_t  tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
  /* Add this code to indicate that local variable is not used */
  ((void)tmp);

  /* Add a period to guaranty minimum wait */
  if(Delay < DDL_MAX_DELAY)
  {
    Delay++;
  }

  while (Delay)
  {
    if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
    {
      Delay--;
    }
  }
}

/**
  * @}
  */

/** @addtogroup UTILS_EF_SYSTEM
  *  @brief    System Configuration functions
  *
  @verbatim
 ===============================================================================
           ##### System Configuration functions #####
 ===============================================================================
    [..]
         System, AHB and APB buses clocks configuration

         (+) The maximum frequency of the SYSCLK, HCLK, PCLK1 and PCLK2 is 180000000 Hz.
  @endverbatim
  @internal
             Depending on the device voltage range, the maximum frequency should be
             adapted accordingly to the Refenece manual.
  @endinternal
  * @{
  */

/**
  * @brief  This function sets directly SystemCoreClock CMSIS variable.
  * @note   Variable can be calculated also through SystemCoreClockUpdate function.
  * @param  HCLKFrequency HCLK frequency in Hz (can be calculated thanks to RCM helper macro)
  * @retval None
  */
void DDL_SetSystemCoreClock(uint32_t HCLKFrequency)
{
  /* HCLK clock frequency */
  SystemCoreClock = HCLKFrequency;
}

/**
  * @brief  Update number of Flash wait states in line with new frequency and current
            voltage range.
  * @note   This Function support ONLY devices with supply voltage (voltage range) between 2.7V and 3.6V
  * @param  HCLK_Frequency  HCLK frequency
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Latency has been modified
  *          - ERROR: Latency cannot be modified
  */
ErrorStatus DDL_SetFlashLatency(uint32_t HCLK_Frequency)
{
  uint32_t timeout;
  uint32_t getlatency;
  uint32_t latency = DDL_FLASH_LATENCY_0;  /* default value 0WS */
  ErrorStatus status = SUCCESS;


  /* Frequency cannot be equal to 0 */
  if(HCLK_Frequency == 0U)
  {
    status = ERROR;
  }
  else
  {
    if(DDL_PMU_GetRegulVoltageScaling() == DDL_PMU_REGU_VOLTAGE_SCALE1)
    {
#if defined (UTILS_SCALE1_LATENCY5_FREQ)
      if((HCLK_Frequency > UTILS_SCALE1_LATENCY5_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_5;
      }
#endif /*UTILS_SCALE1_LATENCY5_FREQ */
#if defined (UTILS_SCALE1_LATENCY4_FREQ)
      if((HCLK_Frequency > UTILS_SCALE1_LATENCY4_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_4;
      }
#endif /* UTILS_SCALE1_LATENCY4_FREQ */
#if defined (UTILS_SCALE1_LATENCY3_FREQ)
      if((HCLK_Frequency > UTILS_SCALE1_LATENCY3_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_3;
      }
#endif /* UTILS_SCALE1_LATENCY3_FREQ */
#if defined (UTILS_SCALE1_LATENCY2_FREQ) 
      if((HCLK_Frequency > UTILS_SCALE1_LATENCY2_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_2;
      }
      else
      {
        if((HCLK_Frequency > UTILS_SCALE1_LATENCY1_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
        {
          latency = DDL_FLASH_LATENCY_1;
        }
      }
#endif /* UTILS_SCALE1_LATENCY2_FREQ */
    }
    if(DDL_PMU_GetRegulVoltageScaling() == DDL_PMU_REGU_VOLTAGE_SCALE2)
    {
#if defined (UTILS_SCALE2_LATENCY5_FREQ)
      if((HCLK_Frequency > UTILS_SCALE2_LATENCY5_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_5;
      }
#endif /*UTILS_SCALE1_LATENCY5_FREQ */
#if defined (UTILS_SCALE2_LATENCY4_FREQ)
      if((HCLK_Frequency > UTILS_SCALE2_LATENCY4_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_4;
      }
#endif /*UTILS_SCALE1_LATENCY4_FREQ */
#if defined (UTILS_SCALE2_LATENCY3_FREQ)
      if((HCLK_Frequency > UTILS_SCALE2_LATENCY3_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_3;
      }
#endif /*UTILS_SCALE1_LATENCY3_FREQ */
      if((HCLK_Frequency > UTILS_SCALE2_LATENCY2_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_2;
      }
      else
      {
        if((HCLK_Frequency > UTILS_SCALE2_LATENCY1_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
        {
          latency = DDL_FLASH_LATENCY_1;
        }
      }
    }
#if defined (DDL_PMU_REGU_VOLTAGE_SCALE3)
    if(DDL_PMU_GetRegulVoltageScaling() == DDL_PMU_REGU_VOLTAGE_SCALE3)
    {
#if defined (UTILS_SCALE3_LATENCY3_FREQ)
      if((HCLK_Frequency > UTILS_SCALE3_LATENCY3_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_3;
      }
#endif /*UTILS_SCALE1_LATENCY3_FREQ */
#if defined (UTILS_SCALE3_LATENCY2_FREQ)
      if((HCLK_Frequency > UTILS_SCALE3_LATENCY2_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
      {
        latency = DDL_FLASH_LATENCY_2;
      }
      else
      {
        if((HCLK_Frequency > UTILS_SCALE3_LATENCY1_FREQ)&&(latency == DDL_FLASH_LATENCY_0))
        {
          latency = DDL_FLASH_LATENCY_1;
        }
      }
    }
#endif /*UTILS_SCALE1_LATENCY2_FREQ */
#endif /* DDL_PMU_REGU_VOLTAGE_SCALE3 */

    DDL_FLASH_SetLatency(latency);
    /* Check that the new number of wait states is taken into account to access the Flash
       memory by reading the FLASH_ACCTRL register */
    timeout = 2;
    do
    {
    /* Wait for Flash latency to be updated */
    getlatency = DDL_FLASH_GetLatency();
    timeout--;
    } while ((getlatency != latency) && (timeout > 0));

    if(getlatency != latency)
    {
      status = ERROR;
    }
    else
    {
      status = SUCCESS;
    }
  }
  return status;
}

/**
  * @brief  This function configures system clock at maximum frequency with HSI as clock source of the PLL
  * @note   The application need to ensure that PLL is disabled.
  * @note   Function is based on the following formula:
  *         - PLL output frequency = (((HSI frequency / PLLB) * PLL1A) / PLL1C)
  *         - PLLB: ensure that the VCO input frequency ranges from @ref RCM_PLLVCO_INPUT_MIN to @ref RCM_PLLVCO_INPUT_MAX (PLLVCO_input = HSI frequency / PLLB)
  *         - PLL1A: ensure that the VCO output frequency is between @ref RCM_PLLVCO_OUTPUT_MIN and @ref RCM_PLLVCO_OUTPUT_MAX (PLLVCO_output = PLLVCO_input * PLL1A)
  *         - PLL1C: ensure that max frequency at 180000000 Hz is reach (PLLVCO_output / PLL1C)
  * @param  UTILS_PLLInitStruct pointer to a @ref DDL_UTILS_PLLInitTypeDef structure that contains
  *                             the configuration information for the PLL.
  * @param  UTILS_ClkInitStruct pointer to a @ref DDL_UTILS_ClkInitTypeDef structure that contains
  *                             the configuration information for the BUS prescalers.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Max frequency configuration done
  *          - ERROR: Max frequency configuration not done
  */
ErrorStatus DDL_PLL_ConfigSystemClock_HSI(DDL_UTILS_PLLInitTypeDef *UTILS_PLLInitStruct,
                                         DDL_UTILS_ClkInitTypeDef *UTILS_ClkInitStruct)
{
  ErrorStatus status = SUCCESS;
  uint32_t pllfreq = 0U;

  /* Check if one of the PLL is enabled */
  if(UTILS_PLL_IsBusy() == SUCCESS)
  {
    /* Calculate the new PLL output frequency */
    pllfreq = UTILS_GetPLLOutputFrequency(HSI_VALUE, UTILS_PLLInitStruct);

    /* Enable HSI if not enabled */
    if(DDL_RCM_HSI_IsReady() != 1U)
    {
      DDL_RCM_HSI_Enable();
      while (DDL_RCM_HSI_IsReady() != 1U)
      {
        /* Wait for HSI ready */
      }
    }

    /* Configure PLL */
    DDL_RCM_PLL_ConfigDomain_SYS(DDL_RCM_PLLSOURCE_HSI, UTILS_PLLInitStruct->PLLB, UTILS_PLLInitStruct->PLL1A,
                                UTILS_PLLInitStruct->PLL1C);

    /* Enable PLL and switch system clock to PLL */
    status = UTILS_EnablePLLAndSwitchSystem(pllfreq, UTILS_ClkInitStruct);
  }
  else
  {
    /* Current PLL configuration cannot be modified */
    status = ERROR;
  }

  return status;
}

/**
  * @brief  This function configures system clock with HSE as clock source of the PLL
  * @note   The application need to ensure that PLL is disabled.
  *         - PLL output frequency = (((HSI frequency / PLLB) * PLL1A) / PLL1C)
  *         - PLLB: ensure that the VCO input frequency ranges from @ref RCM_PLLVCO_INPUT_MIN to @ref RCM_PLLVCO_INPUT_MAX (PLLVCO_input = HSI frequency / PLLB)
  *         - PLL1A: ensure that the VCO output frequency is between @ref RCM_PLLVCO_OUTPUT_MIN and @ref RCM_PLLVCO_OUTPUT_MAX (PLLVCO_output = PLLVCO_input * PLL1A)
  *         - PLL1C: ensure that max frequency at 180000000 Hz is reach (PLLVCO_output / PLL1C)
  * @param  HSEFrequency Value between Min_Data = 4000000 and Max_Data = 26000000
  * @param  HSEBypass This parameter can be one of the following values:
  *         @arg @ref DDL_UTILS_HSEBYPASS_ON
  *         @arg @ref DDL_UTILS_HSEBYPASS_OFF
  * @param  UTILS_PLLInitStruct pointer to a @ref DDL_UTILS_PLLInitTypeDef structure that contains
  *                             the configuration information for the PLL.
  * @param  UTILS_ClkInitStruct pointer to a @ref DDL_UTILS_ClkInitTypeDef structure that contains
  *                             the configuration information for the BUS prescalers.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Max frequency configuration done
  *          - ERROR: Max frequency configuration not done
  */
ErrorStatus DDL_PLL_ConfigSystemClock_HSE(uint32_t HSEFrequency, uint32_t HSEBypass,
                                         DDL_UTILS_PLLInitTypeDef *UTILS_PLLInitStruct, DDL_UTILS_ClkInitTypeDef *UTILS_ClkInitStruct)
{
  ErrorStatus status = SUCCESS;
  uint32_t pllfreq = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_DDL_UTILS_HSE_FREQUENCY(HSEFrequency));
  ASSERT_PARAM(IS_DDL_UTILS_HSE_BYPASS(HSEBypass));

  /* Check if one of the PLL is enabled */
  if(UTILS_PLL_IsBusy() == SUCCESS)
  {
    /* Calculate the new PLL output frequency */
    pllfreq = UTILS_GetPLLOutputFrequency(HSEFrequency, UTILS_PLLInitStruct);

    /* Enable HSE if not enabled */
    if(DDL_RCM_HSE_IsReady() != 1U)
    {
      /* Check if need to enable HSE bypass feature or not */
      if(HSEBypass == DDL_UTILS_HSEBYPASS_ON)
      {
        DDL_RCM_HSE_EnableBypass();
      }
      else
      {
        DDL_RCM_HSE_DisableBypass();
      }

      /* Enable HSE */
      DDL_RCM_HSE_Enable();
      while (DDL_RCM_HSE_IsReady() != 1U)
      {
        /* Wait for HSE ready */
      }
    }

    /* Configure PLL */
    DDL_RCM_PLL_ConfigDomain_SYS(DDL_RCM_PLLSOURCE_HSE, UTILS_PLLInitStruct->PLLB, UTILS_PLLInitStruct->PLL1A,
                                UTILS_PLLInitStruct->PLL1C);

    /* Enable PLL and switch system clock to PLL */
    status = UTILS_EnablePLLAndSwitchSystem(pllfreq, UTILS_ClkInitStruct);
  }
  else
  {
    /* Current PLL configuration cannot be modified */
    status = ERROR;
  }

  return status;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup UTILS_DDL_Private_Functions
  * @{
  */
/**
  * @brief  Function to check that PLL can be modified
  * @param  PLL_InputFrequency  PLL input frequency (in Hz)
  * @param  UTILS_PLLInitStruct pointer to a @ref DDL_UTILS_PLLInitTypeDef structure that contains
  *                             the configuration information for the PLL.
  * @retval PLL output frequency (in Hz)
  */
static uint32_t UTILS_GetPLLOutputFrequency(uint32_t PLL_InputFrequency, DDL_UTILS_PLLInitTypeDef *UTILS_PLLInitStruct)
{
  uint32_t pllfreq = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_DDL_UTILS_PLLB_VALUE(UTILS_PLLInitStruct->PLLB));
  ASSERT_PARAM(IS_DDL_UTILS_PLL1A_VALUE(UTILS_PLLInitStruct->PLL1A));
  ASSERT_PARAM(IS_DDL_UTILS_PLL1C_VALUE(UTILS_PLLInitStruct->PLL1C));
  
  /* Check different PLL parameters according to RM                          */
  /*  - PLLB: ensure that the VCO input frequency ranges from @ref UTILS_PLLVCO_INPUT_MIN to @ref UTILS_PLLVCO_INPUT_MAX MHz.   */
  pllfreq = PLL_InputFrequency / (UTILS_PLLInitStruct->PLLB & (RCM_PLL1CFG_PLLB >> RCM_PLL1CFG_PLLB_Pos));
  ASSERT_PARAM(IS_DDL_UTILS_PLLVCO_INPUT(pllfreq));

  /*  - PLL1A: ensure that the VCO output frequency is between @ref UTILS_PLLVCO_OUTPUT_MIN and @ref UTILS_PLLVCO_OUTPUT_MAX .*/
  pllfreq = pllfreq * (UTILS_PLLInitStruct->PLL1A & (RCM_PLL1CFG_PLL1A >> RCM_PLL1CFG_PLL1A_Pos));
  ASSERT_PARAM(IS_DDL_UTILS_PLLVCO_OUTPUT(pllfreq));
  
  /*  - PLL1C: ensure that max frequency at @ref RCM_MAX_FREQUENCY Hz is reached     */
  pllfreq = pllfreq / (((UTILS_PLLInitStruct->PLL1C >> RCM_PLL1CFG_PLL1C_Pos) + 1) * 2);
  ASSERT_PARAM(IS_DDL_UTILS_PLL_FREQUENCY(pllfreq));

  return pllfreq;
}

/**
  * @brief  Function to check that PLL can be modified
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PLL modification can be done
  *          - ERROR: PLL is busy
  */
static ErrorStatus UTILS_PLL_IsBusy(void)
{
  ErrorStatus status = SUCCESS;

  /* Check if PLL is busy*/
  if(DDL_RCM_PLL_IsReady() != 0U)
  {
    /* PLL configuration cannot be modified */
    status = ERROR;
  }

#if defined(RCM_PLLI2S_SUPPORT)
  /* Check if PLLI2S is busy*/
  if(DDL_RCM_PLLI2S_IsReady() != 0U)
  {
    /* PLLI2S configuration cannot be modified */
    status = ERROR;
  }
#endif /*RCM_PLLI2S_SUPPORT*/
  return status;
}

/**
  * @brief  Function to enable PLL and switch system clock to PLL
  * @param  SYSCLK_Frequency SYSCLK frequency
  * @param  UTILS_ClkInitStruct pointer to a @ref DDL_UTILS_ClkInitTypeDef structure that contains
  *                             the configuration information for the BUS prescalers.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: No problem to switch system to PLL
  *          - ERROR: Problem to switch system to PLL
  */
static ErrorStatus UTILS_EnablePLLAndSwitchSystem(uint32_t SYSCLK_Frequency, DDL_UTILS_ClkInitTypeDef *UTILS_ClkInitStruct)
{
  ErrorStatus status = SUCCESS;
  uint32_t hclk_frequency = 0U;

  ASSERT_PARAM(IS_DDL_UTILS_SYSCLK_DIV(UTILS_ClkInitStruct->AHBCLKDivider));
  ASSERT_PARAM(IS_DDL_UTILS_APB1_DIV(UTILS_ClkInitStruct->APB1CLKDivider));
  ASSERT_PARAM(IS_DDL_UTILS_APB2_DIV(UTILS_ClkInitStruct->APB2CLKDivider));

  /* Calculate HCLK frequency */
  hclk_frequency = __DDL_RCM_CALC_HCLK_FREQ(SYSCLK_Frequency, UTILS_ClkInitStruct->AHBCLKDivider);

  /* Increasing the number of wait states because of higher CPU frequency */
  if(SystemCoreClock < hclk_frequency)
  {
    /* Set FLASH latency to highest latency */
    status = DDL_SetFlashLatency(hclk_frequency);
  }

  /* Update system clock configuration */
  if(status == SUCCESS)
  {
    /* Enable PLL */
    DDL_RCM_PLL_Enable();
    while (DDL_RCM_PLL_IsReady() != 1U)
    {
      /* Wait for PLL ready */
    }

    /* Sysclk activation on the main PLL */
    DDL_RCM_SetAHBPrescaler(UTILS_ClkInitStruct->AHBCLKDivider);
    DDL_RCM_SetSysClkSource(DDL_RCM_SYS_CLKSOURCE_PLL);
    while (DDL_RCM_GetSysClkSource() != DDL_RCM_SYS_CLKSOURCE_STATUS_PLL)
    {
      /* Wait for system clock switch to PLL */
    }

    /* Set APB1 & APB2 prescaler*/
    DDL_RCM_SetAPB1Prescaler(UTILS_ClkInitStruct->APB1CLKDivider);
    DDL_RCM_SetAPB2Prescaler(UTILS_ClkInitStruct->APB2CLKDivider);
  }
    
  /* Decreasing the number of wait states because of lower CPU frequency */
  if(SystemCoreClock > hclk_frequency)
  {
    /* Set FLASH latency to lowest latency */
    status = DDL_SetFlashLatency(hclk_frequency);
  }

  /* Update SystemCoreClock variable */
  if(status == SUCCESS)
  {
    DDL_SetSystemCoreClock(hclk_frequency);
  }

  return status;
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
