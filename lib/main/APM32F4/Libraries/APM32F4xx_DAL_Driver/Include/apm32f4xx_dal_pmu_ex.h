/**
  *
  * @file    apm32f4xx_dal_pmu_ex.h
  * @brief   Header file of PMU DAL Extension module.
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
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_PMU_EX_H
#define APM32F4xx_DAL_PMU_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup PMUEx
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/ 
/* Exported constants --------------------------------------------------------*/
/** @defgroup PMUEx_Exported_Constants PMUEx Exported Constants
  * @{
  */

/** @defgroup PMUEx_Regulator_Voltage_Scale PMUEx Regulator Voltage Scale
  * @{
  */
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define PMU_REGULATOR_VOLTAGE_SCALE1         PMU_CTRL_VOSSEL             /* Scale 1 mode(default value at reset): the maximum value of fHCLK = 168 MHz. */
#define PMU_REGULATOR_VOLTAGE_SCALE2         0x00000000U            /* Scale 2 mode: the maximum value of fHCLK = 144 MHz. */
#else
#define PMU_REGULATOR_VOLTAGE_SCALE1         PMU_CTRL_VOSSEL             /* Scale 1 mode(default value at reset): the maximum value of fHCLK is 168 MHz. It can be extended to
                                                                       180 MHz by activating the over-drive mode. */
#define PMU_REGULATOR_VOLTAGE_SCALE2         PMU_CTRL_VOSSEL_1           /* Scale 2 mode: the maximum value of fHCLK is 144 MHz. It can be extended to
                                                                       168 MHz by activating the over-drive mode. */
#define PMU_REGULATOR_VOLTAGE_SCALE3         PMU_CTRL_VOSSEL_0           /* Scale 3 mode: the maximum value of fHCLK is 120 MHz. */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */ 
/**
  * @}
  */

/**
  * @}
  */ 
  
/* Exported macro ------------------------------------------------------------*/
/** @defgroup PMUEx_Exported_Constants PMUEx Exported Constants
  *  @{
  */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
/** @brief  macros configure the main internal regulator output voltage.
  * @param  __REGULATOR__ specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption when the device does
  *         not operate at the maximum frequency (refer to the datasheets for more details).
  *          This parameter can be one of the following values:
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output Scale 1 mode
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output Scale 2 mode
  * @retval None
  */
#define __DAL_PMU_VOLTAGESCALING_CONFIG(__REGULATOR__) do {                                                     \
                                                            __IO uint32_t tmpreg = 0x00U;                        \
                                                            MODIFY_REG(PMU->CTRL, PMU_CTRL_VOSSEL, (__REGULATOR__));   \
                                                            /* Delay after an RCC peripheral clock enabling */  \
                                                            tmpreg = READ_BIT(PMU->CTRL, PMU_CTRL_VOSSEL);             \
                                                            UNUSED(tmpreg);                                     \
                                                          } while(0U)
#else
/** @brief  macros configure the main internal regulator output voltage.
  * @param  __REGULATOR__ specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption when the device does
  *         not operate at the maximum frequency (refer to the datasheets for more details).
  *          This parameter can be one of the following values:
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output Scale 1 mode
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output Scale 2 mode
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output Scale 3 mode
  * @retval None
  */
#define __DAL_PMU_VOLTAGESCALING_CONFIG(__REGULATOR__) do {                                                     \
                                                            __IO uint32_t tmpreg = 0x00U;                        \
                                                            MODIFY_REG(PMU->CTRL, PMU_CTRL_VOSSEL, (__REGULATOR__));   \
                                                            /* Delay after an RCC peripheral clock enabling */  \
                                                            tmpreg = READ_BIT(PMU->CTRL, PMU_CTRL_VOSSEL);             \
                                                            UNUSED(tmpreg);                                     \
                                                          } while(0U)
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */ 

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PMUEx_Exported_Functions PMUEx Exported Functions
  *  @{
  */
 
/** @addtogroup PMUEx_Exported_Functions_Group1
  * @{
  */
void DAL_PMUEx_EnableFlashPowerDown(void);
void DAL_PMUEx_DisableFlashPowerDown(void); 
DAL_StatusTypeDef DAL_PMUEx_EnableBkUpReg(void);
DAL_StatusTypeDef DAL_PMUEx_DisableBkUpReg(void); 
uint32_t DAL_PMUEx_GetVoltageRange(void);
DAL_StatusTypeDef DAL_PMUEx_ControlVoltageScaling(uint32_t VoltageScaling);

#if defined(APM32F411xx)
void DAL_PMUEx_EnableMainRegulatorLowVoltage(void);
void DAL_PMUEx_DisableMainRegulatorLowVoltage(void);
void DAL_PMUEx_EnableLowRegulatorLowVoltage(void);
void DAL_PMUEx_DisableLowRegulatorLowVoltage(void);
#endif /* APM32F411xx */

/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup PMUEx_Private_Constants PMUEx Private Constants
  * @{
  */

/** @defgroup PMUEx_register_alias_address PMUEx Register alias address
  * @{
  */
/* ------------- PMU registers bit address in the alias region ---------------*/
/* --- CTRL Register ---*/
/* Alias word address of FPDS bit */
#define FPDSM_BIT_NUMBER         PMU_CTRL_FPDSM_Pos
#define CTRL_FPDSM_BB            (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (FPDSM_BIT_NUMBER * 4U))

/* Alias word address of ODEN bit   */
#define ODEN_BIT_NUMBER          PMU_CTRL_ODEN_Pos
#define CTRL_ODEN_BB             (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (ODEN_BIT_NUMBER * 4U))

/* Alias word address of ODSWEN bit */
#define ODSWEN_BIT_NUMBER        PMU_CTRL_ODSWEN_Pos
#define CTRL_ODSWEN_BB           (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (ODSWEN_BIT_NUMBER * 4U))
    
/* Alias word address of MRLVDS bit */
#define MRLV_BIT_NUMBER          PMU_CTRL_MRLV_Pos
#define CTRL_MRLV_BB             (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (MRLV_BIT_NUMBER * 4U))

/* Alias word address of LPLVDS bit */
#define LPRLV_BIT_NUMBER         PMU_CTRL_LPRLV_Pos
#define CTRL_LPRLV_BB            (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (LPRLV_BIT_NUMBER  * 4U))

 /**
  * @}
  */

/** @defgroup PMUEx_CSTS_register_alias PMUx CSTS Register alias address
  * @{
  */  
/* --- CSTS Register ---*/
/* Alias word address of BRE bit */
#define BRE_BIT_NUMBER   PMU_CSTS_BKPREN_Pos
#define CSTS_BKPREN_BB   (uint32_t)(PERIPH_BB_BASE + (PMU_CSTS_OFFSET_BB * 32U) + (BRE_BIT_NUMBER * 4U))

/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PMUEx_Private_Macros PMUEx Private Macros
  * @{
  */

/** @defgroup PMUEx_IS_PMU_Definitions PMUEx Private macros to check input parameters
  * @{
  */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define IS_PMU_VOLTAGE_SCALING_RANGE(VOLTAGE) (((VOLTAGE) == PMU_REGULATOR_VOLTAGE_SCALE1) || \
                                               ((VOLTAGE) == PMU_REGULATOR_VOLTAGE_SCALE2))
#else
#define IS_PMU_VOLTAGE_SCALING_RANGE(VOLTAGE) (((VOLTAGE) == PMU_REGULATOR_VOLTAGE_SCALE1) || \
                                               ((VOLTAGE) == PMU_REGULATOR_VOLTAGE_SCALE2) || \
                                               ((VOLTAGE) == PMU_REGULATOR_VOLTAGE_SCALE3))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */ 

#define IS_PMU_WAKEUP_PIN(PIN) ((PIN) == PMU_WAKEUP_PIN1)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif


#endif /* APM32F4xx_DAL_PMU_EX_H */
