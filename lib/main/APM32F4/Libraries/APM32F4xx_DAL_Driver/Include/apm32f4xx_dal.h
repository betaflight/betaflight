/**
  *
  * @file    apm32f4xx_dal.h
  * @brief   This file contains all the functions prototypes for the DAL 
  *          module driver.
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
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_H
#define APM32F4xx_DAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_cfg.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup DAL
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup DAL_Exported_Constants DAL Exported Constants
  * @{
  */

/** @defgroup DAL_TICK_FREQ Tick Frequency
  * @{
  */
typedef enum
{
  DAL_TICK_FREQ_10HZ         = 100U,
  DAL_TICK_FREQ_100HZ        = 10U,
  DAL_TICK_FREQ_1KHZ         = 1U,
  DAL_TICK_FREQ_DEFAULT      = DAL_TICK_FREQ_1KHZ
} DAL_TickFreqTypeDef;
/**
  * @}
  */

/**
  * @}
  */
   
/* Exported macro ------------------------------------------------------------*/
/** @defgroup DAL_Exported_Macros DAL Exported Macros
  * @{
  */

/** @brief  Freeze/Unfreeze Peripherals in Debug mode 
  */
#define __DAL_DBGMCU_FREEZE_TMR2()           (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR2_STS))
#define __DAL_DBGMCU_FREEZE_TMR3()           (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR3_STS))
#define __DAL_DBGMCU_FREEZE_TMR4()           (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR4_STS))
#define __DAL_DBGMCU_FREEZE_TMR5()           (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR5_STS))
#define __DAL_DBGMCU_FREEZE_TMR6()           (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR6_STS))
#define __DAL_DBGMCU_FREEZE_TMR7()           (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR7_STS))
#define __DAL_DBGMCU_FREEZE_TMR12()          (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR12_STS))
#define __DAL_DBGMCU_FREEZE_TMR13()          (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR13_STS))
#define __DAL_DBGMCU_FREEZE_TMR14()          (DBGMCU->APB1F |= (DBGMCU_APB1F_TMR14_STS))
#define __DAL_DBGMCU_FREEZE_RTC()            (DBGMCU->APB1F |= (DBGMCU_APB1F_RTC_STS))
#define __DAL_DBGMCU_FREEZE_WWDT()           (DBGMCU->APB1F |= (DBGMCU_APB1F_WWDT_STS))
#define __DAL_DBGMCU_FREEZE_IWDT()           (DBGMCU->APB1F |= (DBGMCU_APB1F_IWDT_STS))
#define __DAL_DBGMCU_FREEZE_I2C1_TIMEOUT()   (DBGMCU->APB1F |= (DBGMCU_APB1F_I2C1_SMBUS_TIMEOUT_STS))
#define __DAL_DBGMCU_FREEZE_I2C2_TIMEOUT()   (DBGMCU->APB1F |= (DBGMCU_APB1F_I2C2_SMBUS_TIMEOUT_STS))
#define __DAL_DBGMCU_FREEZE_I2C3_TIMEOUT()   (DBGMCU->APB1F |= (DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS))
#define __DAL_DBGMCU_FREEZE_CAN1()           (DBGMCU->APB1F |= (DBGMCU_APB1F_CAN1_STS))
#define __DAL_DBGMCU_FREEZE_CAN2()           (DBGMCU->APB1F |= (DBGMCU_APB1F_CAN2_STS))
#define __DAL_DBGMCU_FREEZE_TMR1()           (DBGMCU->APB2F |= (DBGMCU_APB2F_TMR1_STS))
#define __DAL_DBGMCU_FREEZE_TMR8()           (DBGMCU->APB2F |= (DBGMCU_APB2F_TMR8_STS))
#define __DAL_DBGMCU_FREEZE_TMR9()           (DBGMCU->APB2F |= (DBGMCU_APB2F_TMR9_STS))
#define __DAL_DBGMCU_FREEZE_TMR10()          (DBGMCU->APB2F |= (DBGMCU_APB2F_TMR10_STS))
#define __DAL_DBGMCU_FREEZE_TMR11()          (DBGMCU->APB2F |= (DBGMCU_APB2F_TMR11_STS))

#define __DAL_DBGMCU_UNFREEZE_TMR2()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR2_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR3()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR3_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR4()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR4_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR5()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR5_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR6()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR6_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR7()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR7_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR12()          (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR12_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR13()          (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR13_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR14()          (DBGMCU->APB1F &= ~(DBGMCU_APB1F_TMR14_STS))
#define __DAL_DBGMCU_UNFREEZE_RTC()            (DBGMCU->APB1F &= ~(DBGMCU_APB1F_RTC_STS))
#define __DAL_DBGMCU_UNFREEZE_WWDT()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_WWDT_STS))
#define __DAL_DBGMCU_UNFREEZE_IWDT()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_IWDT_STS))
#define __DAL_DBGMCU_UNFREEZE_I2C1_TIMEOUT()   (DBGMCU->APB1F &= ~(DBGMCU_APB1F_I2C1_SMBUS_TIMEOUT_STS))
#define __DAL_DBGMCU_UNFREEZE_I2C2_TIMEOUT()   (DBGMCU->APB1F &= ~(DBGMCU_APB1F_I2C2_SMBUS_TIMEOUT_STS))
#define __DAL_DBGMCU_UNFREEZE_I2C3_TIMEOUT()   (DBGMCU->APB1F &= ~(DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS))
#define __DAL_DBGMCU_UNFREEZE_CAN1()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_CAN1_STS))
#define __DAL_DBGMCU_UNFREEZE_CAN2()           (DBGMCU->APB1F &= ~(DBGMCU_APB1F_CAN2_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR1()           (DBGMCU->APB2F &= ~(DBGMCU_APB2F_TMR1_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR8()           (DBGMCU->APB2F &= ~(DBGMCU_APB2F_TMR8_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR9()           (DBGMCU->APB2F &= ~(DBGMCU_APB2F_TMR9_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR10()          (DBGMCU->APB2F &= ~(DBGMCU_APB2F_TMR10_STS))
#define __DAL_DBGMCU_UNFREEZE_TMR11()          (DBGMCU->APB2F &= ~(DBGMCU_APB2F_TMR11_STS))

/** @brief  Main Flash memory mapped at 0x00000000
  */
#define __DAL_SYSCFG_REMAPMEMORY_FLASH()             (SYSCFG->MMSEL &= ~(SYSCFG_MMSEL_MMSEL))

/** @brief  System Flash memory mapped at 0x00000000
  */
#define __DAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH()       do {SYSCFG->MMSEL &= ~(SYSCFG_MMSEL_MMSEL);\
                                                         SYSCFG->MMSEL |= SYSCFG_MMSEL_MMSEL_0;\
                                                        }while(0);

/** @brief  Embedded SRAM mapped at 0x00000000
  */
#define __DAL_SYSCFG_REMAPMEMORY_SRAM()       do {SYSCFG->MMSEL &= ~(SYSCFG_MMSEL_MMSEL);\
                                                  SYSCFG->MMSEL |= (SYSCFG_MMSEL_MMSEL_0 | SYSCFG_MMSEL_MMSEL_1);\
                                                 }while(0);

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx) || defined(APM32F411xx)
/** @brief  SMC Bank1 (NOR/PSRAM 1 and 2) mapped at 0x00000000
  */
#define __DAL_SYSCFG_REMAPMEMORY_SMC()        do {SYSCFG->MMSEL &= ~(SYSCFG_MMSEL_MMSEL);\
                                                  SYSCFG->MMSEL |= (SYSCFG_MMSEL_MMSEL_1);\
                                                 }while(0);
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

/**
  * @}
  */

/** @defgroup DAL_Private_Macros DAL Private Macros
  * @{
  */
#define IS_TICKFREQ(FREQ) (((FREQ) == DAL_TICK_FREQ_10HZ)  || \
                           ((FREQ) == DAL_TICK_FREQ_100HZ) || \
                           ((FREQ) == DAL_TICK_FREQ_1KHZ))
/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/

/** @addtogroup DAL_Exported_Variables
  * @{
  */
extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern DAL_TickFreqTypeDef uwTickFreq;
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup DAL_Exported_Functions
  * @{
  */
/** @addtogroup DAL_Exported_Functions_Group1
  * @{
  */
/* Initialization and Configuration functions  ******************************/
DAL_StatusTypeDef DAL_Init(void);
DAL_StatusTypeDef DAL_DeInit(void);
void DAL_MspInit(void);
void DAL_MspDeInit(void);
DAL_StatusTypeDef DAL_InitTick (uint32_t TickPriority);
/**
  * @}
  */

/** @addtogroup DAL_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  ************************************************/
void DAL_IncTick(void);
void DAL_Delay(uint32_t Delay);
uint32_t DAL_GetTick(void);
uint32_t DAL_GetTickPrio(void);
DAL_StatusTypeDef DAL_SetTickFreq(DAL_TickFreqTypeDef Freq);
DAL_TickFreqTypeDef DAL_GetTickFreq(void);
void DAL_SuspendTick(void);
void DAL_ResumeTick(void);
uint32_t DAL_GetHalVersion(void);
uint32_t DAL_GetREVID(void);
uint32_t DAL_GetDEVID(void);
void DAL_DBGMCU_EnableDBGSleepMode(void);
void DAL_DBGMCU_DisableDBGSleepMode(void);
void DAL_DBGMCU_EnableDBGStopMode(void);
void DAL_DBGMCU_DisableDBGStopMode(void);
void DAL_DBGMCU_EnableDBGStandbyMode(void);
void DAL_DBGMCU_DisableDBGStandbyMode(void);
void DAL_EnableCompensationCell(void);
void DAL_DisableCompensationCell(void);
uint32_t DAL_GetUIDw0(void);
uint32_t DAL_GetUIDw1(void);
uint32_t DAL_GetUIDw2(void);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup DAL_Private_Variables DAL Private Variables
  * @{
  */
/**
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/** @defgroup DAL_Private_Constants DAL Private Constants
  * @{
  */
/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @}
  */

/**
  * @}
  */ 
  
#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_H */


