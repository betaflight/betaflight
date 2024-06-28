/**
  *
  * @file    apm32f4xx_dal_pmu.h
  * @brief   Header file of PMU DAL module.
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
#ifndef APM32F4xx_DAL_PMU_H
#define APM32F4xx_DAL_PMU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup PMU
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

/** @defgroup PMU_Exported_Types PMU Exported Types
  * @{
  */
   
/**
  * @brief  PMU PVD configuration structure definition
  */
typedef struct
{
  uint32_t PVDLevel;   /*!< PVDLevel: Specifies the PVD detection level.
                            This parameter can be a value of @ref PMU_PVD_detection_level */

  uint32_t Mode;      /*!< Mode: Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref PMU_PVD_Mode */
}PMU_PVDTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PMU_Exported_Constants PMU Exported Constants
  * @{
  */
  
/** @defgroup PMU_WakeUp_Pins PMU WakeUp Pins
  * @{
  */
#define PMU_WAKEUP_PIN1                 0x00000100U
/**
  * @}
  */

/** @defgroup PMU_PVD_detection_level PMU PVD detection level
  * @{
  */ 
#define PMU_PVDLEVEL_0                  PMU_CTRL_PLSEL_LEV0
#define PMU_PVDLEVEL_1                  PMU_CTRL_PLSEL_LEV1
#define PMU_PVDLEVEL_2                  PMU_CTRL_PLSEL_LEV2
#define PMU_PVDLEVEL_3                  PMU_CTRL_PLSEL_LEV3
#define PMU_PVDLEVEL_4                  PMU_CTRL_PLSEL_LEV4
#define PMU_PVDLEVEL_5                  PMU_CTRL_PLSEL_LEV5
#define PMU_PVDLEVEL_6                  PMU_CTRL_PLSEL_LEV6
#define PMU_PVDLEVEL_7                  PMU_CTRL_PLSEL_LEV7/* External input analog voltage 
                                                          (Compare internally to VREFINT) */
/**
  * @}
  */   
 
/** @defgroup PMU_PVD_Mode PMU PVD Mode
  * @{
  */
#define PMU_PVD_MODE_NORMAL                 0x00000000U   /*!< basic mode is used */
#define PMU_PVD_MODE_IT_RISING              0x00010001U   /*!< External Interrupt Mode with Rising edge trigger detection */
#define PMU_PVD_MODE_IT_FALLING             0x00010002U   /*!< External Interrupt Mode with Falling edge trigger detection */
#define PMU_PVD_MODE_IT_RISING_FALLING      0x00010003U   /*!< External Interrupt Mode with Rising/Falling edge trigger detection */
#define PMU_PVD_MODE_EVENT_RISING           0x00020001U   /*!< Event Mode with Rising edge trigger detection */
#define PMU_PVD_MODE_EVENT_FALLING          0x00020002U   /*!< Event Mode with Falling edge trigger detection */
#define PMU_PVD_MODE_EVENT_RISING_FALLING   0x00020003U   /*!< Event Mode with Rising/Falling edge trigger detection */
/**
  * @}
  */


/** @defgroup PMU_Regulator_state_in_STOP_mode PMU Regulator state in SLEEP/STOP mode
  * @{
  */
#define PMU_MAINREGULATOR_ON                        0x00000000U
#define PMU_LOWPOWERREGULATOR_ON                    PMU_CTRL_LPDSCFG
/**
  * @}
  */
    
/** @defgroup PMU_SLEEP_mode_entry PMU SLEEP mode entry
  * @{
  */
#define PMU_SLEEPENTRY_WFI              ((uint8_t)0x01)
#define PMU_SLEEPENTRY_WFE              ((uint8_t)0x02)
/**
  * @}
  */

/** @defgroup PMU_STOP_mode_entry PMU STOP mode entry
  * @{
  */
#define PMU_STOPENTRY_WFI               ((uint8_t)0x01)
#define PMU_STOPENTRY_WFE               ((uint8_t)0x02)
/**
  * @}
  */

/** @defgroup PMU_Flag PMU Flag
  * @{
  */
#define PMU_FLAG_WU                     PMU_CSTS_WUEFLG
#define PMU_FLAG_SB                     PMU_CSTS_SBFLG
#define PMU_FLAG_PVDO                   PMU_CSTS_PVDOFLG
#define PMU_FLAG_BRR                    PMU_CSTS_BKPRFLG
#define PMU_FLAG_VOSRDY                 PMU_CSTS_VOSRFLG
/**
  * @}
  */

/**
  * @}
  */ 
  
/* Exported macro ------------------------------------------------------------*/
/** @defgroup PMU_Exported_Macro PMU Exported Macro
  * @{
  */

/** @brief  Check PMU flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *           This parameter can be one of the following values:
  *            @arg PMU_FLAG_WU: Wake Up flag. This flag indicates that a wakeup event 
  *                  was received from the WKUP pin or from the RTC alarm (Alarm A 
  *                  or Alarm B), RTC Tamper event, RTC TimeStamp event or RTC Wakeup.
  *                  An additional wakeup event is detected if the WKUP pin is enabled 
  *                  (by setting the EWUP bit) when the WKUP pin level is already high.  
  *            @arg PMU_FLAG_SB: StandBy flag. This flag indicates that the system was
  *                  resumed from StandBy mode.    
  *            @arg PMU_FLAG_PVDO: PVD Output. This flag is valid only if PVD is enabled 
  *                  by the DAL_PMU_EnablePVD() function. The PVD is stopped by Standby mode 
  *                  For this reason, this bit is equal to 0 after Standby or reset
  *                  until the PVDE bit is set.
  *            @arg PMU_FLAG_BRR: Backup regulator ready flag. This bit is not reset 
  *                  when the device wakes up from Standby mode or by a system reset 
  *                  or power reset.  
  *            @arg PMU_FLAG_VOSRDY: This flag indicates that the Regulator voltage 
  *                 scaling output selection is ready.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_PMU_GET_FLAG(__FLAG__) ((PMU->CSTS & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the PMU's pending flags.
  * @param  __FLAG__ specifies the flag to clear.
  *          This parameter can be one of the following values:
  *            @arg PMU_FLAG_WU: Wake Up flag
  *            @arg PMU_FLAG_SB: StandBy flag
  */
#define __DAL_PMU_CLEAR_FLAG(__FLAG__) (PMU->CTRL |=  (__FLAG__) << 2U)

/**
  * @brief Enable the PVD Exti Line 16.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_ENABLE_IT()   (EINT->IMASK |= (PMU_EINT_LINE_PVD))

/**
  * @brief Disable the PVD EINT Line 16.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_DISABLE_IT()  (EINT->IMASK &= ~(PMU_EINT_LINE_PVD))

/**
  * @brief Enable event on PVD Exti Line 16.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_ENABLE_EVENT()   (EINT->EMASK |= (PMU_EINT_LINE_PVD))

/**
  * @brief Disable event on PVD Exti Line 16.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_DISABLE_EVENT()  (EINT->EMASK &= ~(PMU_EINT_LINE_PVD))

/**
  * @brief Enable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_ENABLE_RISING_EDGE()   SET_BIT(EINT->RTEN, PMU_EINT_LINE_PVD)

/**
  * @brief Disable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_DISABLE_RISING_EDGE()  CLEAR_BIT(EINT->RTEN, PMU_EINT_LINE_PVD)

/**
  * @brief Enable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_ENABLE_FALLING_EDGE()   SET_BIT(EINT->FTEN, PMU_EINT_LINE_PVD)


/**
  * @brief Disable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_DISABLE_FALLING_EDGE()  CLEAR_BIT(EINT->FTEN, PMU_EINT_LINE_PVD)


/**
  * @brief  PVD EINT line configuration: set rising & falling edge trigger.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_ENABLE_RISING_FALLING_EDGE()   do{__DAL_PMU_PVD_EINT_ENABLE_RISING_EDGE();\
                                                             __DAL_PMU_PVD_EINT_ENABLE_FALLING_EDGE();\
                                                            }while(0U)

/**
  * @brief Disable the PVD Extended Interrupt Rising & Falling Trigger.
  * This parameter can be:
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_DISABLE_RISING_FALLING_EDGE()  do{__DAL_PMU_PVD_EINT_DISABLE_RISING_EDGE();\
                                                             __DAL_PMU_PVD_EINT_DISABLE_FALLING_EDGE();\
                                                            }while(0U) 

/**
  * @brief checks whether the specified PVD Exti interrupt flag is set or not.
  * @retval EINT PVD Line Status.
  */
#define __DAL_PMU_PVD_EINT_GET_FLAG()  (EINT->IPEND & (PMU_EINT_LINE_PVD))

/**
  * @brief Clear the PVD Exti flag.
  * @retval None.
  */
#define __DAL_PMU_PVD_EINT_CLEAR_FLAG()  (EINT->IPEND = (PMU_EINT_LINE_PVD))

/**
  * @brief  Generates a Software interrupt on PVD EINT line.
  * @retval None
  */
#define __DAL_PMU_PVD_EINT_GENERATE_SWIT() (EINT->SWINTE |= (PMU_EINT_LINE_PVD))

/**
  * @}
  */

/* Include PMU DAL Extension module */
#include "apm32f4xx_dal_pmu_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PMU_Exported_Functions PMU Exported Functions
  * @{
  */
  
/** @addtogroup PMU_Exported_Functions_Group1 Initialization and de-initialization functions 
  * @{
  */
/* Initialization and de-initialization functions *****************************/
void DAL_PMU_DeInit(void);
void DAL_PMU_EnableBkUpAccess(void);
void DAL_PMU_DisableBkUpAccess(void);
/**
  * @}
  */

/** @addtogroup PMU_Exported_Functions_Group2 Peripheral Control functions 
  * @{
  */
/* Peripheral Control functions  **********************************************/
/* PVD configuration */
void DAL_PMU_ConfigPVD(PMU_PVDTypeDef *sConfigPVD);
void DAL_PMU_EnablePVD(void);
void DAL_PMU_DisablePVD(void);

/* WakeUp pins configuration */
void DAL_PMU_EnableWakeUpPin(uint32_t WakeUpPinx);
void DAL_PMU_DisableWakeUpPin(uint32_t WakeUpPinx);

/* Low Power modes entry */
void DAL_PMU_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void DAL_PMU_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void DAL_PMU_EnterSTANDBYMode(void);

/* Power PVD IRQ Handler */
void DAL_PMU_PVD_IRQHandler(void);
void DAL_PMU_PVDCallback(void);

/* Cortex System Control functions  *******************************************/
void DAL_PMU_EnableSleepOnExit(void);
void DAL_PMU_DisableSleepOnExit(void);
void DAL_PMU_EnableSEVOnPend(void);
void DAL_PMU_DisableSEVOnPend(void);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup PMU_Private_Constants PMU Private Constants
  * @{
  */

/** @defgroup PMU_PVD_EINT_Line PMU PVD EINT Line
  * @{
  */
#define PMU_EINT_LINE_PVD  ((uint32_t)EINT_IMASK_IMASK16)  /*!< External interrupt line 16 Connected to the PVD EINT Line */
/**
  * @}
  */

/** @defgroup PMU_register_alias_address PMU Register alias address
  * @{
  */
/* ------------- PMU registers bit address in the alias region ---------------*/
#define PMU_OFFSET                  (PMU_BASE - PERIPH_BASE)
#define PMU_CTRL_OFFSET             0x00U
#define PMU_CSTS_OFFSET             0x04U
#define PMU_CTRL_OFFSET_BB          (PMU_OFFSET + PMU_CTRL_OFFSET)
#define PMU_CSTS_OFFSET_BB          (PMU_OFFSET + PMU_CSTS_OFFSET)
/**
  * @}
  */

/** @defgroup PMU_CTRL_register_alias PMU CTRL Register alias address
  * @{
  */
/* --- CTRL Register ---*/
/* Alias word address of DBP bit */
#define DBP_BIT_NUMBER   PMU_CTRL_BPWEN_Pos
#define CTRL_BPWEN_BB    (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (DBP_BIT_NUMBER * 4U))

/* Alias word address of PVDE bit */
#define PVDE_BIT_NUMBER  PMU_CTRL_PVDEN_Pos
#define CTRL_PVDEN_BB    (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (PVDE_BIT_NUMBER * 4U))

/* Alias word address of VOS bit */
#define VOS_BIT_NUMBER  PMU_CTRL_VOSSEL_Pos
#define CTRL_VOSSEL_BB  (uint32_t)(PERIPH_BB_BASE + (PMU_CTRL_OFFSET_BB * 32U) + (VOS_BIT_NUMBER * 4U))
/**
  * @}
  */

/** @defgroup PMU_CSTS_register_alias PMU CSTS Register alias address
  * @{
  */
/* --- CSTS Register ---*/
/* Alias word address of EWUP bit */
#define EWUP_BIT_NUMBER  PMU_CSTS_WKUPCFG_Pos
#define CSTS_WKUPCFG_BB  (PERIPH_BB_BASE + (PMU_CSTS_OFFSET_BB * 32U) + (EWUP_BIT_NUMBER * 4U))
/**
  * @}
  */

/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup PMU_Private_Macros PMU Private Macros
  * @{
  */

/** @defgroup PMU_IS_PMU_Definitions PMU Private macros to check input parameters
  * @{
  */
#define IS_PMU_PVD_LEVEL(LEVEL) (((LEVEL) == PMU_PVDLEVEL_0) || ((LEVEL) == PMU_PVDLEVEL_1)|| \
                                 ((LEVEL) == PMU_PVDLEVEL_2) || ((LEVEL) == PMU_PVDLEVEL_3)|| \
                                 ((LEVEL) == PMU_PVDLEVEL_4) || ((LEVEL) == PMU_PVDLEVEL_5)|| \
                                 ((LEVEL) == PMU_PVDLEVEL_6) || ((LEVEL) == PMU_PVDLEVEL_7))
#define IS_PMU_PVD_MODE(MODE) (((MODE) == PMU_PVD_MODE_IT_RISING)|| ((MODE) == PMU_PVD_MODE_IT_FALLING) || \
                              ((MODE) == PMU_PVD_MODE_IT_RISING_FALLING) || ((MODE) == PMU_PVD_MODE_EVENT_RISING) || \
                              ((MODE) == PMU_PVD_MODE_EVENT_FALLING) || ((MODE) == PMU_PVD_MODE_EVENT_RISING_FALLING) || \
                              ((MODE) == PMU_PVD_MODE_NORMAL))
#define IS_PMU_REGULATOR(REGULATOR) (((REGULATOR) == PMU_MAINREGULATOR_ON) || \
                                     ((REGULATOR) == PMU_LOWPOWERREGULATOR_ON))
#define IS_PMU_SLEEP_ENTRY(ENTRY) (((ENTRY) == PMU_SLEEPENTRY_WFI) || ((ENTRY) == PMU_SLEEPENTRY_WFE))
#define IS_PMU_STOP_ENTRY(ENTRY) (((ENTRY) == PMU_STOPENTRY_WFI) || ((ENTRY) == PMU_STOPENTRY_WFE))
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


#endif /* APM32F4xx_DAL_PMU_H */
