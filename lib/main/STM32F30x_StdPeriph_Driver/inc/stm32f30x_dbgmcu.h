/**
  ******************************************************************************
  * @file    stm32f30x_dbgmcu.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the DBGMCU firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F30x_DBGMCU_H
#define __STM32F30x_DBGMCU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"

/** @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */

/** @addtogroup DBGMCU
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup DBGMCU_Exported_Constants
  * @{
  */ 
#define DBGMCU_SLEEP                          ((uint32_t)0x00000001)
#define DBGMCU_STOP                           ((uint32_t)0x00000002)
#define DBGMCU_STANDBY                        ((uint32_t)0x00000004)
#define IS_DBGMCU_PERIPH(PERIPH) ((((PERIPH) & 0xFFFFFFF8) == 0x00) && ((PERIPH) != 0x00))

#define DBGMCU_TIM2_STOP             ((uint32_t)0x00000001)
#define DBGMCU_TIM3_STOP             ((uint32_t)0x00000002)
#define DBGMCU_TIM4_STOP             ((uint32_t)0x00000004)
#define DBGMCU_TIM6_STOP             ((uint32_t)0x00000010)
#define DBGMCU_TIM7_STOP             ((uint32_t)0x00000020)
#define DBGMCU_RTC_STOP              ((uint32_t)0x00000400)
#define DBGMCU_WWDG_STOP             ((uint32_t)0x00000800)
#define DBGMCU_IWDG_STOP             ((uint32_t)0x00001000)
#define DBGMCU_I2C1_SMBUS_TIMEOUT    ((uint32_t)0x00200000)
#define DBGMCU_I2C2_SMBUS_TIMEOUT    ((uint32_t)0x00400000)
#define DBGMCU_CAN1_STOP             ((uint32_t)0x02000000)

#define IS_DBGMCU_APB1PERIPH(PERIPH) ((((PERIPH) & 0xFD9FE3C8) == 0x00) && ((PERIPH) != 0x00))

#define DBGMCU_TIM1_STOP             ((uint32_t)0x00000001)
#define DBGMCU_TIM8_STOP             ((uint32_t)0x00000002)
#define DBGMCU_TIM15_STOP            ((uint32_t)0x00000004)
#define DBGMCU_TIM16_STOP            ((uint32_t)0x00000008)
#define DBGMCU_TIM17_STOP            ((uint32_t)0x00000010)
#define IS_DBGMCU_APB2PERIPH(PERIPH) ((((PERIPH) & 0xFFFFFFE0) == 0x00) && ((PERIPH) != 0x00))

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 
/* Device and Revision ID management functions ********************************/
uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);

/* Peripherals Configuration functions ****************************************/
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB2PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F30x_DBGMCU_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
