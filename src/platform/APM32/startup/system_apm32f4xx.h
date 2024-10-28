/**
 *
 * @file        system_apm32f4xx.h
 *
 * @brief       CMSIS Cortex-M4 Device System Source File for APM32F4xx devices.       
 *
 * @version     V1.0.0
 *
 * @date        2023-07-31
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 *
 */ 

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup apm32f4xx_system
  * @{
  */  
  
/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef __SYSTEM_APM32F4XX_H
#define __SYSTEM_APM32F4XX_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup APM32F4xx_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup APM32F4xx_System_Exported_types
  * @{
  */
extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

extern const uint8_t  AHBPrescTable[16];    /*!< AHB prescalers table values */
extern const uint8_t  APBPrescTable[8];     /*!< APB prescalers table values */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Exported_Functions
  * @{
  */
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
extern void OverclockRebootIfNecessary(uint32_t overclockLevel);
extern void systemClockSetHSEValue(uint32_t frequency);
extern int SystemSYSCLKSource(void);
extern int SystemPLLSource(void);
extern void DAL_ErrorHandler(void);
extern void DAL_SysClkConfig(void);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_APM32F4XX_H */

/**
  * @}
  */
  
/**
  * @}
  */  
