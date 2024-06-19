/**
  *
  * @file    apm32f4xx_dal_iwdt.h
  * @brief   Header file of IWDT DAL module.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_IWDT_H
#define APM32F4xx_DAL_IWDT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup IWDT IWDT
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup IWDT_Exported_Types IWDT Exported Types
  * @{
  */

/**
  * @brief  IWDT Init structure definition
  */
typedef struct
{
  uint32_t Prescaler;  /*!< Select the prescaler of the IWDT.
                            This parameter can be a value of @ref IWDT_Prescaler */

  uint32_t Reload;     /*!< Specifies the IWDT down-counter reload value.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 0x0FFF */

} IWDT_InitTypeDef;

/**
  * @brief  IWDT Handle Structure definition
  */
typedef struct
{
  IWDT_TypeDef                 *Instance;  /*!< Register base address    */

  IWDT_InitTypeDef             Init;       /*!< IWDT required parameters */
} IWDT_HandleTypeDef;


/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup IWDT_Exported_Constants IWDT Exported Constants
  * @{
  */

/** @defgroup IWDT_Prescaler IWDT Prescaler
  * @{
  */
#define IWDT_PSCESCALER_4                0x00000000u                                     /*!< IWDT prescaler set to 4   */
#define IWDT_PSCESCALER_8                IWDT_PSC_PSC_0                                    /*!< IWDT prescaler set to 8   */
#define IWDT_PSCESCALER_16               IWDT_PSC_PSC_1                                    /*!< IWDT prescaler set to 16  */
#define IWDT_PSCESCALER_32               (IWDT_PSC_PSC_1 | IWDT_PSC_PSC_0)                   /*!< IWDT prescaler set to 32  */
#define IWDT_PSCESCALER_64               IWDT_PSC_PSC_2                                    /*!< IWDT prescaler set to 64  */
#define IWDT_PSCESCALER_128              (IWDT_PSC_PSC_2 | IWDT_PSC_PSC_0)                   /*!< IWDT prescaler set to 128 */
#define IWDT_PSCESCALER_256              (IWDT_PSC_PSC_2 | IWDT_PSC_PSC_1)                   /*!< IWDT prescaler set to 256 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup IWDT_Exported_Macros IWDT Exported Macros
  * @{
  */

/**
  * @brief  Enable the IWDT peripheral.
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define __DAL_IWDT_START(__HANDLE__)                WRITE_REG((__HANDLE__)->Instance->KEY, IWDT_KEY_ENABLE)

/**
  * @brief  Reload IWDT counter with value defined in the reload register
  *         (write access to IWDT_PSC and IWDT_CNTRLD registers disabled).
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define __DAL_IWDT_RELOAD_COUNTER(__HANDLE__)       WRITE_REG((__HANDLE__)->Instance->KEY, IWDT_KEY_RELOAD)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup IWDT_Exported_Functions  IWDT Exported Functions
  * @{
  */

/** @defgroup IWDT_Exported_Functions_Group1 Initialization and Start functions
  * @{
  */
/* Initialization/Start functions  ********************************************/
DAL_StatusTypeDef     DAL_IWDT_Init(IWDT_HandleTypeDef *hiwdt);
/**
  * @}
  */

/** @defgroup IWDT_Exported_Functions_Group2 IO operation functions
  * @{
  */
/* I/O operation functions ****************************************************/
DAL_StatusTypeDef     DAL_IWDT_Refresh(IWDT_HandleTypeDef *hiwdt);
/**
  * @}
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup IWDT_Private_Constants IWDT Private Constants
  * @{
  */

/**
  * @brief  IWDT Key Register BitMask
  */
#define IWDT_KEY_RELOAD                 0x0000AAAAu  /*!< IWDT Reload Counter Enable   */
#define IWDT_KEY_ENABLE                 0x0000CCCCu  /*!< IWDT Peripheral Enable       */
#define IWDT_KEY_WRITE_ACCESS_ENABLE    0x00005555u  /*!< IWDT KR Write Access Enable  */
#define IWDT_KEY_WRITE_ACCESS_DISABLE   0x00000000u  /*!< IWDT KR Write Access Disable */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup IWDT_Private_Macros IWDT Private Macros
  * @{
  */

/**
  * @brief  Enable write access to IWDT_PSC and IWDT_CNTRLD registers.
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define IWDT_ENABLE_WRITE_ACCESS(__HANDLE__)  WRITE_REG((__HANDLE__)->Instance->KEY, IWDT_KEY_WRITE_ACCESS_ENABLE)

/**
  * @brief  Disable write access to IWDT_PSC and IWDT_CNTRLD registers.
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define IWDT_DISABLE_WRITE_ACCESS(__HANDLE__) WRITE_REG((__HANDLE__)->Instance->KEY, IWDT_KEY_WRITE_ACCESS_DISABLE)

/**
  * @brief  Check IWDT prescaler value.
  * @param  __PRESCALER__  IWDT prescaler value
  * @retval None
  */
#define IS_IWDT_PSCESCALER(__PRESCALER__)      (((__PRESCALER__) == IWDT_PSCESCALER_4)  || \
                                               ((__PRESCALER__) == IWDT_PSCESCALER_8)  || \
                                               ((__PRESCALER__) == IWDT_PSCESCALER_16) || \
                                               ((__PRESCALER__) == IWDT_PSCESCALER_32) || \
                                               ((__PRESCALER__) == IWDT_PSCESCALER_64) || \
                                               ((__PRESCALER__) == IWDT_PSCESCALER_128)|| \
                                               ((__PRESCALER__) == IWDT_PSCESCALER_256))

/**
  * @brief  Check IWDT reload value.
  * @param  __RELOAD__  IWDT reload value
  * @retval None
  */
#define IS_IWDT_RELOAD(__RELOAD__)            ((__RELOAD__) <= IWDT_CNTRLD_CNTRLD)



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

#endif /* APM32F4xx_DAL_IWDT_H */
