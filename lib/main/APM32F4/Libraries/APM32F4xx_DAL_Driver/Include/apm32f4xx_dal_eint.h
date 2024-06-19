/**
  *
  * @file    apm32f4xx_dal_eint.h
  * @brief   Header file of EINT DAL module.
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
  * Copyright (c) 2018 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.Clause
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32f4xx_DAL_EINT_H
#define APM32f4xx_DAL_EINT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup EINT EINT
  * @brief EINT DAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup EINT_Exported_Types EINT Exported Types
  * @{
  */
typedef enum
{
  DAL_EINT_COMMON_CB_ID          = 0x00U
} EINT_CallbackIDTypeDef;

/**
  * @brief  EINT Handle structure definition
  */
typedef struct
{
  uint32_t Line;                    /*!<  Eint line number */
  void (* PendingCallback)(void);   /*!<  Eint pending callback */
} EINT_HandleTypeDef;

/**
  * @brief  EINT Configuration structure definition
  */
typedef struct
{
  uint32_t Line;      /*!< The Eint line to be configured. This parameter
                           can be a value of @ref EINT_Line */
  uint32_t Mode;      /*!< The Exit Mode to be configured for a core.
                           This parameter can be a combination of @ref EINT_Mode */
  uint32_t Trigger;   /*!< The Eint Trigger to be configured. This parameter
                           can be a value of @ref EINT_Trigger */
  uint32_t GPIOSel;   /*!< The Eint GPIO multiplexer selection to be configured.
                           This parameter is only possible for line 0 to 15. It
                           can be a value of @ref EINT_GPIOSel */
} EINT_ConfigTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup EINT_Exported_Constants EINT Exported Constants
  * @{
  */

/** @defgroup EINT_Line  EINT Line
  * @{
  */
#define EINT_LINE_0                        (EINT_GPIO       | 0x00u)    /*!< External interrupt line 0 */
#define EINT_LINE_1                        (EINT_GPIO       | 0x01u)    /*!< External interrupt line 1 */
#define EINT_LINE_2                        (EINT_GPIO       | 0x02u)    /*!< External interrupt line 2 */
#define EINT_LINE_3                        (EINT_GPIO       | 0x03u)    /*!< External interrupt line 3 */
#define EINT_LINE_4                        (EINT_GPIO       | 0x04u)    /*!< External interrupt line 4 */
#define EINT_LINE_5                        (EINT_GPIO       | 0x05u)    /*!< External interrupt line 5 */
#define EINT_LINE_6                        (EINT_GPIO       | 0x06u)    /*!< External interrupt line 6 */
#define EINT_LINE_7                        (EINT_GPIO       | 0x07u)    /*!< External interrupt line 7 */
#define EINT_LINE_8                        (EINT_GPIO       | 0x08u)    /*!< External interrupt line 8 */
#define EINT_LINE_9                        (EINT_GPIO       | 0x09u)    /*!< External interrupt line 9 */
#define EINT_LINE_10                       (EINT_GPIO       | 0x0Au)    /*!< External interrupt line 10 */
#define EINT_LINE_11                       (EINT_GPIO       | 0x0Bu)    /*!< External interrupt line 11 */
#define EINT_LINE_12                       (EINT_GPIO       | 0x0Cu)    /*!< External interrupt line 12 */
#define EINT_LINE_13                       (EINT_GPIO       | 0x0Du)    /*!< External interrupt line 13 */
#define EINT_LINE_14                       (EINT_GPIO       | 0x0Eu)    /*!< External interrupt line 14 */
#define EINT_LINE_15                       (EINT_GPIO       | 0x0Fu)    /*!< External interrupt line 15 */
#define EINT_LINE_16                       (EINT_CONFIG     | 0x10u)    /*!< External interrupt line 16 Connected to the PVD Output */
#define EINT_LINE_17                       (EINT_CONFIG     | 0x11u)    /*!< External interrupt line 17 Connected to the RTC Alarm event */
#if defined(EINT_IMASK_IM18)
#define EINT_LINE_18                       (EINT_CONFIG     | 0x12u)    /*!< External interrupt line 18 Connected to the USB OTG FS Wakeup from suspend event */
#else
#define EINT_LINE_18                       (EINT_RESERVED   | 0x12u)    /*!< No interrupt supported in this line */
#endif /* EINT_IMASK_IM18 */
#if defined(EINT_IMASK_IM19)
#define EINT_LINE_19                       (EINT_CONFIG     | 0x13u)    /*!< External interrupt line 19 Connected to the Ethernet Wakeup event */
#else
#define EINT_LINE_19                       (EINT_RESERVED   | 0x13u)    /*!< No interrupt supported in this line */
#endif /* EINT_IMASK_IM19 */
#if defined(EINT_IMASK_IM20)
#define EINT_LINE_20                       (EINT_CONFIG     | 0x14u)    /*!< External interrupt line 20 Connected to the USB OTG HS (configured in FS) Wakeup event  */
#else
#define EINT_LINE_20                       (EINT_RESERVED   | 0x14u)    /*!< No interrupt supported in this line */
#endif /* EINT_IMASK_IM20 */
#define EINT_LINE_21                       (EINT_CONFIG     | 0x15u)    /*!< External interrupt line 21 Connected to the RTC Tamper and Time Stamp events */
#define EINT_LINE_22                       (EINT_CONFIG     | 0x16u)    /*!< External interrupt line 22 Connected to the RTC Wakeup event */
#if defined(EINT_IMASK_IM23)
#define EINT_LINE_23                       (EINT_CONFIG     | 0x17u)    /*!< External interrupt line 23 Connected to the LPTIM1 asynchronous event */
#endif /* EINT_IMASK_IM23 */

/**
  * @}
  */

/** @defgroup EINT_Mode  EINT Mode
  * @{
  */
#define EINT_MODE_NONE                      0x00000000u
#define EINT_MODE_INTERRUPT                 0x00000001u
#define EINT_MODE_EVENT                     0x00000002u
/**
  * @}
  */

/** @defgroup EINT_Trigger  EINT Trigger
  * @{
  */

#define EINT_TRIGGER_NONE                   0x00000000u
#define EINT_TRIGGER_RISING                 0x00000001u
#define EINT_TRIGGER_FALLING                0x00000002u
#define EINT_TRIGGER_RISING_FALLING         (EINT_TRIGGER_RISING | EINT_TRIGGER_FALLING)
/**
  * @}
  */

/** @defgroup EINT_GPIOSel  EINT GPIOSel
  * @brief
  * @{
  */
#define EINT_GPIOA                          0x00000000u
#define EINT_GPIOB                          0x00000001u
#define EINT_GPIOC                          0x00000002u
#if defined (GPIOD)
#define EINT_GPIOD                          0x00000003u
#endif /* GPIOD */
#if defined (GPIOE)
#define EINT_GPIOE                          0x00000004u
#endif /* GPIOE */
#if defined (GPIOF)
#define EINT_GPIOF                          0x00000005u
#endif /* GPIOF */
#if defined (GPIOG)
#define EINT_GPIOG                          0x00000006u
#endif /* GPIOG */
#if defined (GPIOH)
#define EINT_GPIOH                          0x00000007u
#endif /* GPIOH */
#if defined (GPIOI)
#define EINT_GPIOI                          0x00000008u
#endif /* GPIOI */
#if defined (GPIOJ)
#define EINT_GPIOJ                          0x00000009u
#endif /* GPIOJ */
#if defined (GPIOK)
#define EINT_GPIOK                          0x0000000Au
#endif /* GPIOK */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup EINT_Exported_Macros EINT Exported Macros
  * @{
  */

/**
  * @}
  */

/* Private constants --------------------------------------------------------*/
/** @defgroup EINT_Private_Constants EINT Private Constants
  * @{
  */
/**
  * @brief  EINT Line property definition
  */
#define EINT_IPENDOPERTY_SHIFT              24u
#define EINT_CONFIG                         (0x02uL << EINT_IPENDOPERTY_SHIFT)
#define EINT_GPIO                           ((0x04uL << EINT_IPENDOPERTY_SHIFT) | EINT_CONFIG)
#define EINT_RESERVED                       (0x08uL << EINT_IPENDOPERTY_SHIFT)
#define EINT_IPENDOPERTY_MASK               (EINT_CONFIG | EINT_GPIO)

/**
  * @brief  EINT bit usage
  */
#define EINT_PIN_MASK                       0x0000001Fu

/**
  * @brief  EINT Mask for interrupt & event mode
  */
#define EINT_MODE_MASK                      (EINT_MODE_EVENT | EINT_MODE_INTERRUPT)

/**
  * @brief  EINT Mask for trigger possibilities
  */
#define EINT_TRIGGER_MASK                   (EINT_TRIGGER_RISING | EINT_TRIGGER_FALLING)

/**
  * @brief  EINT Line number
  */
#if defined(EINT_IMASK_IM23)
#define EINT_LINE_NB                        24UL
#else
#define EINT_LINE_NB                        23UL
#endif /* EINT_IMASK_IM23 */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup EINT_Private_Macros EINT Private Macros
  * @{
  */
#define IS_EINT_LINE(__EINT_LINE__)          ((((__EINT_LINE__) & ~(EINT_IPENDOPERTY_MASK | EINT_PIN_MASK)) == 0x00u) && \
                                             ((((__EINT_LINE__) & EINT_IPENDOPERTY_MASK) == EINT_CONFIG)              || \
                                              (((__EINT_LINE__) & EINT_IPENDOPERTY_MASK) == EINT_GPIO))               && \
                                              (((__EINT_LINE__) & EINT_PIN_MASK) < EINT_LINE_NB))

#define IS_EINT_MODE(__EINT_LINE__)          ((((__EINT_LINE__) & EINT_MODE_MASK) != 0x00u) && \
                                              (((__EINT_LINE__) & ~EINT_MODE_MASK) == 0x00u))

#define IS_EINT_TRIGGER(__EINT_LINE__)       (((__EINT_LINE__)  & ~EINT_TRIGGER_MASK) == 0x00u)

#define IS_EINT_PENDING_EDGE(__EINT_LINE__)  ((__EINT_LINE__) == EINT_TRIGGER_RISING_FALLING)

#define IS_EINT_CONFIG_LINE(__EINT_LINE__)   (((__EINT_LINE__) & EINT_CONFIG) != 0x00u)

#if !defined (GPIOD)
#define IS_EINT_GPIO_PORT(__PORT__)     (((__PORT__) == EINT_GPIOA) || \
                                         ((__PORT__) == EINT_GPIOB) || \
                                         ((__PORT__) == EINT_GPIOC) || \
                                         ((__PORT__) == EINT_GPIOH))
#elif !defined (GPIOE)
#define IS_EINT_GPIO_PORT(__PORT__)     (((__PORT__) == EINT_GPIOA) || \
                                         ((__PORT__) == EINT_GPIOB) || \
                                         ((__PORT__) == EINT_GPIOC) || \
                                         ((__PORT__) == EINT_GPIOD) || \
                                         ((__PORT__) == EINT_GPIOH))
#elif !defined (GPIOF)
#define IS_EINT_GPIO_PORT(__PORT__)     (((__PORT__) == EINT_GPIOA) || \
                                         ((__PORT__) == EINT_GPIOB) || \
                                         ((__PORT__) == EINT_GPIOC) || \
                                         ((__PORT__) == EINT_GPIOD) || \
                                         ((__PORT__) == EINT_GPIOE) || \
                                         ((__PORT__) == EINT_GPIOH))
#elif !defined (GPIOI)
#define IS_EINT_GPIO_PORT(__PORT__)     (((__PORT__) == EINT_GPIOA) || \
                                         ((__PORT__) == EINT_GPIOB) || \
                                         ((__PORT__) == EINT_GPIOC) || \
                                         ((__PORT__) == EINT_GPIOD) || \
                                         ((__PORT__) == EINT_GPIOE) || \
                                         ((__PORT__) == EINT_GPIOF) || \
                                         ((__PORT__) == EINT_GPIOG) || \
                                         ((__PORT__) == EINT_GPIOH))
#elif !defined (GPIOJ)
#define IS_EINT_GPIO_PORT(__PORT__)     (((__PORT__) == EINT_GPIOA) || \
                                         ((__PORT__) == EINT_GPIOB) || \
                                         ((__PORT__) == EINT_GPIOC) || \
                                         ((__PORT__) == EINT_GPIOD) || \
                                         ((__PORT__) == EINT_GPIOE) || \
                                         ((__PORT__) == EINT_GPIOF) || \
                                         ((__PORT__) == EINT_GPIOG) || \
                                         ((__PORT__) == EINT_GPIOH) || \
                                         ((__PORT__) == EINT_GPIOI))
#else
#define IS_EINT_GPIO_PORT(__PORT__)     (((__PORT__) == EINT_GPIOA) || \
                                         ((__PORT__) == EINT_GPIOB) || \
                                         ((__PORT__) == EINT_GPIOC) || \
                                         ((__PORT__) == EINT_GPIOD) || \
                                         ((__PORT__) == EINT_GPIOE) || \
                                         ((__PORT__) == EINT_GPIOF) || \
                                         ((__PORT__) == EINT_GPIOG) || \
                                         ((__PORT__) == EINT_GPIOH) || \
                                         ((__PORT__) == EINT_GPIOI) || \
                                         ((__PORT__) == EINT_GPIOJ) || \
                                         ((__PORT__) == EINT_GPIOK))
#endif /* GPIOD */

#define IS_EINT_GPIO_PIN(__PIN__)       ((__PIN__) < 16U)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup EINT_Exported_Functions EINT Exported Functions
  * @brief    EINT Exported Functions
  * @{
  */

/** @defgroup EINT_Exported_Functions_Group1 Configuration functions
  * @brief    Configuration functions
  * @{
  */
/* Configuration functions ****************************************************/
DAL_StatusTypeDef DAL_EINT_SetConfigLine(EINT_HandleTypeDef *heint, EINT_ConfigTypeDef *pEintConfig);
DAL_StatusTypeDef DAL_EINT_GetConfigLine(EINT_HandleTypeDef *heint, EINT_ConfigTypeDef *pEintConfig);
DAL_StatusTypeDef DAL_EINT_ClearConfigLine(EINT_HandleTypeDef *heint);
DAL_StatusTypeDef DAL_EINT_RegisterCallback(EINT_HandleTypeDef *heint, EINT_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void));
DAL_StatusTypeDef DAL_EINT_GetHandle(EINT_HandleTypeDef *heint, uint32_t EintLine);
/**
  * @}
  */

/** @defgroup EINT_Exported_Functions_Group2 IO operation functions
  * @brief    IO operation functions
  * @{
  */
/* IO operation functions *****************************************************/
void DAL_EINT_IRQHandler(EINT_HandleTypeDef *heint);
uint32_t DAL_EINT_GetPending(EINT_HandleTypeDef *heint, uint32_t Edge);
void DAL_EINT_ClearPending(EINT_HandleTypeDef *heint, uint32_t Edge);
void DAL_EINT_GenerateSWI(EINT_HandleTypeDef *heint);

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

#endif /* APM32f4xx_DAL_EINT_H */

