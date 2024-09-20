/**
  *
  * @file    apm32f4xx_ddl_gpio.h
  * @brief   Header file of GPIO DDL module.
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
#ifndef APM32F4xx_DDL_GPIO_H
#define APM32F4xx_DDL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) || defined (GPIOF) || defined (GPIOG) || defined (GPIOH) || defined (GPIOI) || defined (GPIOJ) || defined (GPIOK)

/** @defgroup GPIO_DDL GPIO
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GPIO_DDL_Private_Macros GPIO Private Macros
  * @{
  */

/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GPIO_DDL_ES_INIT GPIO Exported Init structures
  * @{
  */

/**
  * @brief LL GPIO Init Structure definition
  */
typedef struct
{
  uint32_t Pin;          /*!< Specifies the GPIO pins to be configured.
                              This parameter can be any value of @ref GPIO_DDL_EC_PIN */

  uint32_t Mode;         /*!< Specifies the operating mode for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_MODE.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinMode().*/

  uint32_t Speed;        /*!< Specifies the speed for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_SPEED.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinSpeed().*/

  uint32_t OutputType;   /*!< Specifies the operating output type for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_OUTPUT.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinOutputType().*/

  uint32_t Pull;         /*!< Specifies the operating Pull-up/Pull down for the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_PULL.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetPinPull().*/

  uint32_t Alternate;    /*!< Specifies the Peripheral to be connected to the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_AF.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetAFPin_0_7() and DDL_GPIO_SetAFPin_8_15().*/
} DDL_GPIO_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIO_DDL_Exported_Constants GPIO Exported Constants
  * @{
  */

/** @defgroup GPIO_DDL_EC_PIN PIN
  * @{
  */
#define DDL_GPIO_PIN_0                      GPIO_BSC_BS_0 /*!< Select pin 0 */
#define DDL_GPIO_PIN_1                      GPIO_BSC_BS_1 /*!< Select pin 1 */
#define DDL_GPIO_PIN_2                      GPIO_BSC_BS_2 /*!< Select pin 2 */
#define DDL_GPIO_PIN_3                      GPIO_BSC_BS_3 /*!< Select pin 3 */
#define DDL_GPIO_PIN_4                      GPIO_BSC_BS_4 /*!< Select pin 4 */
#define DDL_GPIO_PIN_5                      GPIO_BSC_BS_5 /*!< Select pin 5 */
#define DDL_GPIO_PIN_6                      GPIO_BSC_BS_6 /*!< Select pin 6 */
#define DDL_GPIO_PIN_7                      GPIO_BSC_BS_7 /*!< Select pin 7 */
#define DDL_GPIO_PIN_8                      GPIO_BSC_BS_8 /*!< Select pin 8 */
#define DDL_GPIO_PIN_9                      GPIO_BSC_BS_9 /*!< Select pin 9 */
#define DDL_GPIO_PIN_10                     GPIO_BSC_BS_10 /*!< Select pin 10 */
#define DDL_GPIO_PIN_11                     GPIO_BSC_BS_11 /*!< Select pin 11 */
#define DDL_GPIO_PIN_12                     GPIO_BSC_BS_12 /*!< Select pin 12 */
#define DDL_GPIO_PIN_13                     GPIO_BSC_BS_13 /*!< Select pin 13 */
#define DDL_GPIO_PIN_14                     GPIO_BSC_BS_14 /*!< Select pin 14 */
#define DDL_GPIO_PIN_15                     GPIO_BSC_BS_15 /*!< Select pin 15 */
#define DDL_GPIO_PIN_ALL                    (GPIO_BSC_BS_0 | GPIO_BSC_BS_1  | GPIO_BSC_BS_2  | \
                                           GPIO_BSC_BS_3  | GPIO_BSC_BS_4  | GPIO_BSC_BS_5  | \
                                           GPIO_BSC_BS_6  | GPIO_BSC_BS_7  | GPIO_BSC_BS_8  | \
                                           GPIO_BSC_BS_9  | GPIO_BSC_BS_10 | GPIO_BSC_BS_11 | \
                                           GPIO_BSC_BS_12 | GPIO_BSC_BS_13 | GPIO_BSC_BS_14 | \
                                           GPIO_BSC_BS_15) /*!< Select all pins */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_MODE Mode
  * @{
  */
#define DDL_GPIO_MODE_INPUT                 (0x00000000U) /*!< Select input mode */
#define DDL_GPIO_MODE_OUTPUT                GPIO_MODE_MODE0_0  /*!< Select output mode */
#define DDL_GPIO_MODE_ALTERNATE             GPIO_MODE_MODE0_1  /*!< Select alternate function mode */
#define DDL_GPIO_MODE_ANALOG                GPIO_MODE_MODE0    /*!< Select analog mode */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_OUTPUT Output Type
  * @{
  */
#define DDL_GPIO_OUTPUT_PUSHPULL            (0x00000000U) /*!< Select push-pull as output type */
#define DDL_GPIO_OUTPUT_OPENDRAIN           GPIO_OMODE_OT_0 /*!< Select open-drain as output type */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_SPEED Output Speed
  * @{
  */
#define DDL_GPIO_SPEED_FREQ_LOW             (0x00000000U) /*!< Select I/O low output speed    */
#define DDL_GPIO_SPEED_FREQ_MEDIUM          GPIO_OSPEEDER_OSPEEDR0_0 /*!< Select I/O medium output speed */
#define DDL_GPIO_SPEED_FREQ_HIGH            GPIO_OSPEEDER_OSPEEDR0_1 /*!< Select I/O fast output speed   */
#define DDL_GPIO_SPEED_FREQ_VERY_HIGH       GPIO_OSPEEDER_OSPEEDR0   /*!< Select I/O high output speed   */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_PULL Pull Up Pull Down
  * @{
  */
#define DDL_GPIO_PULL_NO                    (0x00000000U) /*!< Select I/O no pull */
#define DDL_GPIO_PULL_UP                    GPIO_PUPD_PUPDR0_0 /*!< Select I/O pull up */
#define DDL_GPIO_PULL_DOWN                  GPIO_PUPD_PUPDR0_1 /*!< Select I/O pull down */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_AF Alternate Function
  * @{
  */
#define DDL_GPIO_AF_0                       (0x0000000U) /*!< Select alternate function 0 */
#define DDL_GPIO_AF_1                       (0x0000001U) /*!< Select alternate function 1 */
#define DDL_GPIO_AF_2                       (0x0000002U) /*!< Select alternate function 2 */
#define DDL_GPIO_AF_3                       (0x0000003U) /*!< Select alternate function 3 */
#define DDL_GPIO_AF_4                       (0x0000004U) /*!< Select alternate function 4 */
#define DDL_GPIO_AF_5                       (0x0000005U) /*!< Select alternate function 5 */
#define DDL_GPIO_AF_6                       (0x0000006U) /*!< Select alternate function 6 */
#define DDL_GPIO_AF_7                       (0x0000007U) /*!< Select alternate function 7 */
#define DDL_GPIO_AF_8                       (0x0000008U) /*!< Select alternate function 8 */
#define DDL_GPIO_AF_9                       (0x0000009U) /*!< Select alternate function 9 */
#define DDL_GPIO_AF_10                      (0x000000AU) /*!< Select alternate function 10 */
#define DDL_GPIO_AF_11                      (0x000000BU) /*!< Select alternate function 11 */
#define DDL_GPIO_AF_12                      (0x000000CU) /*!< Select alternate function 12 */
#define DDL_GPIO_AF_13                      (0x000000DU) /*!< Select alternate function 13 */
#define DDL_GPIO_AF_14                      (0x000000EU) /*!< Select alternate function 14 */
#define DDL_GPIO_AF_15                      (0x000000FU) /*!< Select alternate function 15 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIO_DDL_Exported_Macros GPIO Exported Macros
  * @{
  */

/** @defgroup GPIO_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in GPIO register
  * @param  __INSTANCE__ GPIO Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_GPIO_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in GPIO register
  * @param  __INSTANCE__ GPIO Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_GPIO_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup GPIO_DDL_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @defgroup GPIO_DDL_EF_Port_Configuration Port Configuration
  * @{
  */

/**
  * @brief  Configure gpio mode for a dedicated pin on dedicated port.
  * @note   I/O mode can be Input mode, General purpose output, Alternate function mode or Analog.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_MODE_INPUT
  *         @arg @ref DDL_GPIO_MODE_OUTPUT
  *         @arg @ref DDL_GPIO_MODE_ALTERNATE
  *         @arg @ref DDL_GPIO_MODE_ANALOG
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
  MODIFY_REG(GPIOx->MODE, (GPIO_MODE_MODE0 << (POSITION_VAL(Pin) * 2U)), (Mode << (POSITION_VAL(Pin) * 2U)));
}

/**
  * @brief  Return gpio mode for a dedicated pin on dedicated port.
  * @note   I/O mode can be Input mode, General purpose output, Alternate function mode or Analog.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_MODE_INPUT
  *         @arg @ref DDL_GPIO_MODE_OUTPUT
  *         @arg @ref DDL_GPIO_MODE_ALTERNATE
  *         @arg @ref DDL_GPIO_MODE_ANALOG
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->MODE,
                             (GPIO_MODE_MODE0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
}

/**
  * @brief  Configure gpio output type for several pins on dedicated port.
  * @note   Output type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are Push-pull or Open-drain.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @param  OutputType This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref DDL_GPIO_OUTPUT_OPENDRAIN
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinOutputType(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t OutputType)
{
  MODIFY_REG(GPIOx->OMODE, PinMask, (PinMask * OutputType));
}

/**
  * @brief  Return gpio output type for several pins on dedicated port.
  * @note   Output type as to be set when gpio pin is in output or
  *         alternate modes. Possible type are Push-pull or Open-drain.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_OUTPUT_PUSHPULL
  *         @arg @ref DDL_GPIO_OUTPUT_OPENDRAIN
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinOutputType(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->OMODE, Pin) >> POSITION_VAL(Pin));
}

/**
  * @brief  Configure gpio speed for a dedicated pin on dedicated port.
  * @note   I/O speed can be Low, Medium, Fast or High speed.
  * @note   Warning: only one pin can be passed as parameter.
  * @note   Refer to datasheet for frequency specifications and the power
  *         supply and load conditions for each speed.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Speed This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_SPEED_FREQ_LOW
  *         @arg @ref DDL_GPIO_SPEED_FREQ_MEDIUM
  *         @arg @ref DDL_GPIO_SPEED_FREQ_HIGH
  *         @arg @ref DDL_GPIO_SPEED_FREQ_VERY_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinSpeed(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t  Speed)
{
  MODIFY_REG(GPIOx->OSSEL, (GPIO_OSPEEDER_OSPEEDR0 << (POSITION_VAL(Pin) * 2U)),
             (Speed << (POSITION_VAL(Pin) * 2U)));
}

/**
  * @brief  Return gpio speed for a dedicated pin on dedicated port.
  * @note   I/O speed can be Low, Medium, Fast or High speed.
  * @note   Warning: only one pin can be passed as parameter.
  * @note   Refer to datasheet for frequency specifications and the power
  *         supply and load conditions for each speed.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_SPEED_FREQ_LOW
  *         @arg @ref DDL_GPIO_SPEED_FREQ_MEDIUM
  *         @arg @ref DDL_GPIO_SPEED_FREQ_HIGH
  *         @arg @ref DDL_GPIO_SPEED_FREQ_VERY_HIGH
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinSpeed(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->OSSEL,
                             (GPIO_OSPEEDER_OSPEEDR0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
}

/**
  * @brief  Configure gpio pull-up or pull-down for a dedicated pin on a dedicated port.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Pull This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PULL_NO
  *         @arg @ref DDL_GPIO_PULL_UP
  *         @arg @ref DDL_GPIO_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Pull)
{
  MODIFY_REG(GPIOx->PUPD, (GPIO_PUPD_PUPDR0 << (POSITION_VAL(Pin) * 2U)), (Pull << (POSITION_VAL(Pin) * 2U)));
}

/**
  * @brief  Return gpio pull-up or pull-down for a dedicated pin on a dedicated port
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_PULL_NO
  *         @arg @ref DDL_GPIO_PULL_UP
  *         @arg @ref DDL_GPIO_PULL_DOWN
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->PUPD,
                             (GPIO_PUPD_PUPDR0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
}

/**
  * @brief  Configure gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  * @param  Alternate This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  *         @arg @ref DDL_GPIO_AF_8
  *         @arg @ref DDL_GPIO_AF_9
  *         @arg @ref DDL_GPIO_AF_10
  *         @arg @ref DDL_GPIO_AF_11
  *         @arg @ref DDL_GPIO_AF_12
  *         @arg @ref DDL_GPIO_AF_13
  *         @arg @ref DDL_GPIO_AF_14
  *         @arg @ref DDL_GPIO_AF_15
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  MODIFY_REG(GPIOx->ALF[0], (GPIO_ALFL_ALFSEL0 << (POSITION_VAL(Pin) * 4U)),
             (Alternate << (POSITION_VAL(Pin) * 4U)));
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  *         @arg @ref DDL_GPIO_AF_8
  *         @arg @ref DDL_GPIO_AF_9
  *         @arg @ref DDL_GPIO_AF_10
  *         @arg @ref DDL_GPIO_AF_11
  *         @arg @ref DDL_GPIO_AF_12
  *         @arg @ref DDL_GPIO_AF_13
  *         @arg @ref DDL_GPIO_AF_14
  *         @arg @ref DDL_GPIO_AF_15
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->ALF[0],
                             (GPIO_ALFL_ALFSEL0 << (POSITION_VAL(Pin) * 4U))) >> (POSITION_VAL(Pin) * 4U));
}

/**
  * @brief  Configure gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @note   Warning: only one pin can be passed as parameter.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @param  Alternate This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  *         @arg @ref DDL_GPIO_AF_8
  *         @arg @ref DDL_GPIO_AF_9
  *         @arg @ref DDL_GPIO_AF_10
  *         @arg @ref DDL_GPIO_AF_11
  *         @arg @ref DDL_GPIO_AF_12
  *         @arg @ref DDL_GPIO_AF_13
  *         @arg @ref DDL_GPIO_AF_14
  *         @arg @ref DDL_GPIO_AF_15
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  MODIFY_REG(GPIOx->ALF[1], (GPIO_ALFH_ALFSEL8 << (POSITION_VAL(Pin >> 8U) * 4U)),
             (Alternate << (POSITION_VAL(Pin >> 8U) * 4U)));
}

/**
  * @brief  Return gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
  * @note   Possible values are from AF0 to AF15 depending on target.
  * @param  GPIOx GPIO Port
  * @param  Pin This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_0
  *         @arg @ref DDL_GPIO_AF_1
  *         @arg @ref DDL_GPIO_AF_2
  *         @arg @ref DDL_GPIO_AF_3
  *         @arg @ref DDL_GPIO_AF_4
  *         @arg @ref DDL_GPIO_AF_5
  *         @arg @ref DDL_GPIO_AF_6
  *         @arg @ref DDL_GPIO_AF_7
  *         @arg @ref DDL_GPIO_AF_8
  *         @arg @ref DDL_GPIO_AF_9
  *         @arg @ref DDL_GPIO_AF_10
  *         @arg @ref DDL_GPIO_AF_11
  *         @arg @ref DDL_GPIO_AF_12
  *         @arg @ref DDL_GPIO_AF_13
  *         @arg @ref DDL_GPIO_AF_14
  *         @arg @ref DDL_GPIO_AF_15
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(READ_BIT(GPIOx->ALF[1],
                             (GPIO_ALFH_ALFSEL8 << (POSITION_VAL(Pin >> 8U) * 4U))) >> (POSITION_VAL(Pin >> 8U) * 4U));
}


/**
  * @brief  Lock configuration of several pins for a dedicated port.
  * @note   When the lock sequence has been applied on a port bit, the
  *         value of this port bit can no longer be modified until the
  *         next reset.
  * @note   Each lock bit freezes a specific configuration register
  *         (control and alternate function registers).
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  __IO uint32_t temp;
  WRITE_REG(GPIOx->LOCK, GPIO_LOCK_LOCKKEY | PinMask);
  WRITE_REG(GPIOx->LOCK, PinMask);
  WRITE_REG(GPIOx->LOCK, GPIO_LOCK_LOCKKEY | PinMask);
  temp = READ_REG(GPIOx->LOCK);
  (void) temp;
}

/**
  * @brief  Return 1 if all pins passed as parameter, of a dedicated port, are locked. else Return 0.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_IsPinLocked(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (READ_BIT(GPIOx->LOCK, PinMask) == (PinMask));
}

/**
  * @brief  Return 1 if one of the pin of a dedicated port is locked. else return 0.
  * @param  GPIOx GPIO Port
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_IsAnyPinLocked(GPIO_TypeDef *GPIOx)
{
  return (READ_BIT(GPIOx->LOCK, GPIO_LOCK_LOCKKEY) == (GPIO_LOCK_LOCKKEY));
}

/**
  * @}
  */

/** @defgroup GPIO_DDL_EF_Data_Access Data Access
  * @{
  */

/**
  * @brief  Return full input data register value for a dedicated port.
  * @param  GPIOx GPIO Port
  * @retval Input data register value of port
  */
__STATIC_INLINE uint32_t DDL_GPIO_ReadInputPort(GPIO_TypeDef *GPIOx)
{
  return (uint32_t)(READ_REG(GPIOx->IDATA));
}

/**
  * @brief  Return if input data level for several pins of dedicated port is high or low.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_IsInputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (READ_BIT(GPIOx->IDATA, PinMask) == (PinMask));
}

/**
  * @brief  Write output data register for the port.
  * @param  GPIOx GPIO Port
  * @param  PortValue Level value for each pin of the port
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_WriteOutputPort(GPIO_TypeDef *GPIOx, uint32_t PortValue)
{
  WRITE_REG(GPIOx->ODATA, PortValue);
}

/**
  * @brief  Return full output data register value for a dedicated port.
  * @param  GPIOx GPIO Port
  * @retval Output data register value of port
  */
__STATIC_INLINE uint32_t DDL_GPIO_ReadOutputPort(GPIO_TypeDef *GPIOx)
{
  return (uint32_t)(READ_REG(GPIOx->ODATA));
}

/**
  * @brief  Return if input data level for several pins of dedicated port is high or low.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_IsOutputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (READ_BIT(GPIOx->ODATA, PinMask) == (PinMask));
}

/**
  * @brief  Set several pins to high level on dedicated gpio port.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  WRITE_REG(GPIOx->BSC, PinMask);
}

/**
  * @brief  Set several pins to low level on dedicated gpio port.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_ResetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  WRITE_REG(GPIOx->BSC, (PinMask << 16));
}

/**
  * @brief  Toggle data value for several pin of dedicated port.
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref DDL_GPIO_PIN_0
  *         @arg @ref DDL_GPIO_PIN_1
  *         @arg @ref DDL_GPIO_PIN_2
  *         @arg @ref DDL_GPIO_PIN_3
  *         @arg @ref DDL_GPIO_PIN_4
  *         @arg @ref DDL_GPIO_PIN_5
  *         @arg @ref DDL_GPIO_PIN_6
  *         @arg @ref DDL_GPIO_PIN_7
  *         @arg @ref DDL_GPIO_PIN_8
  *         @arg @ref DDL_GPIO_PIN_9
  *         @arg @ref DDL_GPIO_PIN_10
  *         @arg @ref DDL_GPIO_PIN_11
  *         @arg @ref DDL_GPIO_PIN_12
  *         @arg @ref DDL_GPIO_PIN_13
  *         @arg @ref DDL_GPIO_PIN_14
  *         @arg @ref DDL_GPIO_PIN_15
  *         @arg @ref DDL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  uint32_t odr = READ_REG(GPIOx->ODATA);
  WRITE_REG(GPIOx->BSC, ((odr & PinMask) << 16u) | (~odr & PinMask));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup GPIO_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus DDL_GPIO_DeInit(GPIO_TypeDef *GPIOx);
ErrorStatus DDL_GPIO_Init(GPIO_TypeDef *GPIOx, DDL_GPIO_InitTypeDef *GPIO_InitStruct);
void        DDL_GPIO_StructInit(DDL_GPIO_InitTypeDef *GPIO_InitStruct);

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) || defined (GPIOF) || defined (GPIOG) || defined (GPIOH) || defined (GPIOI) || defined (GPIOJ) || defined (GPIOK) */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_GPIO_H */

