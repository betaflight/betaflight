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
  * The original code has been modified by Geehy Semiconductor.
  * Copyright (c) 2017 STMicroelectronics. Copyright (C) 2023-2025 Geehy Semiconductor.
  * All rights reserved.
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

#if defined (GPIO_ALFL_ALFSEL0)
  uint32_t Alternate;    /*!< Specifies the Peripheral to be connected to the selected pins.
                              This parameter can be a value of @ref GPIO_DDL_EC_AF.

                              GPIO HW configuration can be modified afterwards using unitary function @ref DDL_GPIO_SetAFPin_0_7() and DDL_GPIO_SetAFPin_8_15().*/
#endif /* GPIO_ALFL_ALFSEL0 */
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
#if defined (GPIO_PIN_MASK_POS)
#define DDL_GPIO_PIN_0                    ((GPIO_BSC_BS0  << GPIO_PIN_MASK_POS) | 0x00000001U)  /*!< Select pin 0  */
#define DDL_GPIO_PIN_1                    ((GPIO_BSC_BS1  << GPIO_PIN_MASK_POS) | 0x00000002U)  /*!< Select pin 1  */
#define DDL_GPIO_PIN_2                    ((GPIO_BSC_BS2  << GPIO_PIN_MASK_POS) | 0x00000004U)  /*!< Select pin 2  */
#define DDL_GPIO_PIN_3                    ((GPIO_BSC_BS3  << GPIO_PIN_MASK_POS) | 0x00000008U)  /*!< Select pin 3  */
#define DDL_GPIO_PIN_4                    ((GPIO_BSC_BS4  << GPIO_PIN_MASK_POS) | 0x00000010U)  /*!< Select pin 4  */
#define DDL_GPIO_PIN_5                    ((GPIO_BSC_BS5  << GPIO_PIN_MASK_POS) | 0x00000020U)  /*!< Select pin 5  */
#define DDL_GPIO_PIN_6                    ((GPIO_BSC_BS6  << GPIO_PIN_MASK_POS) | 0x00000040U)  /*!< Select pin 6  */
#define DDL_GPIO_PIN_7                    ((GPIO_BSC_BS7  << GPIO_PIN_MASK_POS) | 0x00000080U)  /*!< Select pin 7  */
#define DDL_GPIO_PIN_8                    ((GPIO_BSC_BS8  << GPIO_PIN_MASK_POS) | 0x04000001U)  /*!< Select pin 8  */
#define DDL_GPIO_PIN_9                    ((GPIO_BSC_BS9  << GPIO_PIN_MASK_POS) | 0x04000002U)  /*!< Select pin 9  */
#define DDL_GPIO_PIN_10                   ((GPIO_BSC_BS10 << GPIO_PIN_MASK_POS) | 0x04000004U)  /*!< Select pin 10 */
#define DDL_GPIO_PIN_11                   ((GPIO_BSC_BS11 << GPIO_PIN_MASK_POS) | 0x04000008U)  /*!< Select pin 11 */
#define DDL_GPIO_PIN_12                   ((GPIO_BSC_BS12 << GPIO_PIN_MASK_POS) | 0x04000010U)  /*!< Select pin 12 */
#define DDL_GPIO_PIN_13                   ((GPIO_BSC_BS13 << GPIO_PIN_MASK_POS) | 0x04000020U)  /*!< Select pin 13 */
#define DDL_GPIO_PIN_14                   ((GPIO_BSC_BS14 << GPIO_PIN_MASK_POS) | 0x04000040U)  /*!< Select pin 14 */
#define DDL_GPIO_PIN_15                   ((GPIO_BSC_BS15 << GPIO_PIN_MASK_POS) | 0x04000080U)  /*!< Select pin 15 */
#define DDL_GPIO_PIN_ALL                  (DDL_GPIO_PIN_0  | DDL_GPIO_PIN_1  | DDL_GPIO_PIN_2  | \
                                           DDL_GPIO_PIN_3  | DDL_GPIO_PIN_4  | DDL_GPIO_PIN_5  | \
                                           DDL_GPIO_PIN_6  | DDL_GPIO_PIN_7  | DDL_GPIO_PIN_8  | \
                                           DDL_GPIO_PIN_9  | DDL_GPIO_PIN_10 | DDL_GPIO_PIN_11 | \
                                           DDL_GPIO_PIN_12 | DDL_GPIO_PIN_13 | DDL_GPIO_PIN_14 | \
                                           DDL_GPIO_PIN_15)   /*!< Select all pins */
#else
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
#endif /* GPIO_PIN_MASK_POS */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_MODE Mode
  * @{
  */
#if defined (GPIO_CFGLOW_MODE) && defined (GPIO_CFGHIG_MODE)
#define DDL_GPIO_MODE_ANALOG                0x00000000U          /*!< Select analog mode */
#define DDL_GPIO_MODE_FLOATING              GPIO_CFGLOW_CFG0_0   /*!< Select floating mode */
#define DDL_GPIO_MODE_INPUT                 GPIO_CFGLOW_CFG0_1   /*!< Select input mode */
#define DDL_GPIO_MODE_OUTPUT                GPIO_CFGLOW_MODE0_0  /*!< Select general purpose output mode */
#define DDL_GPIO_MODE_ALTERNATE             (GPIO_CFGLOW_CFG0_1 | GPIO_CFGLOW_MODE0_0) /*!< Select alternate function mode */
#else
#define DDL_GPIO_MODE_INPUT                 (0x00000000U)      /*!< Select input mode */
#define DDL_GPIO_MODE_OUTPUT                GPIO_MODE_MODE0_0  /*!< Select output mode */
#define DDL_GPIO_MODE_ALTERNATE             GPIO_MODE_MODE0_1  /*!< Select alternate function mode */
#define DDL_GPIO_MODE_ANALOG                GPIO_MODE_MODE0    /*!< Select analog mode */
#endif
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_OUTPUT Output Type
  * @{
  */
#define DDL_GPIO_OUTPUT_PUSHPULL            (0x00000000U)      /*!< Select push-pull as output type */
#if defined (GPIO_OMODE_OMODE0)
#define DDL_GPIO_OUTPUT_OPENDRAIN           GPIO_OMODE_OT_0    /*!< Select open-drain as output type */
#elif defined (GPIO_CFGLOW_MODE)
#define DDL_GPIO_OUTPUT_OPENDRAIN           GPIO_CFGLOW_CFG0_0 /*!< Select open-drain as output type */
#endif /* GPIO_OMODE_OMODE0 */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_SPEED Output Speed
  * @{
  */
#if defined (GPIO_OSSEL_OSSEL0)
#define DDL_GPIO_SPEED_FREQ_LOW             (0x00000000U)            /*!< Select I/O low output speed    */
#define DDL_GPIO_SPEED_FREQ_MEDIUM          GPIO_OSPEEDER_OSPEEDR0_0 /*!< Select I/O medium output speed */
#define DDL_GPIO_SPEED_FREQ_HIGH            GPIO_OSPEEDER_OSPEEDR0_1 /*!< Select I/O fast output speed   */
#define DDL_GPIO_SPEED_FREQ_VERY_HIGH       GPIO_OSPEEDER_OSPEEDR0   /*!< Select I/O high output speed   */
#else
#define DDL_GPIO_MODE_OUTPUT_10MHz          GPIO_CFGLOW_MODE0_0      /*!< Select Output mode, max speed 10 MHz */
#define DDL_GPIO_MODE_OUTPUT_2MHz           GPIO_CFGLOW_MODE0_1      /*!< Select Output mode, max speed 20 MHz */
#define DDL_GPIO_MODE_OUTPUT_50MHz          GPIO_CFGLOW_MODE0        /*!< Select Output mode, max speed 50 MHz */

#define DDL_GPIO_SPEED_FREQ_LOW             DDL_GPIO_MODE_OUTPUT_2MHz    /*!< Select I/O low output speed    */
#define DDL_GPIO_SPEED_FREQ_MEDIUM          DDL_GPIO_MODE_OUTPUT_10MHz   /*!< Select I/O medium output speed */
#define DDL_GPIO_SPEED_FREQ_HIGH            DDL_GPIO_MODE_OUTPUT_50MHz   /*!< Select I/O high output speed   */
#endif /* GPIO_OSSEL_OSSEL0 */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_PULL Pull Up Pull Down
  * @{
  */
#if defined (GPIO_PUPD_PUPD0)
#define DDL_GPIO_PULL_NO                    (0x00000000U)      /*!< Select I/O no pull */
#define DDL_GPIO_PULL_UP                    GPIO_PUPD_PUPDR0_0 /*!< Select I/O pull up */
#define DDL_GPIO_PULL_DOWN                  GPIO_PUPD_PUPDR0_1 /*!< Select I/O pull down */
#else
#define DDL_GPIO_PULL_DOWN                  0x00000000U          /*!< Select I/O pull down */
#define DDL_GPIO_PULL_UP                    GPIO_ODATA_ODATA0    /*!< Select I/O pull up */
#endif
/**
  * @}
  */

#if defined (GPIO_ALFL_ALFSEL0)
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
#endif /* GPIO_ALFL_ALFSEL0 */

#if defined (AFIO)
/** @defgroup GPIO_LL_EVENTOUT_PIN EVENTOUT Pin
  * @{
  */

#define DDL_GPIO_AF_EVENTOUT_PIN_0        AFIO_EVCTRL_PINSEL_PX0   /*!< EVENTOUT on pin 0 */
#define DDL_GPIO_AF_EVENTOUT_PIN_1        AFIO_EVCTRL_PINSEL_PX1   /*!< EVENTOUT on pin 1 */
#define DDL_GPIO_AF_EVENTOUT_PIN_2        AFIO_EVCTRL_PINSEL_PX2   /*!< EVENTOUT on pin 2 */
#define DDL_GPIO_AF_EVENTOUT_PIN_3        AFIO_EVCTRL_PINSEL_PX3   /*!< EVENTOUT on pin 3 */
#define DDL_GPIO_AF_EVENTOUT_PIN_4        AFIO_EVCTRL_PINSEL_PX4   /*!< EVENTOUT on pin 4 */
#define DDL_GPIO_AF_EVENTOUT_PIN_5        AFIO_EVCTRL_PINSEL_PX5   /*!< EVENTOUT on pin 5 */
#define DDL_GPIO_AF_EVENTOUT_PIN_6        AFIO_EVCTRL_PINSEL_PX6   /*!< EVENTOUT on pin 6 */
#define DDL_GPIO_AF_EVENTOUT_PIN_7        AFIO_EVCTRL_PINSEL_PX7   /*!< EVENTOUT on pin 7 */
#define DDL_GPIO_AF_EVENTOUT_PIN_8        AFIO_EVCTRL_PINSEL_PX8   /*!< EVENTOUT on pin 8 */
#define DDL_GPIO_AF_EVENTOUT_PIN_9        AFIO_EVCTRL_PINSEL_PX9   /*!< EVENTOUT on pin 9 */
#define DDL_GPIO_AF_EVENTOUT_PIN_10       AFIO_EVCTRL_PINSEL_PX10  /*!< EVENTOUT on pin 10 */
#define DDL_GPIO_AF_EVENTOUT_PIN_11       AFIO_EVCTRL_PINSEL_PX11  /*!< EVENTOUT on pin 11 */
#define DDL_GPIO_AF_EVENTOUT_PIN_12       AFIO_EVCTRL_PINSEL_PX12  /*!< EVENTOUT on pin 12 */
#define DDL_GPIO_AF_EVENTOUT_PIN_13       AFIO_EVCTRL_PINSEL_PX13  /*!< EVENTOUT on pin 13 */
#define DDL_GPIO_AF_EVENTOUT_PIN_14       AFIO_EVCTRL_PINSEL_PX14  /*!< EVENTOUT on pin 14 */
#define DDL_GPIO_AF_EVENTOUT_PIN_15       AFIO_EVCTRL_PINSEL_PX15  /*!< EVENTOUT on pin 15 */

/**
  * @}
  */

/** @defgroup GPIO_DDL_EVENTOUT_PORT EVENTOUT Port
  * @{
  */

#define DDL_GPIO_AF_EVENTOUT_PORT_A       AFIO_EVCTRL_PORTSEL_PA  /*!< EVENTOUT on port A */
#define DDL_GPIO_AF_EVENTOUT_PORT_B       AFIO_EVCTRL_PORTSEL_PB  /*!< EVENTOUT on port B */
#define DDL_GPIO_AF_EVENTOUT_PORT_C       AFIO_EVCTRL_PORTSEL_PC  /*!< EVENTOUT on port C */
#define DDL_GPIO_AF_EVENTOUT_PORT_D       AFIO_EVCTRL_PORTSEL_PD  /*!< EVENTOUT on port D */

/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_EINT_PORT GPIO EINT PORT
  * @{
  */
#define DDL_GPIO_AF_EINT_PORTA            0U   /*!< EINT PORT A */
#define DDL_GPIO_AF_EINT_PORTB            1U   /*!< EINT PORT B */
#define DDL_GPIO_AF_EINT_PORTC            2U   /*!< EINT PORT C */
#define DDL_GPIO_AF_EINT_PORTD            3U   /*!< EINT PORT D */
/**
  * @}
  */

/** @defgroup GPIO_DDL_EC_EINT_LINE GPIO EINT LINE
  * @{
  */
#define DDL_GPIO_AF_EINT_LINE0            (0x000FU << 16U | 0U)  /*!< AFIO_EINTSEL1  | EINT[0] */
#define DDL_GPIO_AF_EINT_LINE1            (0x00F0U << 16U | 0U)  /*!< AFIO_EINTSEL2  | EINT[0] */
#define DDL_GPIO_AF_EINT_LINE2            (0x0F00U << 16U | 0U)  /*!< AFIO_EINTSEL3  | EINT[0] */
#define DDL_GPIO_AF_EINT_LINE3            (0xF000U << 16U | 0U)  /*!< AFIO_EINTSEL4  | EINT[0] */
#define DDL_GPIO_AF_EINT_LINE4            (0x000FU << 16U | 1U)  /*!< AFIO_EINTSEL1  | EINT[1] */
#define DDL_GPIO_AF_EINT_LINE5            (0x00F0U << 16U | 1U)  /*!< AFIO_EINTSEL2  | EINT[1] */
#define DDL_GPIO_AF_EINT_LINE6            (0x0F00U << 16U | 1U)  /*!< AFIO_EINTSEL3  | EINT[1] */
#define DDL_GPIO_AF_EINT_LINE7            (0xF000U << 16U | 1U)  /*!< AFIO_EINTSEL4  | EINT[1] */
#define DDL_GPIO_AF_EINT_LINE8            (0x000FU << 16U | 2U)  /*!< AFIO_EINTSEL1  | EINT[2] */
#define DDL_GPIO_AF_EINT_LINE9            (0x00F0U << 16U | 2U)  /*!< AFIO_EINTSEL2  | EINT[2] */
#define DDL_GPIO_AF_EINT_LINE10           (0x0F00U << 16U | 2U)  /*!< AFIO_EINTSEL3  | EINT[2] */
#define DDL_GPIO_AF_EINT_LINE11           (0xF000U << 16U | 2U)  /*!< AFIO_EINTSEL4  | EINT[2] */
#define DDL_GPIO_AF_EINT_LINE12           (0x000FU << 16U | 3U)  /*!< AFIO_EINTSEL1  | EINT[3] */
#define DDL_GPIO_AF_EINT_LINE13           (0x00F0U << 16U | 3U)  /*!< AFIO_EINTSEL2  | EINT[3] */
#define DDL_GPIO_AF_EINT_LINE14           (0x0F00U << 16U | 3U)  /*!< AFIO_EINTSEL3  | EINT[3] */
#define DDL_GPIO_AF_EINT_LINE15           (0xF000U << 16U | 3U)  /*!< AFIO_EINTSEL4  | EINT[3] */
/**
  * @}
  */
#endif /* AFIO */

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
  *         @arg @ref DDL_GPIO_MODE_FLOATING (1)
  *
  *         (1) This parameter is only for APM32F402/403xx device.\n
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
#if defined (GPIO_MODE_MODE0)
  MODIFY_REG(GPIOx->MODE, (GPIO_MODE_MODE0 << (POSITION_VAL(Pin) * 2U)), (Mode << (POSITION_VAL(Pin) * 2U)));
#else
  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CFGLOW) + (Pin >> 24));
  MODIFY_REG(*pReg, ((GPIO_CFGLOW_CFG0 | GPIO_CFGLOW_MODE0) << (POSITION_VAL(Pin) * 4U)), (Mode << (POSITION_VAL(Pin) * 4U)));
#endif /* GPIO_MODE_MODE0 */
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
  *         @arg @ref DDL_GPIO_MODE_FLOATING (1)
  *
  *         (1) This parameter is only for APM32F402/403xx device.\n
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
#if defined (GPIO_MODE_MODE0)
  return (uint32_t)(READ_BIT(GPIOx->MODE,
                             (GPIO_MODE_MODE0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
#else
  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CFGLOW) + (Pin >> 24));
  return (READ_BIT(*pReg, ((GPIO_CFGLOW_CFG0 | GPIO_CFGLOW_MODE0) << (POSITION_VAL(Pin) * 4U))) >> (POSITION_VAL(Pin) * 4U));
#endif /* GPIO_MODE_MODE0 */
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
#if defined (GPIO_OMODE_OMODE0)
  MODIFY_REG(GPIOx->OMODE, PinMask, (PinMask * OutputType));
#else
  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CFGLOW) + (PinMask >> 24));
  MODIFY_REG(*pReg, (GPIO_CFGLOW_CFG0_0 << (POSITION_VAL(PinMask) * 4U)),
             (OutputType << (POSITION_VAL(PinMask) * 4U)));
#endif /* GPIO_OMODE_OMODE0 */
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
#if defined (GPIO_OMODE_OMODE0)
  return (uint32_t)(READ_BIT(GPIOx->OMODE, Pin) >> POSITION_VAL(Pin));
#else
  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CFGLOW) + (Pin >> 24));
  return (READ_BIT(*pReg, (GPIO_CFGLOW_CFG0_0 << (POSITION_VAL(Pin) * 4U))) >> (POSITION_VAL(Pin) * 4U));
#endif /* GPIO_OMODE_OMODE0 */
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
  *         @arg @ref DDL_GPIO_SPEED_FREQ_VERY_HIGH (1)
  *
  *         (1) This parameter is only for APM32F402/403xx device.\n
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinSpeed(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t  Speed)
{
#if defined (GPIO_OSPEEDER_OSPEEDR0)
  MODIFY_REG(GPIOx->OSSEL, (GPIO_OSPEEDER_OSPEEDR0 << (POSITION_VAL(Pin) * 2U)),
             (Speed << (POSITION_VAL(Pin) * 2U)));
#else
  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CFGLOW) + (Pin >> 24));
  MODIFY_REG(*pReg, (GPIO_CFGLOW_MODE0 << (POSITION_VAL(Pin) * 4U)),
             (Speed << (POSITION_VAL(Pin) * 4U)));
#endif /* GPIO_OSPEEDER_OSPEEDR0 */
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
  *         @arg @ref DDL_GPIO_SPEED_FREQ_VERY_HIGH (1)
  *
  *         (1) This parameter is only for APM32F402/403xx device.\n
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinSpeed(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
#if defined (GPIO_OSPEEDER_OSPEEDR0)
  return (uint32_t)(READ_BIT(GPIOx->OSSEL,
                             (GPIO_OSPEEDER_OSPEEDR0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
#else
  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CFGLOW) + (Pin >> 24));
  return (READ_BIT(*pReg, (GPIO_CFGLOW_MODE0 << (POSITION_VAL(Pin) * 4U))) >> (POSITION_VAL(Pin) * 4U));
#endif /* GPIO_OSPEEDER_OSPEEDR0 */
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
  *         @arg @ref DDL_GPIO_PULL_NO   (1)
  *         @arg @ref DDL_GPIO_PULL_UP
  *         @arg @ref DDL_GPIO_PULL_DOWN
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_SetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Pull)
{
#if defined (GPIO_PUPD_PUPDR0)
  MODIFY_REG(GPIOx->PUPD, (GPIO_PUPD_PUPDR0 << (POSITION_VAL(Pin) * 2U)), (Pull << (POSITION_VAL(Pin) * 2U)));
#else
  MODIFY_REG(GPIOx->ODATA, (Pin >> GPIO_PIN_MASK_POS), Pull << (POSITION_VAL(Pin >> GPIO_PIN_MASK_POS)));
#endif /* GPIO_PUPD_PUPDR0 */
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
  *         @arg @ref DDL_GPIO_PULL_NO   (1)
  *         @arg @ref DDL_GPIO_PULL_UP
  *         @arg @ref DDL_GPIO_PULL_DOWN
  */
__STATIC_INLINE uint32_t DDL_GPIO_GetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
#if defined (GPIO_PUPD_PUPDR0)
  return (uint32_t)(READ_BIT(GPIOx->PUPD,
                             (GPIO_PUPD_PUPDR0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
#else
  return (READ_BIT(GPIOx->ODATA, (GPIO_ODATA_ODATA0 << (POSITION_VAL(Pin >> GPIO_PIN_MASK_POS))))
                   >> (POSITION_VAL(Pin >> GPIO_PIN_MASK_POS)));
#endif /* GPIO_PUPD_PUPDR0 */
}

#if defined (GPIO_ALFL_ALFSEL0)
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
#endif /* GPIO_ALFL_ALFSEL0 */

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
#if defined (GPIO_PIN_MASK_POS)
  WRITE_REG(GPIOx->LOCK, GPIO_LOCK_LOCKKEY | ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU));
  WRITE_REG(GPIOx->LOCK, PinMask);
  WRITE_REG(GPIOx->LOCK, GPIO_LOCK_LOCKKEY | ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU));
  temp = READ_REG(GPIOx->LOCK);
  (void) temp;
#else
  WRITE_REG(GPIOx->LOCK, GPIO_LOCK_LOCKKEY | PinMask);
  WRITE_REG(GPIOx->LOCK, PinMask);
  WRITE_REG(GPIOx->LOCK, GPIO_LOCK_LOCKKEY | PinMask);
  temp = READ_REG(GPIOx->LOCK);
  (void) temp;
#endif /* GPIO_PIN_MASK_POS */
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
#if defined (GPIO_PIN_MASK_POS)
  return (READ_BIT(GPIOx->LOCK, ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU)) == (((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU)));
#else
  return (READ_BIT(GPIOx->LOCK, PinMask) == (PinMask));
#endif /* GPIO_PIN_MASK_POS */
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
#if defined (GPIO_PIN_MASK_POS)
  return (READ_BIT(GPIOx->IDATA, ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU)) == ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU));
#else
  return (READ_BIT(GPIOx->IDATA, PinMask) == (PinMask));
#endif /* GPIO_PIN_MASK_POS */
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
#if defined (GPIO_PIN_MASK_POS)
  return (READ_BIT(GPIOx->ODATA, ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU)) == ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU));
#else
  return (READ_BIT(GPIOx->ODATA, PinMask) == (PinMask));
#endif /* GPIO_PIN_MASK_POS */
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
#if defined (GPIO_PIN_MASK_POS)
  WRITE_REG(GPIOx->BSC, ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU));
#else
  WRITE_REG(GPIOx->BSC, PinMask);
#endif /* GPIO_PIN_MASK_POS */
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
#if defined (GPIO_PIN_MASK_POS)
  WRITE_REG(GPIOx->BSC, ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU) << 16);
#else
  WRITE_REG(GPIOx->BSC, (PinMask << 16));
#endif /* GPIO_PIN_MASK_POS */
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
#if defined (GPIO_PIN_MASK_POS)
  uint32_t odr = READ_REG(GPIOx->ODATA);
  WRITE_REG(GPIOx->BSC, ((odr & ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU)) << 16u) |
                         (~odr & ((PinMask >> GPIO_PIN_MASK_POS) & 0x0000FFFFU)));
#else
  uint32_t odr = READ_REG(GPIOx->ODATA);
  WRITE_REG(GPIOx->BSC, ((odr & PinMask) << 16u) | (~odr & PinMask));
#endif
}

/**
  * @}
  */

#if defined (AFIO)

/** @defgroup GPIO_AF_REMAPPING Alternate Function Remapping
  * @brief This section propose definition to remap the alternate function to some other port/pins.
  * @{
  */

/**
  * @brief  Enable the remapping of SPI1 alternate function NSS, SCK, MISO and MOSI.
  * @note   ENABLE: Remap (NSS/PA15, SCK/PB3, MISO/PB4, MOSI/PB5)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_SPI1(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_SPI1_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of SPI1 alternate function NSS, SCK, MISO and MOSI.
  * @note  DISABLE: No remap (NSS/PA4,  SCK/PA5, MISO/PA6, MOSI/PA7)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_SPI1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_SPI1_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if SPI1 has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_SPI1(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_SPI1_RMP) == (AFIO_REMAP1_SPI1_RMP));
}

/**
  * @brief Enable the remapping of I2C1 alternate function SCL and SDA.
  * @note  ENABLE: Remap     (SCL/PB8, SDA/PB9)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_I2C1(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_I2C1_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of I2C1 alternate function SCL and SDA.
  * @note  DISABLE: No remap (SCL/PB6, SDA/PB7)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_I2C1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_I2C1_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if I2C1 has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_I2C1(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_I2C1_RMP) == (AFIO_REMAP1_I2C1_RMP));
}

/**
  * @brief Enable the remapping of USART1 alternate function TX and RX.
  * @note  ENABLE: Remap     (TX/PB6, RX/PB7)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_USART1(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_USART1_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of USART1 alternate function TX and RX.
  * @rmtoll MAPR          USART1_REMAP           LL_GPIO_AF_DisableRemap_USART1
  * @note  DISABLE: No remap (TX/PA9, RX/PA10)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_USART1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_USART1_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if USART1 has been remaped or not
  * @rmtoll MAPR         USART1_REMAP         LL_GPIO_AF_IsEnabledRemap_USART1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_USART1(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_USART1_RMP) == (AFIO_REMAP1_USART1_RMP));
}

/**
  * @brief Enable the remapping of USART2 alternate function CTS, RTS, CK, TX and RX.
  * @note  ENABLE: Remap no effect
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_USART2(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_USART2_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of USART2 alternate function CTS, RTS, CK, TX and RX.
  * @note  DISABLE: No remap (CTS/PA0, RTS/PA1, TX/PA2, RX/PA3, CK/PA4)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_USART2(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_USART2_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if USART2 has been remaped or not
  * @rmtoll MAPR         USART2_REMAP         LL_GPIO_AF_IsEnabledRemap_USART2
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_USART2(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_USART2_RMP) == (AFIO_REMAP1_USART2_RMP));
}

/**
  * @brief Enable the remapping of USART3 alternate function CTS, RTS, CK, TX and RX.
  * @note  PARTIAL: Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial_USART3(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_USART3_RMP | AFIO_REMAP1_SWJ_CFG),
                          (AFIO_REMAP1_USART3_RMP_PARTIALREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Disable the remapping of USART3 alternate function CTS, RTS, CK, TX and RX.
  * @note  DISABLE: No remap      (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_USART3(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_USART3_RMP | AFIO_REMAP1_SWJ_CFG),
                         (AFIO_REMAP1_USART3_RMP_NOREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of TMR1 alternate function channels 1 to 4, 1N to 3N, external trigger (ETR) and Break input (BKIN)
  * @note  PARTIAL: Partial remap (ETR/PA12, CH1/PA8, CH2/PA9,  CH3/PA10, CH4/PA11, BKIN/PA6,  CH1N/PA7,  CH2N/PB0,  CH3N/PB1)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial_TMR1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR1_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR1_RMP_PARTIALREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Disable the remapping of TMR1 alternate function channels 1 to 4, 1N to 3N, external trigger (ETR) and Break input (BKIN)
  * @note  DISABLE: No remap      (ETR/PA12, CH1/PA8, CH2/PA9,  CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_TMR1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR1_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR1_RMP_NOREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  ENABLE: Full remap       (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_TMR2(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR2_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR2_RMP_FULLREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  PARTIAL_2: Partial remap (CH1/ETR/PA0,  CH2/PA1, CH3/PB10, CH4/PB11)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial2_TMR2(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR2_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR2_RMP_PARTIALREMAP2 | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  PARTIAL_1: Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2,  CH4/PA3)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial1_TMR2(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR2_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR2_RMP_PARTIALREMAP1 | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Disable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  DISABLE: No remap        (CH1/ETR/PA0,  CH2/PA1, CH3/PA2,  CH4/PA3)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_TMR2(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR2_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR2_RMP_NOREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of TMR3 alternate function channels 1 to 4
  * @note  ENABLE: Full remap     (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9)
  * @note  TMR3_ETR on PE0 is not re-mapped.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_TMR3(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR3_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR3_RMP_FULLREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of TMR3 alternate function channels 1 to 4
  * @note  PARTIAL: Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1)
  * @note  TMR3_ETR on PE0 is not re-mapped.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial_TMR3(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR3_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR3_RMP_PARTIALREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Disable the remapping of TMR3 alternate function channels 1 to 4
  * @note  DISABLE: No remap      (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1)
  * @note  TMR3_ETR on PE0 is not re-mapped.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_TMR3(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR3_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_TMR3_RMP_NOREMAP | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of TMR4 alternate function channels 1 to 4.
  * @note  ENABLE: Full remap  no effect
  * @note  TMR4_ETR on PE0 is not re-mapped.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_TMR4(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_TMR4_RMP | AFIO_REMAP1_SWJ_CFG);
}
/**
  * @brief Disable the remapping of TMR4 alternate function channels 1 to 4.
  * @note  DISABLE: No remap  (TMR4_CH1/PB6,  TMR4_CH2/PB7,  TMR4_CH3/PB8,  TMR4_CH4/PB9)
  * @note  TMR4_ETR on PE0 is not re-mapped.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_TMR4(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR4_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if TMR4 has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_TMR4(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_TMR4_RMP) == (AFIO_REMAP1_TMR4_RMP));
}

/**
  * @brief Enable or disable the remapping of CAN alternate function CAN_RX and CAN_TX in devices with a single CAN interface.
  * @note  CASE 1: CAN_RX mapped to PA11, CAN_TX mapped to PA12
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial1_CAN1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_CAN1_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_CAN1_RMP_REMAP1 | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable or disable the remapping of CAN alternate function CAN_RX and CAN_TX in devices with a single CAN interface.
  * @note  CASE 2: CAN_RX mapped to PB8,  CAN_TX mapped to PB9 (not available on 36-pin package)
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial2_CAN1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_CAN1_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_CAN1_RMP_REMAP2 | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable or disable the remapping of CAN alternate function CAN_RX and CAN_TX in devices with a single CAN interface.
  * @note  CASE 3: CAN_RX mapped to PD0,  CAN_TX mapped to PD1
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_RemapPartial3_CAN1(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_CAN1_RMP | AFIO_REMAP1_SWJ_CFG), (AFIO_REMAP1_CAN1_RMP_REMAP3 | AFIO_REMAP1_SWJ_CFG));
}

/**
  * @brief Enable the remapping of PD0 and PD1. When the HSE oscillator is not used
  *        (application running on internal 8 MHz RC) PD0 and PD1 can be mapped on OSC_IN and
  *        OSC_OUT. This is available only on 36, 48 and 64 pins packages (PD0 and PD1 are available
  *        on 100-pin and 144-pin packages, no need for remapping).
  * @note  ENABLE: PD0 remapped on OSC_IN, PD1 remapped on OSC_OUT.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_PD01(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_PD01_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of PD0 and PD1. When the HSE oscillator is not used
  *        (application running on internal 8 MHz RC) PD0 and PD1 can be mapped on OSC_IN and
  *        OSC_OUT. This is available only on 36, 48 and 64 pins packages (PD0 and PD1 are available
  *        on 100-pin and 144-pin packages, no need for remapping).
  * @note  DISABLE: No remapping of PD0 and PD1
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_PD01(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_PD01_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if PD01 has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_PD01(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_PD01_RMP) == (AFIO_REMAP1_PD01_RMP));
}

/**
  * @brief Enable the remapping of TMR5CH4.
  * @note  ENABLE: LSI internal clock is connected to TMR5_CH4 input for calibration purpose.
  * @note  This function is available only in high density value line devices.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_TMR5CH4(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_TMR5CH4_IRMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of TMR5CH4.
  * @note  DISABLE: TMR5_CH4 is connected to PA3
  * @note  This function is available only in high density value line devices.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_TMR5CH4(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_TMR5CH4_IRMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if TMR5CH4 has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_TMR5CH4(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_TMR5CH4_IRMP) == (AFIO_REMAP1_TMR5CH4_IRMP));
}

/**
  * @brief Enable the remapping of ADC1_ETRGINJ (ADC 1 External trigger injected conversion).
  * @note  ENABLE: ADC1 External Event injected conversion is connected to TMR8 Channel4.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_ADC1_ETRGINJ(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC1_ETRGINJC_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of ADC1_ETRGINJ (ADC 1 External trigger injected conversion).
  * @note  DISABLE: ADC1 External trigger injected conversion is connected to EINT15
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_ADC1_ETRGINJ(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_ADC1_ETRGINJC_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if ADC1_ETRGINJ has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_ADC1_ETRGINJ(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC1_ETRGINJC_RMP) == (AFIO_REMAP1_ADC1_ETRGINJC_RMP));
}

/**
  * @brief Enable the remapping of ADC1_ETRGREG (ADC 1 External trigger regular conversion).
  * @note  ENABLE: ADC1 External Event regular conversion is connected to TMR8 TRG0.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_ADC1_ETRGREG(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC1_ETRGREGC_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of ADC1_ETRGREG (ADC 1 External trigger regular conversion).
  * @note  DISABLE: ADC1 External trigger regular conversion is connected to EINT11
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_ADC1_ETRGREG(void)
{
   MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_ADC1_ETRGREGC_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if ADC1_ETRGREG has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_ADC1_ETRGREG(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC1_ETRGREGC_RMP) == (AFIO_REMAP1_ADC1_ETRGREGC_RMP));
}

/**
  * @brief Enable the remapping of ADC2_ETRGREG (ADC 2 External trigger injected conversion).
  * @note  ENABLE: ADC2 External Event injected conversion is connected to TMR8 Channel4.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_ADC2_ETRGINJ(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC2_ETRGINJC_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of ADC2_ETRGREG (ADC 2 External trigger injected conversion).
  * @note  DISABLE: ADC2 External trigger injected conversion is connected to EINT15
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_ADC2_ETRGINJ(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_ADC2_ETRGINJC_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if ADC2_ETRGINJ has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_ADC2_ETRGINJ(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC2_ETRGINJC_RMP) == (AFIO_REMAP1_ADC2_ETRGINJC_RMP));
}

/**
  * @brief Enable the remapping of ADC2_ETRGREG (ADC 2 External trigger regular conversion).
  * @note  ENABLE: ADC2 External Event regular conversion is connected to TMR8 TRG0.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_ADC2_ETRGREG(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC2_ETRGREGC_RMP | AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief Disable the remapping of ADC2_ETRGREG (ADC 2 External trigger regular conversion).
  * @note  DISABLE: ADC2 External trigger regular conversion is connected to EINT11
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_ADC2_ETRGREG(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_ADC2_ETRGREGC_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if ADC2_ETRGREG has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_ADC2_ETRGREG(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_ADC2_ETRGREGC_RMP) == (AFIO_REMAP1_ADC2_ETRGREGC_RMP));
}

/**
  * @brief Enable the remapping of CAN2 alternate function CAN2_RX and CAN2_TX.
  * @note  ENABLE: Remap     (CAN2_RX/PB5,  CAN2_TX/PB6)
  * @note  This bit is available only in connectivity line devices and is reserved otherwise.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_CAN2(void)
{
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_CAN2_RMP | AFIO_REMAP1_SWJ_CFG);
}
/**
  * @brief Disable the remapping of CAN2 alternate function CAN2_RX and CAN2_TX.
  * @note  DISABLE: No remap (CAN2_RX/PB12, CAN2_TX/PB13)
  * @note  This bit is available only in connectivity line devices and is reserved otherwise.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_CAN2(void)
{
  MODIFY_REG(AFIO->REMAP1, (AFIO_REMAP1_CAN2_RMP | AFIO_REMAP1_SWJ_CFG), AFIO_REMAP1_SWJ_CFG);
}

/**
  * @brief  Check if CAN2 has been remaped or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_IsEnabledRemap_CAN2(void)
{
  return (READ_BIT(AFIO->REMAP1, AFIO_REMAP1_CAN2_RMP) == (AFIO_REMAP1_CAN2_RMP));
}

/**
  * @brief Enable the Serial wire JTAG configuration
  * @note  ENABLE: Full SWJ (JTAG-DP + SW-DP): Reset State
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableRemap_SWJ(void)
{
  CLEAR_BIT(AFIO->REMAP1,AFIO_REMAP1_SWJ_CFG);
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_SWJ_CFG_RESET);
}

/**
  * @brief Enable the Serial wire JTAG configuration
  * @note  NONJTRST: Full SWJ (JTAG-DP + SW-DP) but without NJTRST
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_Remap_SWJ_NONJTRST(void)
{
  CLEAR_BIT(AFIO->REMAP1,AFIO_REMAP1_SWJ_CFG);
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_SWJ_CFG_NOJNTRST);
}

/**
  * @brief Enable the Serial wire JTAG configuration
  * @note  NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_Remap_SWJ_NOJTAG(void)
{
  CLEAR_BIT(AFIO->REMAP1,AFIO_REMAP1_SWJ_CFG);
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_SWJ_CFG_JTAGDISABLE);
}

/**
  * @brief Disable the Serial wire JTAG configuration
  * @note  DISABLE: JTAG-DP Disabled and SW-DP Disabled
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableRemap_SWJ(void)
{
  CLEAR_BIT(AFIO->REMAP1,AFIO_REMAP1_SWJ_CFG);
  SET_BIT(AFIO->REMAP1, AFIO_REMAP1_SWJ_CFG_DISABLE);
}

/**
  * @}
  */

/** @defgroup GPIO_AF_LL_EVENTOUT Output Event configuration
  * @brief This section propose definition to Configure EVENTOUT Cortex feature .
  * @{
  */

/**
  * @brief  Configures the port and pin on which the EVENTOUT Cortex signal will be connected.
  * @param  DDL_GPIO_PortSource This parameter can be one of the following values:
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PORT_A
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PORT_B
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PORT_C
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PORT_D
  * @param  DDL_GPIO_PinSource This parameter can be one of the following values:
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_0
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_1
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_2
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_3
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_4
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_5
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_6
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_7
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_8
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_9
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_10
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_11
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_12
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_13
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_14
  *      @arg @ref DDL_GPIO_AF_EVENTOUT_PIN_15
  * @retval None
*/
__STATIC_INLINE void DDL_GPIO_AF_ConfigEventout(uint32_t DDL_GPIO_PortSource, uint32_t DDL_GPIO_PinSource)
{
  MODIFY_REG(AFIO->EVCTRL, (AFIO_EVCTRL_PORTSEL_Msk) | (AFIO_EVCTRL_PINSEL), (DDL_GPIO_PortSource) | (DDL_GPIO_PinSource));
}

/**
  * @brief  Enables the Event Output.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_EnableEventout(void)
{
  SET_BIT(AFIO->EVCTRL, AFIO_EVCTRL_EVOEN);
}

/**
  * @brief  Disables the Event Output.
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_DisableEventout(void)
{
  CLEAR_BIT(AFIO->EVCTRL, AFIO_EVCTRL_EVOEN);
}

/**
  * @}
  */

/** @defgroup GPIO_AF_DDL_EINT EINT external interrupt
  * @brief This section Configure source input for the EINT external interrupt .
  * @{
  */

/**
  * @brief  Configure source input for the EINT external interrupt.
  * @param  Port This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_EINT_PORTA
  *         @arg @ref DDL_GPIO_AF_EINT_PORTB
  *         @arg @ref DDL_GPIO_AF_EINT_PORTC
  *         @arg @ref DDL_GPIO_AF_EINT_PORTD
  * @param  Line This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_EINT_LINE0
  *         @arg @ref DDL_GPIO_AF_EINT_LINE1
  *         @arg @ref DDL_GPIO_AF_EINT_LINE2
  *         @arg @ref DDL_GPIO_AF_EINT_LINE3
  *         @arg @ref DDL_GPIO_AF_EINT_LINE4
  *         @arg @ref DDL_GPIO_AF_EINT_LINE5
  *         @arg @ref DDL_GPIO_AF_EINT_LINE6
  *         @arg @ref DDL_GPIO_AF_EINT_LINE7
  *         @arg @ref DDL_GPIO_AF_EINT_LINE8
  *         @arg @ref DDL_GPIO_AF_EINT_LINE9
  *         @arg @ref DDL_GPIO_AF_EINT_LINE10
  *         @arg @ref DDL_GPIO_AF_EINT_LINE11
  *         @arg @ref DDL_GPIO_AF_EINT_LINE12
  *         @arg @ref DDL_GPIO_AF_EINT_LINE13
  *         @arg @ref DDL_GPIO_AF_EINT_LINE14
  *         @arg @ref DDL_GPIO_AF_EINT_LINE15
  * @retval None
  */
__STATIC_INLINE void DDL_GPIO_AF_SetEINTSource(uint32_t Port, uint32_t Line)
{
  MODIFY_REG(AFIO->EINTSEL[Line & 0xFF], (Line >> 16), Port << POSITION_VAL((Line >> 16)));
}

/**
  * @brief  Get the configured defined for specific EINT Line
  * @param  Line This parameter can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_EINT_LINE0
  *         @arg @ref DDL_GPIO_AF_EINT_LINE1
  *         @arg @ref DDL_GPIO_AF_EINT_LINE2
  *         @arg @ref DDL_GPIO_AF_EINT_LINE3
  *         @arg @ref DDL_GPIO_AF_EINT_LINE4
  *         @arg @ref DDL_GPIO_AF_EINT_LINE5
  *         @arg @ref DDL_GPIO_AF_EINT_LINE6
  *         @arg @ref DDL_GPIO_AF_EINT_LINE7
  *         @arg @ref DDL_GPIO_AF_EINT_LINE8
  *         @arg @ref DDL_GPIO_AF_EINT_LINE9
  *         @arg @ref DDL_GPIO_AF_EINT_LINE10
  *         @arg @ref DDL_GPIO_AF_EINT_LINE11
  *         @arg @ref DDL_GPIO_AF_EINT_LINE12
  *         @arg @ref DDL_GPIO_AF_EINT_LINE13
  *         @arg @ref DDL_GPIO_AF_EINT_LINE14
  *         @arg @ref DDL_GPIO_AF_EINT_LINE15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_GPIO_AF_EINT_PORTA
  *         @arg @ref DDL_GPIO_AF_EINT_PORTB
  *         @arg @ref DDL_GPIO_AF_EINT_PORTC
  *         @arg @ref DDL_GPIO_AF_EINT_PORTD
  */
__STATIC_INLINE uint32_t DDL_GPIO_AF_GetEINTSource(uint32_t Line)
{
  return (uint32_t)(READ_BIT(AFIO->EINTSEL[Line & 0xFF], (Line >> 16)) >> POSITION_VAL(Line >> 16));
}

/**
  * @}
  */
#endif /* AFIO */

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

