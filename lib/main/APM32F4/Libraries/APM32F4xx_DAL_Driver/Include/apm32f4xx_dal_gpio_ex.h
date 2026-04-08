/**
  *
  * @file    apm32f4xx_dal_gpio_ex.h
  * @brief   Header file of GPIO DAL Extension module.
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
#ifndef APM32F4xx_DAL_GPIO_EX_H
#define APM32F4xx_DAL_GPIO_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup GPIOEx GPIOEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Constants GPIO Exported Constants
  * @{
  */

/** @defgroup GPIO_Alternate_function_selection GPIO Alternate Function Selection
  * @{
  */

/*---------------------------------- APM32F407xx/APM32F417xx------------------*/
#if defined(APM32F407xx) || defined(APM32F417xx)
/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TMR1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TMR2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TMR3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TMR4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TMR5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TMR8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TMR9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TMR10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TMR11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping        */
#define GPIO_AF5_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_AF5_I2S3ext       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF6_I2S2ext       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_AF7_I2S3ext       ((uint8_t)0x07)  /* I2S3ext_SD Alternate Function mapping */

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_AF9_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_AF9_TMR12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF9_TMR13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF9_TMR14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_OTG_FS        ((uint8_t)0x0A)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF10_OTG_HS        ((uint8_t)0x0A)  /* OTG_HS Alternate Function mapping */

/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_ETH           ((uint8_t)0x0B)  /* ETHERNET Alternate Function mapping */

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_SMC           ((uint8_t)0x0C)  /* SMC Alternate Function mapping                      */
#define GPIO_AF12_DMC           ((uint8_t)0x0C)  /* DMC Alternate Function mapping                      */
#define GPIO_AF12_OTG_HS_FS     ((uint8_t)0x0C)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF12_SDIO          ((uint8_t)0x0C)  /* SDIO Alternate Function mapping                     */

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_DCI           ((uint8_t)0x0D)  /* DCI Alternate Function mapping */

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */
#endif /* APM32F407xx || APM32F417xx */
/*----------------------------------------------------------------------------*/

/*---------------------------------- APM32F405xx/APM32F415xx------------------*/
#if defined(APM32F405xx) || defined(APM32F415xx)
/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TMR1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TMR2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TMR3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TMR4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TMR5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TMR8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TMR9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TMR10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TMR11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping        */
#define GPIO_AF5_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_AF5_I2S3ext       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF6_I2S2ext       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_AF7_I2S3ext       ((uint8_t)0x07)  /* I2S3ext_SD Alternate Function mapping */

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_AF9_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_AF9_TMR12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF9_TMR13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF9_TMR14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_OTG_FS        ((uint8_t)0x0A)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF10_OTG_HS        ((uint8_t)0x0A)  /* OTG_HS Alternate Function mapping */

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_SMC           ((uint8_t)0x0C)  /* EMMC Alternate Function mapping                     */
#define GPIO_AF12_OTG_HS_FS     ((uint8_t)0x0C)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF12_SDIO          ((uint8_t)0x0C)  /* SDIO Alternate Function mapping                     */

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */
#endif /* APM32F405xx || APM32F415xx */

/*----------------------------------------------------------------------------*/

/*---------------------------------- APM32F465xx------------------*/
#if defined(APM32F465xx)
/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TMR1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TMR2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TMR3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TMR4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TMR5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TMR8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TMR9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TMR10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TMR11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping        */
#define GPIO_AF5_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_AF5_I2S3ext       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF6_I2S2ext       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_AF7_I2S3ext       ((uint8_t)0x07)  /* I2S3ext_SD Alternate Function mapping */

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_AF9_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_AF9_TMR12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF9_TMR13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF9_TMR14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_OTG_FS        ((uint8_t)0x0A)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF10_OTG_HS        ((uint8_t)0x0A)  /* OTG_HS Alternate Function mapping */

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_SMC           ((uint8_t)0x0C)  /* EMMC Alternate Function mapping                     */
#define GPIO_AF12_OTG_HS_FS     ((uint8_t)0x0C)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF12_SDIO          ((uint8_t)0x0C)  /* SDIO Alternate Function mapping                     */

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */
#endif /* APM32F465xx */

/*----------------------------------------------------------------------------*/

/*---------------------------------------- APM32F411xx------------------------*/
#if defined(APM32F411xx)
/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TMR1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TMR2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TMR3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TMR4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TMR5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TMR8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TMR9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TMR10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TMR11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)  /* SPI1/I2S1 Alternate Function mapping   */
#define GPIO_AF5_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_AF5_SPI3          ((uint8_t)0x05)  /* SPI3/I2S3 Alternate Function mapping   */
#define GPIO_AF5_SPI4          ((uint8_t)0x05)  /* SPI4 Alternate Function mapping        */
#define GPIO_AF5_I2S3ext       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI2          ((uint8_t)0x06)  /* I2S2 Alternate Function mapping       */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF6_SPI4          ((uint8_t)0x06)  /* SPI4/I2S4 Alternate Function mapping  */
#define GPIO_AF6_SPI5          ((uint8_t)0x06)  /* SPI5/I2S5 Alternate Function mapping  */
#define GPIO_AF6_I2S2ext       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_SPI3          ((uint8_t)0x07)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_AF7_I2S3ext       ((uint8_t)0x07)  /* I2S3ext_SD Alternate Function mapping */

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_USART3        ((uint8_t)0x08)  /* USART3 Alternate Function mapping */
#define GPIO_AF8_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */
#define GPIO_AF8_CAN1          ((uint8_t)0x08)  /* CAN1 Alternate Function mapping  */

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_AF9_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_AF9_TMR12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF9_TMR13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF9_TMR14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */
#define GPIO_AF9_I2C2          ((uint8_t)0x09)  /* I2C2 Alternate Function mapping  */
#define GPIO_AF9_I2C3          ((uint8_t)0x09)  /* I2C3 Alternate Function mapping  */
#define GPIO_AF9_QSPI          ((uint8_t)0x09)  /* QSPI Alternate Function mapping  */

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_OTG_FS        ((uint8_t)0x0A)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF10_QSPI          ((uint8_t)0x0A)  /* QSPI Alternate Function mapping   */
#define GPIO_AF10_SMC           ((uint8_t)0x0A)  /* SMC Alternate Function mapping    */

/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_SDIO          ((uint8_t)0x0B)  /* SDIO Alternate Function mapping  */
#define GPIO_AF11_SMC           ((uint8_t)0x0B)  /* SMC Alternate Function mapping   */

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */
#endif /* APM32F411xx */

/*---------------------------------- APM32F423xx/APM32F425xx/APM32F427xx------------------*/
#if defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TMR1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TMR2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TMR3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TMR4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TMR5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TMR8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TMR9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TMR10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TMR11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping        */
#define GPIO_AF5_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping   */

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_AF9_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_AF9_TMR12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF9_TMR13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF9_TMR14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */
#define GPIO_AF9_QSPI          ((uint8_t)0x09)  /* QSPI Alternate Function mapping */

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_OTG_FS        ((uint8_t)0x0A)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF10_QSPI          ((uint8_t)0x0A)  /* QSPI Alternate Function mapping */

/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_ETH           ((uint8_t)0x0B)  /* ETHERNET Alternate Function mapping */

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_SMC           ((uint8_t)0x0C)  /* SMC Alternate Function mapping                      */
#define GPIO_AF12_DMC           ((uint8_t)0x0C)  /* DMC Alternate Function mapping                      */
#define GPIO_AF12_OTG_FS2       ((uint8_t)0x0C)  /* OTG FS2 configured in FS, Alternate Function mapping */
#define GPIO_AF12_SDIO          ((uint8_t)0x0C)  /* SDIO Alternate Function mapping                     */

/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */
#endif /* APM32F423xx || APM32F425xx || APM32F427xx */
/*----------------------------------------------------------------------------*/

/**
  * @}
  */

#if defined(APM32F403xx) || defined(APM32F402xx)
/** @defgroup GPIO_Alternate_function_selection GPIO Alternate Function Remapping
  * @{
  */

/**
  * @brief Enable the remapping of SPI1 alternate function NSS, SCK, MISO and MOSI.
  * @note  ENABLE: Remap     (NSS/PA15, SCK/PB3, MISO/PB4, MOSI/PB5)
  * @retval None
  */
#define __DAL_AFIO_REMAP_SPI1_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_SPI1_RMP)

/**
  * @brief Disable the remapping of SPI1 alternate function NSS, SCK, MISO and MOSI.
  * @note  DISABLE: No remap  (NSS/PA4, SCK/PA5, MISO/PA6, MOSI/PA7)
  * @retval None
  */
#define __DAL_AFIO_REMAP_SPI1_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_SPI1_RMP)

/**
  * @brief Enable the remapping of I2C1 alternate function SCL and SDA.
  * @note  ENABLE: Remap     (SCL/PB8, SDA/PB9)
  * @retval None
  */
#define __DAL_AFIO_REMAP_I2C1_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_I2C1_RMP)

/**
  * @brief Disable the remapping of I2C1 alternate function SCL and SDA.
  * @note  DISABLE: No remap  (SCL/PB6, SDA/PB7)
  * @retval None
  */
#define __DAL_AFIO_REMAP_I2C1_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_I2C1_RMP)

/**
  * @brief Enable the remapping of USART1 alternate function TX and RX.
  * @note  ENABLE: Remap     (TX/PB6, RX/PB7)
  * @retval None
  */
#define __DAL_AFIO_REMAP_USART1_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_USART1_RMP)

/**
  * @brief Disable the remapping of USART1 alternate function TX and RX.
  * @note  DISABLE: No remap  (TX/PA9, RX/PA10)
  * @retval None
  */
#define __DAL_AFIO_REMAP_USART1_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_USART1_RMP)

/**
  * @brief Enable the remapping of USART3 alternate function CTS, RTS, CK, TX and RX.
  * @note  PARTIAL: Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14)
  * @retval None
  */
#define __DAL_AFIO_REMAP_USART3_PARTIAL() AFIO_REMAP_PARTIAL(AFIO_REMAP1_USART3_RMP_PARTIALREMAP, AFIO_REMAP1_USART3_RMP_FULLREMAP)

/**
  * @brief Disable the remapping of USART3 alternate function CTS, RTS, CK, TX and RX.
  * @note  DISABLE: No remap      (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14)
  * @retval None
  */
#define __DAL_AFIO_REMAP_USART3_DISABLE() AFIO_REMAP_PARTIAL(AFIO_REMAP1_USART3_RMP_NOREMAP, AFIO_REMAP1_USART3_RMP_FULLREMAP)

/**
  * @brief Enable the remapping of TMR1 alternate function channels 1 to 4, 1N to 3N, external trigger (ETR) and Break input (BKIN)
  * @note  PARTIAL: Partial remap (ETR/PA12, CH1/PA8, CH2/PA9,  CH3/PA10, CH4/PA11, BKIN/PA6,  CH1N/PA7,  CH2N/PB0,  CH3N/PB1)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR1_PARTIAL() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR1_RMP_PARTIALREMAP, AFIO_REMAP1_TMR1_RMP_FULLREMAP)

/**
  * @brief Disable the remapping of TMR1 alternate function channels 1 to 4, 1N to 3N, external trigger (ETR) and Break input (BKIN)
  * @note  DISABLE: No remap      (ETR/PA12, CH1/PA8, CH2/PA9,  CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR1_DISABLE() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR1_RMP_NOREMAP, AFIO_REMAP1_TMR1_RMP_FULLREMAP)

/**
  * @brief Enable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  ENABLE: Full remap       (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR2_ENABLE() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR2_RMP_FULLREMAP, AFIO_REMAP1_TMR2_RMP_FULLREMAP)

/**
  * @brief Enable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  PARTIAL_2: Partial remap (CH1/ETR/PA0,  CH2/PA1, CH3/PB10, CH4/PB11)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR2_PARTIAL_2() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR2_RMP_PARTIALREMAP2, AFIO_REMAP1_TMR2_RMP_FULLREMAP)

/**
  * @brief Enable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  PARTIAL_1: Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2,  CH4/PA3)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR2_PARTIAL_1() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR2_RMP_PARTIALREMAP1, AFIO_REMAP1_TMR2_RMP_FULLREMAP)

/**
  * @brief Disable the remapping of TMR2 alternate function channels 1 to 4 and external trigger (ETR)
  * @note  DISABLE: No remap        (CH1/ETR/PA0,  CH2/PA1, CH3/PA2,  CH4/PA3)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR2_DISABLE() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR2_RMP_NOREMAP, AFIO_REMAP1_TMR2_RMP_FULLREMAP)

/**
  * @brief Enable the remapping of TMR3 alternate function channels 1 to 4
  * @note  ENABLE: Full remap     (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR3_ENABLE() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR3_RMP_FULLREMAP, AFIO_REMAP1_TMR3_RMP_FULLREMAP)

/**
  * @brief Enable the remapping of TMR3 alternate function channels 1 to 4
  * @note  PARTIAL: Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR3_PARTIAL() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR3_RMP_PARTIALREMAP, AFIO_REMAP1_TMR3_RMP_FULLREMAP)

/**
  * @brief Disable the remapping of TMR3 alternate function channels 1 to 4
  * @note  DISABLE: No remap      (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1)
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR3_DISABLE() AFIO_REMAP_PARTIAL(AFIO_REMAP1_TMR3_RMP_NOREMAP, AFIO_REMAP1_TMR3_RMP_FULLREMAP)

/**
  * @brief Enable or disable the remapping of CAN alternate function CAN_RX and CAN_TX in devices with a single CAN interface.
  * @note  CASE 1: CAN_RX mapped to PA11, CAN_TX mapped to PA12
  * @retval None
  */
#define __DAL_AFIO_REMAP_CAN1_1() AFIO_REMAP_PARTIAL(AFIO_REMAP1_CAN1_RMP_REMAP1, AFIO_REMAP1_CAN1_RMP)

/**
  * @brief Enable or disable the remapping of CAN alternate function CAN_RX and CAN_TX in devices with a single CAN interface.
  * @note  CASE 2: CAN_RX mapped to PB8,  CAN_TX mapped to PB9 (not available on 36-pin package)
  * @retval None
  */
#define __DAL_AFIO_REMAP_CAN1_2() AFIO_REMAP_PARTIAL(AFIO_REMAP1_CAN1_RMP_REMAP2, AFIO_REMAP1_CAN1_RMP)

/**
  * @brief Enable or disable the remapping of CAN alternate function CAN_RX and CAN_TX in devices with a single CAN interface.
  * @note  CASE 3: CAN_RX mapped to PD0,  CAN_TX mapped to PD1
  * @retval None
  */
#define __DAL_AFIO_REMAP_CAN1_3() AFIO_REMAP_PARTIAL(AFIO_REMAP1_CAN1_RMP_REMAP3, AFIO_REMAP1_CAN1_RMP)

/**
  * @brief Enable the remapping of PD0 and PD1. When the HSE oscillator is not used
  *        (application running on internal 8 MHz RC) PD0 and PD1 can be mapped on OSC_IN and
  *        OSC_OUT. This is available only on 36, 48 and 64 pins packages (PD0 and PD1 are available
  *        on 100-pin and 144-pin packages, no need for remapping).
  * @note  ENABLE: PD0 remapped on OSC_IN, PD1 remapped on OSC_OUT.
  * @retval None
  */
#define __DAL_AFIO_REMAP_PD01_ENABLE() AFIO_REMAP_ENABLE(AFIO_REMAP1_PD01_RMP)

/**
  * @brief Disable the remapping of PD0 and PD1. When the HSE oscillator is not used
  *        (application running on internal 8 MHz RC) PD0 and PD1 can be mapped on OSC_IN and
  *        OSC_OUT. This is available only on 36, 48 and 64 pins packages (PD0 and PD1 are available
  *        on 100-pin and 144-pin packages, no need for remapping).
  * @note  DISABLE: No remapping of PD0 and PD1
  * @retval None
  */
#define __DAL_AFIO_REMAP_PD01_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_PD01_RMP)

/**
  * @brief Enable the remapping of TMR5CH4.
  * @note  ENABLE: LSI internal clock is connected to TMR5_CH4 input for calibration purpose.
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR5CH4_ENABLE() AFIO_REMAP_ENABLE(AFIO_REMAP1_TMR5CH4_IRMP)

/**
  * @brief Disable the remapping of TMR5CH4.
  * @note  DISABLE: TMR5_CH4 is connected to PA3
  * @retval None
  */
#define __DAL_AFIO_REMAP_TMR5CH4_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_TMR5CH4_IRMP)

/**
  * @brief Enable the remapping of CAN2 alternate function CAN2_RX and CAN2_TX.
  * @note  ENABLE: Remap     (CAN2_RX/PB5,  CAN2_TX/PB6)
  * @retval None
  */
#define __DAL_AFIO_REMAP_CAN2_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_CAN2_RMP)

/**
  * @brief Disable the remapping of CAN2 alternate function CAN2_RX and CAN2_TX.
  * @note  DISABLE: No remap (CAN2_RX/PB12, CAN2_TX/PB13)
  * @retval None
  */
#define __DAL_AFIO_REMAP_CAN2_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_CAN2_RMP)

/**
  * @brief Enable the remapping of ADC1_ETRGINJ (ADC 1 External trigger injected conversion).
  * @note  ENABLE: ADC1 External Event injected conversion is connected to TMR8 Channel4.
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC1_ETRGINJ_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_ADC1_ETRGINJC_RMP)

/**
  * @brief Disable the remapping of ADC1_ETRGINJ (ADC 1 External trigger injected conversion).
  * @note  DISABLE: ADC1 External trigger injected conversion is connected to EINT15
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC1_ETRGINJ_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_ADC1_ETRGINJC_RMP)

/**
  * @brief Enable the remapping of ADC1_ETRGREG (ADC 1 External trigger regular conversion).
  * @note  ENABLE: ADC1 External Event regular conversion is connected to TMR8 TRG0.
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_ADC1_ETRGREGC_RMP)

/**
  * @brief Disable the remapping of ADC1_ETRGREG (ADC 1 External trigger regular conversion).
  * @note  DISABLE: ADC1 External trigger regular conversion is connected to EINT11
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC1_ETRGREG_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_ADC1_ETRGREGC_RMP)

/**
  * @brief Enable the remapping of ADC2_ETRGREG (ADC 2 External trigger injected conversion).
  * @note  ENABLE: ADC2 External Event injected conversion is connected to TMR8 Channel4.
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC2_ETRGINJ_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_ADC2_ETRGINJC_RMP)

/**
  * @brief Disable the remapping of ADC2_ETRGREG (ADC 2 External trigger injected conversion).
  * @note  DISABLE: ADC2 External trigger injected conversion is connected to EINT15
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC2_ETRGINJ_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_ADC2_ETRGINJC_RMP)

/**
  * @brief Enable the remapping of ADC2_ETRGREG (ADC 2 External trigger regular conversion).
  * @note  ENABLE: ADC2 External Event regular conversion is connected to TMR8 TRG0.
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC2_ETRGREG_ENABLE()  AFIO_REMAP_ENABLE(AFIO_REMAP1_ADC2_ETRGREGC_RMP)

/**
  * @brief Disable the remapping of ADC2_ETRGREG (ADC 2 External trigger regular conversion).
  * @note  DISABLE: ADC2 External trigger regular conversion is connected to EINT11
  * @retval None
  */
#define __DAL_AFIO_REMAP_ADC2_ETRGREG_DISABLE() AFIO_REMAP_DISABLE(AFIO_REMAP1_ADC2_ETRGREGC_RMP)

/**
  * @brief Enable the Serial wire JTAG configuration
  * @note  ENABLE: Full SWJ (JTAG-DP + SW-DP): Reset State
  * @retval None
  */
#define __DAL_AFIO_REMAP_SWJ_ENABLE() AFIO_DBGAFR_CONFIG(AFIO_REMAP1_SWJ_CFG_RESET)

/**
  * @brief Enable the Serial wire JTAG configuration
  * @note  NONJTRST: Full SWJ (JTAG-DP + SW-DP) but without NJTRST
  * @retval None
  */
#define __DAL_AFIO_REMAP_SWJ_NONJTRST() AFIO_DBGAFR_CONFIG(AFIO_REMAP1_SWJ_CFG_NOJNTRST)

/**
  * @brief Enable the Serial wire JTAG configuration
  * @note  NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  * @retval None
  */
#define __DAL_AFIO_REMAP_SWJ_NOJTAG() AFIO_DBGAFR_CONFIG(AFIO_REMAP1_SWJ_CFG_JTAGDISABLE)

/**
  * @brief Disable the Serial wire JTAG configuration
  * @note  DISABLE: JTAG-DP Disabled and SW-DP Disabled
  * @retval None
  */
#define __DAL_AFIO_REMAP_SWJ_DISABLE() AFIO_DBGAFR_CONFIG(AFIO_REMAP1_SWJ_CFG_DISABLE)

/**
  * @}
  */
#endif /* APM32F403xx || APM32F402xx */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Macros GPIO Exported Macros
  * @{
  */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Functions GPIO Exported Functions
  * @{
  */
/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup GPIOEx_Private_Constants GPIO Private Constants
  * @{
  */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup GPIOEx_Private_Macros GPIO Private Macros
  * @{
  */
/** @defgroup GPIOEx_Get_Port_Index GPIO Get Port Index
  * @{
  */
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || defined(APM32F465xx) || \
    defined(APM32F423xx) || defined(APM32F427xx) || defined(APM32F425xx)
#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U :\
                                               ((__GPIOx__) == (GPIOE))? 4U :\
                                               ((__GPIOx__) == (GPIOF))? 5U :\
                                               ((__GPIOx__) == (GPIOG))? 6U :\
                                               ((__GPIOx__) == (GPIOH))? 7U : 8U)
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F423xx || APM32F427xx || APM32F425xx */

#if defined(APM32F411xx)
#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U :\
                                               ((__GPIOx__) == (GPIOE))? 4U : 7U)
#endif /* APM32F411xx */

#if defined(APM32F403xx) || defined(APM32F402xx)
#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U : 4U)
#endif /* APM32F403xx || APM32F402xx */

/**
  * @}
  */

#if defined(APM32F403xx) || defined(APM32F402xx)
/** @defgroup GPIOEx_Eventout_port_selection GPIO Check Eventout Port
  * @{
  */

#define AFIO_EVENTOUT_PORT_A   AFIO_EVCTRL_PORTSEL_PA    /*!< Eventout on port A */
#define AFIO_EVENTOUT_PORT_B   AFIO_EVCTRL_PORTSEL_PB    /*!< Eventout on port B */
#define AFIO_EVENTOUT_PORT_C   AFIO_EVCTRL_PORTSEL_PC    /*!< Eventout on port C */
#define AFIO_EVENTOUT_PORT_D   AFIO_EVCTRL_PORTSEL_PD    /*!< Eventout on port D */

#define IS_AFIO_EVENTOUT_PORT(__PORT__) (((__PORT__) == AFIO_EVENTOUT_PORT_A) || \
                                         ((__PORT__) == AFIO_EVENTOUT_PORT_B) || \
                                         ((__PORT__) == AFIO_EVENTOUT_PORT_C) || \
                                         ((__PORT__) == AFIO_EVENTOUT_PORT_D))

/**
  * @}
  */

/** @defgroup GPIOEx_Eventout_pin_selection GPIO Check Eventout Pin
  * @{
  */

#define AFIO_EVENTOUT_PIN_0   AFIO_EVCTRL_PINSEL_PX0    /*!< Eventout on pin 0 */
#define AFIO_EVENTOUT_PIN_1   AFIO_EVCTRL_PINSEL_PX1    /*!< Eventout on pin 1 */
#define AFIO_EVENTOUT_PIN_2   AFIO_EVCTRL_PINSEL_PX2    /*!< Eventout on pin 2 */
#define AFIO_EVENTOUT_PIN_3   AFIO_EVCTRL_PINSEL_PX3    /*!< Eventout on pin 3 */
#define AFIO_EVENTOUT_PIN_4   AFIO_EVCTRL_PINSEL_PX4    /*!< Eventout on pin 4 */
#define AFIO_EVENTOUT_PIN_5   AFIO_EVCTRL_PINSEL_PX5    /*!< Eventout on pin 5 */
#define AFIO_EVENTOUT_PIN_6   AFIO_EVCTRL_PINSEL_PX6    /*!< Eventout on pin 6 */
#define AFIO_EVENTOUT_PIN_7   AFIO_EVCTRL_PINSEL_PX7    /*!< Eventout on pin 7 */
#define AFIO_EVENTOUT_PIN_8   AFIO_EVCTRL_PINSEL_PX8    /*!< Eventout on pin 8 */
#define AFIO_EVENTOUT_PIN_9   AFIO_EVCTRL_PINSEL_PX9    /*!< Eventout on pin 9 */
#define AFIO_EVENTOUT_PIN_10  AFIO_EVCTRL_PINSEL_PX10   /*!< Eventout on pin 10 */
#define AFIO_EVENTOUT_PIN_11  AFIO_EVCTRL_PINSEL_PX11   /*!< Eventout on pin 11 */
#define AFIO_EVENTOUT_PIN_12  AFIO_EVCTRL_PINSEL_PX12   /*!< Eventout on pin 12 */
#define AFIO_EVENTOUT_PIN_13  AFIO_EVCTRL_PINSEL_PX13   /*!< Eventout on pin 13 */
#define AFIO_EVENTOUT_PIN_14  AFIO_EVCTRL_PINSEL_PX14   /*!< Eventout on pin 14 */
#define AFIO_EVENTOUT_PIN_15  AFIO_EVCTRL_PINSEL_PX15   /*!< Eventout on pin 15 */

#define IS_AFIO_EVENTOUT_PIN(__PIN__) (((__PIN__) == AFIO_EVENTOUT_PIN_0)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_1)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_2)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_3)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_4)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_5)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_6)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_7)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_8)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_9)  || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_10) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_11) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_12) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_13) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_14) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_15))

/**
  * @}
  */
#endif /* APM32F403xx || APM32F402xx */

/** @defgroup GPIOEx_IS_Alternat_function_selection GPIO Check Alternate Function
  * @{
  */

/*---------------------------------- APM32F407xx/APM32F417xx------------------*/
#if defined(APM32F407xx) || defined(APM32F417xx)
#define IS_GPIO_AF(AF)   (((AF) == GPIO_AF0_RTC_50Hz)   || ((AF) == GPIO_AF9_TMR14)      || \
                          ((AF) == GPIO_AF0_MCO)        || ((AF) == GPIO_AF0_TAMPER)     || \
                          ((AF) == GPIO_AF0_SWJ)        || ((AF) == GPIO_AF0_TRACE)      || \
                          ((AF) == GPIO_AF1_TMR1)       || ((AF) == GPIO_AF1_TMR2)       || \
                          ((AF) == GPIO_AF2_TMR3)       || ((AF) == GPIO_AF2_TMR4)       || \
                          ((AF) == GPIO_AF2_TMR5)       || ((AF) == GPIO_AF3_TMR8)       || \
                          ((AF) == GPIO_AF4_I2C1)       || ((AF) == GPIO_AF4_I2C2)       || \
                          ((AF) == GPIO_AF4_I2C3)       || ((AF) == GPIO_AF5_SPI1)       || \
                          ((AF) == GPIO_AF5_SPI2)       || ((AF) == GPIO_AF9_TMR13)      || \
                          ((AF) == GPIO_AF6_SPI3)       || ((AF) == GPIO_AF9_TMR12)      || \
                          ((AF) == GPIO_AF7_USART1)     || ((AF) == GPIO_AF7_USART2)     || \
                          ((AF) == GPIO_AF7_USART3)     || ((AF) == GPIO_AF8_UART4)      || \
                          ((AF) == GPIO_AF8_UART5)      || ((AF) == GPIO_AF8_USART6)     || \
                          ((AF) == GPIO_AF9_CAN1)       || ((AF) == GPIO_AF9_CAN2)       || \
                          ((AF) == GPIO_AF10_OTG_FS)    || ((AF) == GPIO_AF10_OTG_HS)    || \
                          ((AF) == GPIO_AF11_ETH)       || ((AF) == GPIO_AF12_OTG_HS_FS) || \
                          ((AF) == GPIO_AF12_SDIO)      || ((AF) == GPIO_AF13_DCI)       || \
                          ((AF) == GPIO_AF12_SMC)       || ((AF) == GPIO_AF12_DMC)       || \
                          ((AF) == GPIO_AF15_EVENTOUT))

#endif /* APM32F407xx || APM32F417xx */
/*----------------------------------------------------------------------------*/

/*---------------------------------- APM32F405xx/APM32F415xx------------------*/
#if defined(APM32F405xx) || defined(APM32F415xx)
#define IS_GPIO_AF(AF)   (((AF) == GPIO_AF0_RTC_50Hz)   || ((AF) == GPIO_AF9_TMR14)      || \
                          ((AF) == GPIO_AF0_MCO)        || ((AF) == GPIO_AF0_TAMPER)     || \
                          ((AF) == GPIO_AF0_SWJ)        || ((AF) == GPIO_AF0_TRACE)      || \
                          ((AF) == GPIO_AF1_TMR1)       || ((AF) == GPIO_AF1_TMR2)       || \
                          ((AF) == GPIO_AF2_TMR3)       || ((AF) == GPIO_AF2_TMR4)       || \
                          ((AF) == GPIO_AF2_TMR5)       || ((AF) == GPIO_AF3_TMR8)       || \
                          ((AF) == GPIO_AF4_I2C1)       || ((AF) == GPIO_AF4_I2C2)       || \
                          ((AF) == GPIO_AF4_I2C3)       || ((AF) == GPIO_AF5_SPI1)       || \
                          ((AF) == GPIO_AF5_SPI2)       || ((AF) == GPIO_AF9_TMR13)      || \
                          ((AF) == GPIO_AF6_SPI3)       || ((AF) == GPIO_AF9_TMR12)      || \
                          ((AF) == GPIO_AF7_USART1)     || ((AF) == GPIO_AF7_USART2)     || \
                          ((AF) == GPIO_AF7_USART3)     || ((AF) == GPIO_AF8_UART4)      || \
                          ((AF) == GPIO_AF8_UART5)      || ((AF) == GPIO_AF8_USART6)     || \
                          ((AF) == GPIO_AF9_CAN1)       || ((AF) == GPIO_AF9_CAN2)       || \
                          ((AF) == GPIO_AF10_OTG_FS)    || ((AF) == GPIO_AF10_OTG_HS)    || \
                          ((AF) == GPIO_AF12_OTG_HS_FS) || ((AF) == GPIO_AF12_SDIO)      || \
                          ((AF) == GPIO_AF12_SMC)       || ((AF) == GPIO_AF15_EVENTOUT))

#endif /* APM32F405xx || APM32F415xx */

/*----------------------------------------------------------------------------*/

/*---------------------------------- APM32F465xx------------------*/
#if defined(APM32F465xx)
#define IS_GPIO_AF(AF)   (((AF) == GPIO_AF0_RTC_50Hz)   || ((AF) == GPIO_AF9_TMR14)      || \
                          ((AF) == GPIO_AF0_MCO)        || ((AF) == GPIO_AF0_TAMPER)     || \
                          ((AF) == GPIO_AF0_SWJ)        || ((AF) == GPIO_AF0_TRACE)      || \
                          ((AF) == GPIO_AF1_TMR1)       || ((AF) == GPIO_AF1_TMR2)       || \
                          ((AF) == GPIO_AF2_TMR3)       || ((AF) == GPIO_AF2_TMR4)       || \
                          ((AF) == GPIO_AF2_TMR5)       || ((AF) == GPIO_AF3_TMR8)       || \
                          ((AF) == GPIO_AF4_I2C1)       || ((AF) == GPIO_AF4_I2C2)       || \
                          ((AF) == GPIO_AF4_I2C3)       || ((AF) == GPIO_AF5_SPI1)       || \
                          ((AF) == GPIO_AF5_SPI2)       || ((AF) == GPIO_AF9_TMR13)      || \
                          ((AF) == GPIO_AF6_SPI3)       || ((AF) == GPIO_AF9_TMR12)      || \
                          ((AF) == GPIO_AF7_USART1)     || ((AF) == GPIO_AF7_USART2)     || \
                          ((AF) == GPIO_AF7_USART3)     || ((AF) == GPIO_AF8_UART4)      || \
                          ((AF) == GPIO_AF8_UART5)      || ((AF) == GPIO_AF8_USART6)     || \
                          ((AF) == GPIO_AF9_CAN1)       || ((AF) == GPIO_AF9_CAN2)       || \
                          ((AF) == GPIO_AF10_OTG_FS)    || ((AF) == GPIO_AF10_OTG_HS)    || \
                          ((AF) == GPIO_AF12_OTG_HS_FS) || ((AF) == GPIO_AF12_SDIO)      || \
                          ((AF) == GPIO_AF12_SMC)       || ((AF) == GPIO_AF15_EVENTOUT))

#endif /* APM32F465xx */

/*----------------------------------------------------------------------------*/

/*---------------------------------------- APM32F411xx------------------------*/
#if defined(APM32F411xx)
#define IS_GPIO_AF(AF)   (((AF) == GPIO_AF0_RTC_50Hz)   || ((AF) == GPIO_AF11_SMC)       || \
                          ((AF) == GPIO_AF0_MCO)        || ((AF) == GPIO_AF0_TAMPER)     || \
                          ((AF) == GPIO_AF0_SWJ)        || ((AF) == GPIO_AF0_TRACE)      || \
                          ((AF) == GPIO_AF1_TMR1)       || ((AF) == GPIO_AF1_TMR2)       || \
                          ((AF) == GPIO_AF2_TMR3)       || ((AF) == GPIO_AF2_TMR4)       || \
                          ((AF) == GPIO_AF2_TMR5)       || ((AF) == GPIO_AF3_TMR8)       || \
                          ((AF) == GPIO_AF3_TMR9)       || ((AF) == GPIO_AF3_TMR10)      || \
                          ((AF) == GPIO_AF3_TMR11)      || ((AF) == GPIO_AF4_I2C1)       || \
                          ((AF) == GPIO_AF4_I2C2)       || ((AF) == GPIO_AF4_I2C3)       || \
                          ((AF) == GPIO_AF5_SPI1)       || ((AF) == GPIO_AF5_SPI2)       || \
                          ((AF) == GPIO_AF5_SPI3)       || ((AF) == GPIO_AF5_SPI4)       || \
                          ((AF) == GPIO_AF5_I2S3ext)    || ((AF) == GPIO_AF6_SPI2)       || \
                          ((AF) == GPIO_AF6_SPI3)       || ((AF) == GPIO_AF6_SPI4)       || \
                          ((AF) == GPIO_AF6_SPI5)       || ((AF) == GPIO_AF6_I2S2ext)    || \
                          ((AF) == GPIO_AF7_SPI3)       || ((AF) == GPIO_AF7_USART1)     || \
                          ((AF) == GPIO_AF7_USART2)     || ((AF) == GPIO_AF7_USART3)     || \
                          ((AF) == GPIO_AF7_I2S3ext)    || ((AF) == GPIO_AF8_USART3)     || \
                          ((AF) == GPIO_AF8_UART4)      || ((AF) == GPIO_AF8_UART5)      || \
                          ((AF) == GPIO_AF8_USART6)     || ((AF) == GPIO_AF8_CAN1)       || \
                          ((AF) == GPIO_AF9_CAN2)       || ((AF) == GPIO_AF9_TMR12)      || \
                          ((AF) == GPIO_AF9_TMR13)      || ((AF) == GPIO_AF9_TMR14)      || \
                          ((AF) == GPIO_AF9_I2C2)       || ((AF) == GPIO_AF9_I2C3)       || \
                          ((AF) == GPIO_AF9_QSPI)       || ((AF) == GPIO_AF10_OTG_FS)    || \
                          ((AF) == GPIO_AF10_QSPI)      || ((AF) == GPIO_AF10_SMC)       || \
                          ((AF) == GPIO_AF11_SDIO)      || ((AF) == GPIO_AF15_EVENTOUT))

#endif /* APM32F411xx */
/*----------------------------------------------------------------------------*/

/*---------------------------------- APM32F423xx/APM32F425xx/APM32F427xx------------------*/
#if defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define IS_GPIO_AF(AF)   (((AF) == GPIO_AF0_RTC_50Hz)   || ((AF) == GPIO_AF9_TMR14)      || \
                          ((AF) == GPIO_AF0_MCO)        || ((AF) == GPIO_AF0_TAMPER)     || \
                          ((AF) == GPIO_AF0_SWJ)        || ((AF) == GPIO_AF0_TRACE)      || \
                          ((AF) == GPIO_AF1_TMR1)       || ((AF) == GPIO_AF1_TMR2)       || \
                          ((AF) == GPIO_AF2_TMR3)       || ((AF) == GPIO_AF2_TMR4)       || \
                          ((AF) == GPIO_AF2_TMR5)       || ((AF) == GPIO_AF3_TMR8)       || \
                          ((AF) == GPIO_AF4_I2C1)       || ((AF) == GPIO_AF4_I2C2)       || \
                          ((AF) == GPIO_AF4_I2C3)       || ((AF) == GPIO_AF5_SPI1)       || \
                          ((AF) == GPIO_AF5_SPI2)       || ((AF) == GPIO_AF9_TMR13)      || \
                          ((AF) == GPIO_AF6_SPI3)       || ((AF) == GPIO_AF9_TMR12)      || \
                          ((AF) == GPIO_AF7_USART1)     || ((AF) == GPIO_AF7_USART2)     || \
                          ((AF) == GPIO_AF7_USART3)     || ((AF) == GPIO_AF8_UART4)      || \
                          ((AF) == GPIO_AF8_UART5)      || ((AF) == GPIO_AF8_USART6)     || \
                          ((AF) == GPIO_AF9_CAN1)       || ((AF) == GPIO_AF9_CAN2)       || \
                          ((AF) == GPIO_AF9_QSPI)       || ((AF) == GPIO_AF10_OTG_FS)    || \
                          ((AF) == GPIO_AF10_QSPI)      || ((AF) == GPIO_AF11_ETH)       || \
                          ((AF) == GPIO_AF12_OTG_FS2)   || ((AF) == GPIO_AF12_SDIO)      || \
                          ((AF) == GPIO_AF12_SMC)       || ((AF) == GPIO_AF12_DMC)       || \
                          ((AF) == GPIO_AF15_EVENTOUT))

#endif /* APM32F423xx || APM32F425xx || APM32F427xx */
/*----------------------------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup GPIOEx_Private_Functions GPIO Private Functions
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define AFIO_REMAP_ENABLE(REMAP_PIN)       do{ uint32_t tmpreg = AFIO->REMAP1; \
                                               tmpreg |= AFIO_REMAP1_SWJ_CFG;  \
                                               tmpreg |= REMAP_PIN;            \
                                               AFIO->REMAP1 = tmpreg;          \
                                               }while(0u)

#define AFIO_REMAP_DISABLE(REMAP_PIN)      do{ uint32_t tmpreg = AFIO->REMAP1;  \
                                               tmpreg |= AFIO_REMAP1_SWJ_CFG;   \
                                               tmpreg &= ~REMAP_PIN;            \
                                               AFIO->REMAP1 = tmpreg;           \
                                               }while(0u)

#define AFIO_REMAP_PARTIAL(REMAP_PIN, REMAP_PIN_MASK) do{ uint32_t tmpreg = AFIO->REMAP1; \
                                                          tmpreg &= ~REMAP_PIN_MASK;      \
                                                          tmpreg |= AFIO_REMAP1_SWJ_CFG;  \
                                                          tmpreg |= REMAP_PIN;            \
                                                          AFIO->REMAP1 = tmpreg;          \
                                                          }while(0u)

#define AFIO_DBGAFR_CONFIG(DBGAFR_SWJCFG)  do{ uint32_t tmpreg = AFIO->REMAP1;    \
                                               tmpreg &= ~AFIO_REMAP1_SWJ_CFG;    \
                                               tmpreg |= DBGAFR_SWJCFG;           \
                                               AFIO->REMAP1 = tmpreg;             \
                                               }while(0u)
#endif /* APM32F403xx || APM32F402xx */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup GPIOEx_Exported_Functions
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
/** @addtogroup GPIOEx_Exported_Functions_Group1
  * @{
  */
void DAL_GPIOEx_ConfigEventout(uint32_t GPIO_PortSource, uint32_t GPIO_PinSource);
void DAL_GPIOEx_EnableEventout(void);
void DAL_GPIOEx_DisableEventout(void);
#endif /* APM32F403xx || APM32F402xx */
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

#endif /* APM32F4xx_DAL_GPIO_EX_H */

