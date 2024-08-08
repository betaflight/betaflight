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
#define GPIO_AF13_DCI          ((uint8_t)0x0D)  /* DCI Alternate Function mapping */

/** 
  * @brief   AF 15 selection  
  */ 
#define GPIO_AF15_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */
#endif /* APM32F407xx || APM32F417xx */
/*----------------------------------------------------------------------------*/

/*---------------------------------- APM32F405xx------------------*/
#if defined(APM32F405xx)
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
#endif /* APM32F405xx */

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

/**
  * @}
  */ 

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
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U :\
                                               ((__GPIOx__) == (GPIOE))? 4U :\
                                               ((__GPIOx__) == (GPIOF))? 5U :\
                                               ((__GPIOx__) == (GPIOG))? 6U :\
                                               ((__GPIOx__) == (GPIOH))? 7U : 8U)
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

#if defined(APM32F411xx) 
#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U :\
                                               ((__GPIOx__) == (GPIOE))? 4U : 7U)
#endif /* APM32F411xx */

/**
  * @}
  */

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

/*---------------------------------- APM32F405xx------------------*/
#if defined(APM32F405xx)
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

#endif /* APM32F405xx */

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

