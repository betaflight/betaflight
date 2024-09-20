/**
  *
  * @file    apm32f4xx_ddl_rcm.h
  * @brief   Header file of RCM DDL module.
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
#ifndef APM32F4xx_DDL_RCM_H
#define APM32F4xx_DDL_RCM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(RCM)

/** @defgroup RCM_DDL RCM
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup RCM_DDL_Private_Variables RCM Private Variables
  * @{
  */

/**
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RCM_DDL_Private_Macros RCM Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RCM_DDL_Exported_Types RCM Exported Types
  * @{
  */

/** @defgroup DDL_ES_CLOCK_FREQ Clocks Frequency Structure
  * @{
  */

/**
  * @brief  RCM Clocks Frequency Structure
  */
typedef struct
{
  uint32_t SYSCLK_Frequency;        /*!< SYSCLK clock frequency */
  uint32_t HCLK_Frequency;          /*!< HCLK clock frequency */
  uint32_t PCLK1_Frequency;         /*!< PCLK1 clock frequency */
  uint32_t PCLK2_Frequency;         /*!< PCLK2 clock frequency */
} DDL_RCM_ClocksTypeDef;

/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCM_DDL_Exported_Constants RCM Exported Constants
  * @{
  */

/** @defgroup RCM_DDL_EC_OSC_VALUES Oscillator Values adaptation
  * @brief    Defines used to adapt values of different oscillators
  * @note     These values could be modified in the user environment according to 
  *           HW set-up.
  * @{
  */
#if !defined  (HSE_VALUE)
#define HSE_VALUE    25000000U  /*!< Value of the HSE oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    16000000U  /*!< Value of the HSI oscillator in Hz */
#endif /* HSI_VALUE */

#if !defined  (LSE_VALUE)
#define LSE_VALUE    32768U     /*!< Value of the LSE oscillator in Hz */
#endif /* LSE_VALUE */

#if !defined  (LSI_VALUE)
#define LSI_VALUE    32000U     /*!< Value of the LSI oscillator in Hz */
#endif /* LSI_VALUE */

#if !defined  (EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE    12288000U /*!< Value of the I2S_CKIN external oscillator in Hz */
#endif /* EXTERNAL_CLOCK_VALUE */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with DDL_RCM_WriteReg function
  * @{
  */
#define DDL_RCM_INT_LSIRDYCLR                RCM_INT_LSIRDYCLR     /*!< LSI Ready Interrupt Clear */
#define DDL_RCM_INT_LSERDYCLR                RCM_INT_LSERDYCLR     /*!< LSE Ready Interrupt Clear */
#define DDL_RCM_INT_HSIRDYCLR                RCM_INT_HSIRDYCLR     /*!< HSI Ready Interrupt Clear */
#define DDL_RCM_INT_HSERDYCLR                RCM_INT_HSERDYCLR     /*!< HSE Ready Interrupt Clear */
#define DDL_RCM_INT_PLL1RDYCLR               RCM_INT_PLL1RDYCLR    /*!< PLL Ready Interrupt Clear */
#if defined(RCM_PLLI2S_SUPPORT)
#define DDL_RCM_INT_PLL2RDYCLR               RCM_INT_PLL2RDYCLR    /*!< PLLI2S Ready Interrupt Clear */
#endif /* RCM_PLLI2S_SUPPORT */
#define DDL_RCM_INT_CSSCLR                   RCM_INT_CSSCLR        /*!< Clock Security System Interrupt Clear */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_RCM_ReadReg function
  * @{
  */
#define DDL_RCM_INT_LSIRDYFLG                RCM_INT_LSIRDYFLG     /*!< LSI Ready Interrupt flag */
#define DDL_RCM_INT_LSERDYFLG                RCM_INT_LSERDYFLG     /*!< LSE Ready Interrupt flag */
#define DDL_RCM_INT_HSIRDYFLG                RCM_INT_HSIRDYFLG     /*!< HSI Ready Interrupt flag */
#define DDL_RCM_INT_HSERDYFLG                RCM_INT_HSERDYFLG     /*!< HSE Ready Interrupt flag */
#define DDL_RCM_INT_PLL1RDYFLG               RCM_INT_PLL1RDYFLG    /*!< PLL Ready Interrupt flag */
#if defined(RCM_PLLI2S_SUPPORT)
#define DDL_RCM_INT_PLL2RDYFLG               RCM_INT_PLL2RDYFLG    /*!< PLLI2S Ready Interrupt flag */
#endif /* RCM_PLLI2S_SUPPORT */
#define DDL_RCM_INT_CSSFLG                   RCM_INT_CSSFLG        /*!< Clock Security System Interrupt flag */
#define DDL_RCM_CSTS_LPWRRSTFLG              RCM_CSTS_LPWRRSTFLG   /*!< Low-Power reset flag */
#define DDL_RCM_CSTS_PINRSTFLG               RCM_CSTS_PINRSTFLG    /*!< PIN reset flag */
#define DDL_RCM_CSTS_PODRSTFLG               RCM_CSTS_PODRSTFLG    /*!< POR/PDR reset flag */
#define DDL_RCM_CSTS_SWRSTFLG                RCM_CSTS_SWRSTFLG     /*!< Software Reset flag */
#define DDL_RCM_CSTS_IWDTRSTFLG              RCM_CSTS_IWDTRSTFLG   /*!< Independent Watchdog reset flag */
#define DDL_RCM_CSTS_WWDTRSTFLG              RCM_CSTS_WWDTRSTFLG   /*!< Window watchdog reset flag */
#if defined(RCM_CSTS_BORRSTFLG)
#define DDL_RCM_CSTS_BORRSTFLG               RCM_CSTS_BORRSTFLG    /*!< BOR reset flag */
#endif /* RCM_CSTS_BORRSTFLG */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_RCM_ReadReg and  DDL_RCM_WriteReg functions
  * @{
  */
#define DDL_RCM_INT_LSIRDYEN               RCM_INT_LSIRDYEN      /*!< LSI Ready Interrupt Enable */
#define DDL_RCM_INT_LSERDYEN               RCM_INT_LSERDYEN      /*!< LSE Ready Interrupt Enable */
#define DDL_RCM_INT_HSIRDYEN               RCM_INT_HSIRDYEN      /*!< HSI Ready Interrupt Enable */
#define DDL_RCM_INT_HSERDYEN               RCM_INT_HSERDYEN      /*!< HSE Ready Interrupt Enable */
#define DDL_RCM_INT_PLL1RDYEN              RCM_INT_PLL1RDYEN     /*!< PLL Ready Interrupt Enable */
#if defined(RCM_PLLI2S_SUPPORT)
#define DDL_RCM_INT_PLL2RDYEN              RCM_INT_PLL2RDYEN     /*!< PLLI2S Ready Interrupt Enable */
#endif /* RCM_PLLI2S_SUPPORT */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_SYS_CLKSOURCE  System clock switch
  * @{
  */
#define DDL_RCM_SYS_CLKSOURCE_HSI           RCM_CFG_SCLKSEL_HSI    /*!< HSI selection as system clock */
#define DDL_RCM_SYS_CLKSOURCE_HSE           RCM_CFG_SCLKSEL_HSE    /*!< HSE selection as system clock */
#define DDL_RCM_SYS_CLKSOURCE_PLL           RCM_CFG_SCLKSEL_PLL    /*!< PLL selection as system clock */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_SYS_CLKSOURCE_STATUS  System clock switch status
  * @{
  */
#define DDL_RCM_SYS_CLKSOURCE_STATUS_HSI    RCM_CFG_SCLKSWSTS_HSI   /*!< HSI used as system clock */
#define DDL_RCM_SYS_CLKSOURCE_STATUS_HSE    RCM_CFG_SCLKSWSTS_HSE   /*!< HSE used as system clock */
#define DDL_RCM_SYS_CLKSOURCE_STATUS_PLL    RCM_CFG_SCLKSWSTS_PLL   /*!< PLL used as system clock */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_SYSCLK_DIV  AHB prescaler
  * @{
  */
#define DDL_RCM_SYSCLK_DIV_1                RCM_CFG_AHBPSC_DIV1   /*!< SYSCLK not divided */
#define DDL_RCM_SYSCLK_DIV_2                RCM_CFG_AHBPSC_DIV2   /*!< SYSCLK divided by 2 */
#define DDL_RCM_SYSCLK_DIV_4                RCM_CFG_AHBPSC_DIV4   /*!< SYSCLK divided by 4 */
#define DDL_RCM_SYSCLK_DIV_8                RCM_CFG_AHBPSC_DIV8   /*!< SYSCLK divided by 8 */
#define DDL_RCM_SYSCLK_DIV_16               RCM_CFG_AHBPSC_DIV16  /*!< SYSCLK divided by 16 */
#define DDL_RCM_SYSCLK_DIV_64               RCM_CFG_AHBPSC_DIV64  /*!< SYSCLK divided by 64 */
#define DDL_RCM_SYSCLK_DIV_128              RCM_CFG_AHBPSC_DIV128 /*!< SYSCLK divided by 128 */
#define DDL_RCM_SYSCLK_DIV_256              RCM_CFG_AHBPSC_DIV256 /*!< SYSCLK divided by 256 */
#define DDL_RCM_SYSCLK_DIV_512              RCM_CFG_AHBPSC_DIV512 /*!< SYSCLK divided by 512 */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_APB1_DIV  APB low-speed prescaler (APB1)
  * @{
  */
#define DDL_RCM_APB1_DIV_1                  RCM_CFG_APB1PSC_DIV1  /*!< HCLK not divided */
#define DDL_RCM_APB1_DIV_2                  RCM_CFG_APB1PSC_DIV2  /*!< HCLK divided by 2 */
#define DDL_RCM_APB1_DIV_4                  RCM_CFG_APB1PSC_DIV4  /*!< HCLK divided by 4 */
#define DDL_RCM_APB1_DIV_8                  RCM_CFG_APB1PSC_DIV8  /*!< HCLK divided by 8 */
#define DDL_RCM_APB1_DIV_16                 RCM_CFG_APB1PSC_DIV16 /*!< HCLK divided by 16 */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_APB2_DIV  APB high-speed prescaler (APB2)
  * @{
  */
#define DDL_RCM_APB2_DIV_1                  RCM_CFG_APB2PSC_DIV1  /*!< HCLK not divided */
#define DDL_RCM_APB2_DIV_2                  RCM_CFG_APB2PSC_DIV2  /*!< HCLK divided by 2 */
#define DDL_RCM_APB2_DIV_4                  RCM_CFG_APB2PSC_DIV4  /*!< HCLK divided by 4 */
#define DDL_RCM_APB2_DIV_8                  RCM_CFG_APB2PSC_DIV8  /*!< HCLK divided by 8 */
#define DDL_RCM_APB2_DIV_16                 RCM_CFG_APB2PSC_DIV16 /*!< HCLK divided by 16 */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_MCOxSOURCE  MCO source selection
  * @{
  */
#define DDL_RCM_MCO1SOURCE_HSI              (uint32_t)(RCM_CFG_MCO1SEL|0x00000000U)                    /*!< HSI selection as MCO1 source */
#define DDL_RCM_MCO1SOURCE_LSE              (uint32_t)(RCM_CFG_MCO1SEL|(RCM_CFG_MCO1SEL_0 >> 16U))       /*!< LSE selection as MCO1 source */
#define DDL_RCM_MCO1SOURCE_HSE              (uint32_t)(RCM_CFG_MCO1SEL|(RCM_CFG_MCO1SEL_1 >> 16U))       /*!< HSE selection as MCO1 source */
#define DDL_RCM_MCO1SOURCE_PLLCLK           (uint32_t)(RCM_CFG_MCO1SEL|((RCM_CFG_MCO1SEL_1|RCM_CFG_MCO1SEL_0) >> 16U))       /*!< PLLCLK selection as MCO1 source */
#if defined(RCM_CFG_MCO2SEL)
#define DDL_RCM_MCO2SOURCE_SYSCLK           (uint32_t)(RCM_CFG_MCO2SEL|0x00000000U)                    /*!< SYSCLK selection as MCO2 source */
#define DDL_RCM_MCO2SOURCE_PLLI2S           (uint32_t)(RCM_CFG_MCO2SEL|(RCM_CFG_MCO2SEL_0 >> 16U))       /*!< PLLI2S selection as MCO2 source */
#define DDL_RCM_MCO2SOURCE_HSE              (uint32_t)(RCM_CFG_MCO2SEL|(RCM_CFG_MCO2SEL_1 >> 16U))       /*!< HSE selection as MCO2 source */
#define DDL_RCM_MCO2SOURCE_PLLCLK           (uint32_t)(RCM_CFG_MCO2SEL|((RCM_CFG_MCO2SEL_1|RCM_CFG_MCO2SEL_0) >> 16U))       /*!< PLLCLK selection as MCO2 source */
#endif /* RCM_CFG_MCO2SEL */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_MCOx_DIV  MCO prescaler
  * @{
  */
#define DDL_RCM_MCO1_DIV_1                  (uint32_t)(RCM_CFG_MCO1PSC|0x00000000U)                       /*!< MCO1 not divided */
#define DDL_RCM_MCO1_DIV_2                  (uint32_t)(RCM_CFG_MCO1PSC|(RCM_CFG_MCO1PSC_2 >> 16U))       /*!< MCO1 divided by 2 */
#define DDL_RCM_MCO1_DIV_3                  (uint32_t)(RCM_CFG_MCO1PSC|((RCM_CFG_MCO1PSC_2|RCM_CFG_MCO1PSC_0) >> 16U))       /*!< MCO1 divided by 3 */
#define DDL_RCM_MCO1_DIV_4                  (uint32_t)(RCM_CFG_MCO1PSC|((RCM_CFG_MCO1PSC_2|RCM_CFG_MCO1PSC_1) >> 16U))       /*!< MCO1 divided by 4 */
#define DDL_RCM_MCO1_DIV_5                  (uint32_t)(RCM_CFG_MCO1PSC|(RCM_CFG_MCO1PSC >> 16U))         /*!< MCO1 divided by 5 */
#if defined(RCM_CFG_MCO2PSC)
#define DDL_RCM_MCO2_DIV_1                  (uint32_t)(RCM_CFG_MCO2PSC|0x00000000U)                       /*!< MCO2 not divided */
#define DDL_RCM_MCO2_DIV_2                  (uint32_t)(RCM_CFG_MCO2PSC|(RCM_CFG_MCO2PSC_2 >> 16U))       /*!< MCO2 divided by 2 */
#define DDL_RCM_MCO2_DIV_3                  (uint32_t)(RCM_CFG_MCO2PSC|((RCM_CFG_MCO2PSC_2|RCM_CFG_MCO2PSC_0) >> 16U))       /*!< MCO2 divided by 3 */
#define DDL_RCM_MCO2_DIV_4                  (uint32_t)(RCM_CFG_MCO2PSC|((RCM_CFG_MCO2PSC_2|RCM_CFG_MCO2PSC_1) >> 16U))       /*!< MCO2 divided by 4 */
#define DDL_RCM_MCO2_DIV_5                  (uint32_t)(RCM_CFG_MCO2PSC|(RCM_CFG_MCO2PSC >> 16U))         /*!< MCO2 divided by 5 */
#endif /* RCM_CFG_MCO2PSC */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_RTC_HSEDIV  HSE prescaler for RTC clock
  * @{
  */
#define DDL_RCM_RTC_NOCLOCK                  0x00000000U             /*!< HSE not divided */
#define DDL_RCM_RTC_HSE_DIV_2                RCM_CFG_RTCPSC_1       /*!< HSE clock divided by 2 */
#define DDL_RCM_RTC_HSE_DIV_3                (RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 3 */
#define DDL_RCM_RTC_HSE_DIV_4                RCM_CFG_RTCPSC_2       /*!< HSE clock divided by 4 */
#define DDL_RCM_RTC_HSE_DIV_5                (RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 5 */
#define DDL_RCM_RTC_HSE_DIV_6                (RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1)       /*!< HSE clock divided by 6 */
#define DDL_RCM_RTC_HSE_DIV_7                (RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 7 */
#define DDL_RCM_RTC_HSE_DIV_8                RCM_CFG_RTCPSC_3       /*!< HSE clock divided by 8 */
#define DDL_RCM_RTC_HSE_DIV_9                (RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 9 */
#define DDL_RCM_RTC_HSE_DIV_10               (RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_1)       /*!< HSE clock divided by 10 */
#define DDL_RCM_RTC_HSE_DIV_11               (RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 11 */
#define DDL_RCM_RTC_HSE_DIV_12               (RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2)       /*!< HSE clock divided by 12 */
#define DDL_RCM_RTC_HSE_DIV_13               (RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 13 */
#define DDL_RCM_RTC_HSE_DIV_14               (RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1)       /*!< HSE clock divided by 14 */
#define DDL_RCM_RTC_HSE_DIV_15               (RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 15 */
#define DDL_RCM_RTC_HSE_DIV_16               RCM_CFG_RTCPSC_4       /*!< HSE clock divided by 16 */
#define DDL_RCM_RTC_HSE_DIV_17               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 17 */
#define DDL_RCM_RTC_HSE_DIV_18               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_1)       /*!< HSE clock divided by 18 */
#define DDL_RCM_RTC_HSE_DIV_19               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 19 */
#define DDL_RCM_RTC_HSE_DIV_20               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_2)       /*!< HSE clock divided by 20 */
#define DDL_RCM_RTC_HSE_DIV_21               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 21 */
#define DDL_RCM_RTC_HSE_DIV_22               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1)       /*!< HSE clock divided by 22 */
#define DDL_RCM_RTC_HSE_DIV_23               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 23 */
#define DDL_RCM_RTC_HSE_DIV_24               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3)       /*!< HSE clock divided by 24 */
#define DDL_RCM_RTC_HSE_DIV_25               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 25 */
#define DDL_RCM_RTC_HSE_DIV_26               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_1)       /*!< HSE clock divided by 26 */
#define DDL_RCM_RTC_HSE_DIV_27               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 27 */
#define DDL_RCM_RTC_HSE_DIV_28               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2)       /*!< HSE clock divided by 28 */
#define DDL_RCM_RTC_HSE_DIV_29               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 29 */
#define DDL_RCM_RTC_HSE_DIV_30               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1)       /*!< HSE clock divided by 30 */
#define DDL_RCM_RTC_HSE_DIV_31               (RCM_CFG_RTCPSC_4|RCM_CFG_RTCPSC_3|RCM_CFG_RTCPSC_2|RCM_CFG_RTCPSC_1|RCM_CFG_RTCPSC_0)       /*!< HSE clock divided by 31 */
/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RCM_DDL_EC_PERIPH_FREQUENCY Peripheral clock frequency
  * @{
  */
#define DDL_RCM_PERIPH_FREQUENCY_NO         0x00000000U                 /*!< No clock enabled for the peripheral            */
#define DDL_RCM_PERIPH_FREQUENCY_NA         0xFFFFFFFFU                 /*!< Frequency cannot be provided as external clock */
/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/** @defgroup RCM_DDL_EC_I2S1_CLKSOURCE  Peripheral I2S clock source selection
  * @{
  */
#if defined(RCM_CFG_I2SSEL)
#define DDL_RCM_I2S1_CLKSOURCE_PLLI2S     0x00000000U                /*!< I2S oscillator clock used as I2S1 clock */
#define DDL_RCM_I2S1_CLKSOURCE_PIN        RCM_CFG_I2SSEL            /*!< External pin clock used as I2S1 clock */
#endif /* RCM_CFG_I2SSEL */
/**
  * @}
  */

#if defined(SDIO)
/** @defgroup RCM_DDL_EC_SDIOx  Peripheral SDIO get clock source
  * @{
  */
#if defined(RCM_DCKCFGR_SDIOSEL)
#define DDL_RCM_SDIO_CLKSOURCE            RCM_DCKCFGR_SDIOSEL   /*!< SDIO Clock source selection */
#elif defined(RCM_DCKCFGR2_SDIOSEL)
#define DDL_RCM_SDIO_CLKSOURCE            RCM_DCKCFGR2_SDIOSEL  /*!< SDIO Clock source selection */
#else
#define DDL_RCM_SDIO_CLKSOURCE            RCM_PLL1CFG_PLLD      /*!< SDIO Clock source selection */
#endif
/**
  * @}
  */
#endif /* SDIO */

#if defined(RNG)
/** @defgroup RCM_DDL_EC_RNG  Peripheral RNG get clock source
  * @{
  */
#if defined(RCM_DCKCFGR_CK48MSEL) || defined(RCM_DCKCFGR2_CK48MSEL)
#define DDL_RCM_RNG_CLKSOURCE               DDL_RCM_CK48M_CLKSOURCE /*!< RNG Clock source selection */
#else
#define DDL_RCM_RNG_CLKSOURCE               RCM_PLL1CFG_PLLD       /*!< RNG Clock source selection */
#endif /* RCM_DCKCFGR_CK48MSEL || RCM_DCKCFGR2_CK48MSEL */
/**
  * @}
  */
#endif /* RNG */

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
/** @defgroup RCM_DDL_EC_USB  Peripheral USB get clock source
  * @{
  */
#if defined(RCM_DCKCFGR_CK48MSEL) || defined(RCM_DCKCFGR2_CK48MSEL)
#define DDL_RCM_USB_CLKSOURCE               DDL_RCM_CK48M_CLKSOURCE /*!< USB Clock source selection */
#else
#define DDL_RCM_USB_CLKSOURCE               RCM_PLL1CFG_PLLD       /*!< USB Clock source selection */
#endif /* RCM_DCKCFGR_CK48MSEL || RCM_DCKCFGR2_CK48MSEL */
/**
  * @}
  */
#endif /* USB_OTG_FS || USB_OTG_HS */


/** @defgroup RCM_DDL_EC_I2S1  Peripheral I2S get clock source
  * @{
  */
#if defined(RCM_CFG_I2SSEL)
#define DDL_RCM_I2S1_CLKSOURCE              RCM_CFG_I2SSEL     /*!< I2S1 Clock source selection */
#endif /* RCM_CFG_I2SSEL */
/**
  * @}
  */


/** @defgroup RCM_DDL_EC_RTC_CLKSOURCE  RTC clock source selection
  * @{
  */
#define DDL_RCM_RTC_CLKSOURCE_NONE          0x00000000U                  /*!< No clock used as RTC clock */
#define DDL_RCM_RTC_CLKSOURCE_LSE           RCM_BDCTRL_RTCSRCSEL_0       /*!< LSE oscillator clock used as RTC clock */
#define DDL_RCM_RTC_CLKSOURCE_LSI           RCM_BDCTRL_RTCSRCSEL_1       /*!< LSI oscillator clock used as RTC clock */
#define DDL_RCM_RTC_CLKSOURCE_HSE           RCM_BDCTRL_RTCSRCSEL         /*!< HSE oscillator clock divided by HSE prescaler used as RTC clock */
/**
  * @}
  */

#if defined(RCM_CFGSEL_CLKPSEL)
/** @defgroup RCM_DDL_EC_TMR_CLKPRESCALER  Timers clocks prescalers selection
  * @{
  */
#define DDL_RCM_TMR_PRESCALER_TWICE          0x00000000U                  /*!< Timers clock to twice PCLK */
#define DDL_RCM_TMR_PRESCALER_FOUR_TIMES     RCM_CFGSEL_CLKPSEL           /*!< Timers clock to four time PCLK */
/**
  * @}
  */
#endif /* RCM_CFGSEL_CLKPSEL */

/** @defgroup RCM_DDL_EC_PLLSOURCE  PLL, PLLI2S and PLLSAI entry clock source
  * @{
  */
#define DDL_RCM_PLLSOURCE_HSI               RCM_PLL1CFG_PLL1CLKS_HSI  /*!< HSI16 clock selected as PLL entry clock source */
#define DDL_RCM_PLLSOURCE_HSE               RCM_PLL1CFG_PLL1CLKS_HSE  /*!< HSE clock selected as PLL entry clock source */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_PLLB_DIV  PLL, PLLI2S and PLLSAI division factor
  * @{
  */
#define DDL_RCM_PLLB_DIV_2                  (RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 2 */
#define DDL_RCM_PLLB_DIV_3                  (RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 3 */
#define DDL_RCM_PLLB_DIV_4                  (RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 4 */
#define DDL_RCM_PLLB_DIV_5                  (RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 5 */
#define DDL_RCM_PLLB_DIV_6                  (RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 6 */
#define DDL_RCM_PLLB_DIV_7                  (RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 7 */
#define DDL_RCM_PLLB_DIV_8                  (RCM_PLL1CFG_PLLB_3) /*!< PLL, PLLI2S and PLLSAI division factor by 8 */
#define DDL_RCM_PLLB_DIV_9                  (RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 9 */
#define DDL_RCM_PLLB_DIV_10                 (RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 10 */
#define DDL_RCM_PLLB_DIV_11                 (RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 11 */
#define DDL_RCM_PLLB_DIV_12                 (RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 12 */
#define DDL_RCM_PLLB_DIV_13                 (RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 13 */
#define DDL_RCM_PLLB_DIV_14                 (RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 14 */
#define DDL_RCM_PLLB_DIV_15                 (RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 15 */
#define DDL_RCM_PLLB_DIV_16                 (RCM_PLL1CFG_PLLB_4) /*!< PLL, PLLI2S and PLLSAI division factor by 16 */
#define DDL_RCM_PLLB_DIV_17                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 17 */
#define DDL_RCM_PLLB_DIV_18                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 18 */
#define DDL_RCM_PLLB_DIV_19                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 19 */
#define DDL_RCM_PLLB_DIV_20                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 20 */
#define DDL_RCM_PLLB_DIV_21                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 21 */
#define DDL_RCM_PLLB_DIV_22                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 22 */
#define DDL_RCM_PLLB_DIV_23                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 23 */
#define DDL_RCM_PLLB_DIV_24                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3) /*!< PLL, PLLI2S and PLLSAI division factor by 24 */
#define DDL_RCM_PLLB_DIV_25                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 25 */
#define DDL_RCM_PLLB_DIV_26                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 26 */
#define DDL_RCM_PLLB_DIV_27                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 27 */
#define DDL_RCM_PLLB_DIV_28                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 28 */
#define DDL_RCM_PLLB_DIV_29                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 29 */
#define DDL_RCM_PLLB_DIV_30                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 30 */
#define DDL_RCM_PLLB_DIV_31                 (RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 31 */
#define DDL_RCM_PLLB_DIV_32                 (RCM_PLL1CFG_PLLB_5) /*!< PLL, PLLI2S and PLLSAI division factor by 32 */
#define DDL_RCM_PLLB_DIV_33                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 33 */
#define DDL_RCM_PLLB_DIV_34                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 34 */
#define DDL_RCM_PLLB_DIV_35                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 35 */
#define DDL_RCM_PLLB_DIV_36                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 36 */
#define DDL_RCM_PLLB_DIV_37                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 37 */
#define DDL_RCM_PLLB_DIV_38                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 38 */
#define DDL_RCM_PLLB_DIV_39                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 39 */
#define DDL_RCM_PLLB_DIV_40                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3) /*!< PLL, PLLI2S and PLLSAI division factor by 40 */
#define DDL_RCM_PLLB_DIV_41                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 41 */
#define DDL_RCM_PLLB_DIV_42                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 42 */
#define DDL_RCM_PLLB_DIV_43                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 43 */
#define DDL_RCM_PLLB_DIV_44                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 44 */
#define DDL_RCM_PLLB_DIV_45                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 45 */
#define DDL_RCM_PLLB_DIV_46                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 46 */
#define DDL_RCM_PLLB_DIV_47                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 47 */
#define DDL_RCM_PLLB_DIV_48                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4) /*!< PLL, PLLI2S and PLLSAI division factor by 48 */
#define DDL_RCM_PLLB_DIV_49                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 49 */
#define DDL_RCM_PLLB_DIV_50                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 50 */
#define DDL_RCM_PLLB_DIV_51                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 51 */
#define DDL_RCM_PLLB_DIV_52                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 52 */
#define DDL_RCM_PLLB_DIV_53                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 53 */
#define DDL_RCM_PLLB_DIV_54                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 54 */
#define DDL_RCM_PLLB_DIV_55                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 55 */
#define DDL_RCM_PLLB_DIV_56                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3) /*!< PLL, PLLI2S and PLLSAI division factor by 56 */
#define DDL_RCM_PLLB_DIV_57                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 57 */
#define DDL_RCM_PLLB_DIV_58                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 58 */
#define DDL_RCM_PLLB_DIV_59                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 59 */
#define DDL_RCM_PLLB_DIV_60                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2) /*!< PLL, PLLI2S and PLLSAI division factor by 60 */
#define DDL_RCM_PLLB_DIV_61                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 61 */
#define DDL_RCM_PLLB_DIV_62                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1) /*!< PLL, PLLI2S and PLLSAI division factor by 62 */
#define DDL_RCM_PLLB_DIV_63                 (RCM_PLL1CFG_PLLB_5 | RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLLB_3 | RCM_PLL1CFG_PLLB_2 | RCM_PLL1CFG_PLLB_1 | RCM_PLL1CFG_PLLB_0) /*!< PLL, PLLI2S and PLLSAI division factor by 63 */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_PLL1C_DIV  PLL division factor (PLL1C)
  * @{
  */
#define DDL_RCM_PLL1C_DIV_2                  0x00000000U            /*!< Main PLL division factor for PLL1C output by 2 */
#define DDL_RCM_PLL1C_DIV_4                  RCM_PLL1CFG_PLL1C_0     /*!< Main PLL division factor for PLL1C output by 4 */
#define DDL_RCM_PLL1C_DIV_6                  RCM_PLL1CFG_PLL1C_1     /*!< Main PLL division factor for PLL1C output by 6 */
#define DDL_RCM_PLL1C_DIV_8                  (RCM_PLL1CFG_PLL1C_1 | RCM_PLL1CFG_PLL1C_0)   /*!< Main PLL division factor for PLL1C output by 8 */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_PLLD_DIV  PLL division factor (PLLD)
  * @{
  */
#define DDL_RCM_PLLD_DIV_2                  RCM_PLL1CFG_PLLD_1                      /*!< Main PLL division factor for PLLD output by 2 */
#define DDL_RCM_PLLD_DIV_3                  (RCM_PLL1CFG_PLLD_1|RCM_PLL1CFG_PLLD_0) /*!< Main PLL division factor for PLLD output by 3 */
#define DDL_RCM_PLLD_DIV_4                  RCM_PLL1CFG_PLLD_2                      /*!< Main PLL division factor for PLLD output by 4 */
#define DDL_RCM_PLLD_DIV_5                  (RCM_PLL1CFG_PLLD_2|RCM_PLL1CFG_PLLD_0) /*!< Main PLL division factor for PLLD output by 5 */
#define DDL_RCM_PLLD_DIV_6                  (RCM_PLL1CFG_PLLD_2|RCM_PLL1CFG_PLLD_1) /*!< Main PLL division factor for PLLD output by 6 */
#define DDL_RCM_PLLD_DIV_7                  (RCM_PLL1CFG_PLLD_2|RCM_PLL1CFG_PLLD_1|RCM_PLL1CFG_PLLD_0) /*!< Main PLL division factor for PLLD output by 7 */
#define DDL_RCM_PLLD_DIV_8                  RCM_PLL1CFG_PLLD_3                      /*!< Main PLL division factor for PLLD output by 8 */
#define DDL_RCM_PLLD_DIV_9                  (RCM_PLL1CFG_PLLD_3|RCM_PLL1CFG_PLLD_0) /*!< Main PLL division factor for PLLD output by 9 */
#define DDL_RCM_PLLD_DIV_10                 (RCM_PLL1CFG_PLLD_3|RCM_PLL1CFG_PLLD_1) /*!< Main PLL division factor for PLLD output by 10 */
#define DDL_RCM_PLLD_DIV_11                 (RCM_PLL1CFG_PLLD_3|RCM_PLL1CFG_PLLD_1|RCM_PLL1CFG_PLLD_0) /*!< Main PLL division factor for PLLD output by 11 */
#define DDL_RCM_PLLD_DIV_12                 (RCM_PLL1CFG_PLLD_3|RCM_PLL1CFG_PLLD_2) /*!< Main PLL division factor for PLLD output by 12 */
#define DDL_RCM_PLLD_DIV_13                 (RCM_PLL1CFG_PLLD_3|RCM_PLL1CFG_PLLD_2|RCM_PLL1CFG_PLLD_0) /*!< Main PLL division factor for PLLD output by 13 */
#define DDL_RCM_PLLD_DIV_14                 (RCM_PLL1CFG_PLLD_3|RCM_PLL1CFG_PLLD_2|RCM_PLL1CFG_PLLD_1) /*!< Main PLL division factor for PLLD output by 14 */
#define DDL_RCM_PLLD_DIV_15                 (RCM_PLL1CFG_PLLD_3|RCM_PLL1CFG_PLLD_2|RCM_PLL1CFG_PLLD_1|RCM_PLL1CFG_PLLD_0) /*!< Main PLL division factor for PLLD output by 15 */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_PLL_SPRE_SEL  PLL Spread Spectrum Selection
  * @{
  */
#define DDL_RCM_SPREAD_SELECT_CENTER        0x00000000U                   /*!< PLL center spread spectrum selection */
#define DDL_RCM_SPREAD_SELECT_DOWN          RCM_SSCCFG_SSSEL           /*!< PLL down spread spectrum selection */
/**
  * @}
  */

#if defined(RCM_PLLI2S_SUPPORT)
/** @defgroup RCM_DDL_EC_PLL2B  PLL2B division factor (PLL2B)
  * @{
  */
#if defined(RCM_PLL2CFG_PLL2B)
#define DDL_RCM_PLL2B_DIV_2             (RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 2 */
#define DDL_RCM_PLL2B_DIV_3             (RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 3 */
#define DDL_RCM_PLL2B_DIV_4             (RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 4 */
#define DDL_RCM_PLL2B_DIV_5             (RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 5 */
#define DDL_RCM_PLL2B_DIV_6             (RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 6 */
#define DDL_RCM_PLL2B_DIV_7             (RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 7 */
#define DDL_RCM_PLL2B_DIV_8             (RCM_PLL2CFG_PLL2B_3) /*!< PLLI2S division factor for PLL2B output by 8 */
#define DDL_RCM_PLL2B_DIV_9             (RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 9 */
#define DDL_RCM_PLL2B_DIV_10            (RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 10 */
#define DDL_RCM_PLL2B_DIV_11            (RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 11 */
#define DDL_RCM_PLL2B_DIV_12            (RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 12 */
#define DDL_RCM_PLL2B_DIV_13            (RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 13 */
#define DDL_RCM_PLL2B_DIV_14            (RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 14 */
#define DDL_RCM_PLL2B_DIV_15            (RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 15 */
#define DDL_RCM_PLL2B_DIV_16            (RCM_PLL2CFG_PLL2B_4) /*!< PLLI2S division factor for PLL2B output by 16 */
#define DDL_RCM_PLL2B_DIV_17            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 17 */
#define DDL_RCM_PLL2B_DIV_18            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 18 */
#define DDL_RCM_PLL2B_DIV_19            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 19 */
#define DDL_RCM_PLL2B_DIV_20            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 20 */
#define DDL_RCM_PLL2B_DIV_21            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 21 */
#define DDL_RCM_PLL2B_DIV_22            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 22 */
#define DDL_RCM_PLL2B_DIV_23            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 23 */
#define DDL_RCM_PLL2B_DIV_24            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3) /*!< PLLI2S division factor for PLL2B output by 24 */
#define DDL_RCM_PLL2B_DIV_25            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 25 */
#define DDL_RCM_PLL2B_DIV_26            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 26 */
#define DDL_RCM_PLL2B_DIV_27            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 27 */
#define DDL_RCM_PLL2B_DIV_28            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 28 */
#define DDL_RCM_PLL2B_DIV_29            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 29 */
#define DDL_RCM_PLL2B_DIV_30            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 30 */
#define DDL_RCM_PLL2B_DIV_31            (RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 31 */
#define DDL_RCM_PLL2B_DIV_32            (RCM_PLL2CFG_PLL2B_5) /*!< PLLI2S division factor for PLL2B output by 32 */
#define DDL_RCM_PLL2B_DIV_33            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 33 */
#define DDL_RCM_PLL2B_DIV_34            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 34 */
#define DDL_RCM_PLL2B_DIV_35            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 35 */
#define DDL_RCM_PLL2B_DIV_36            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 36 */
#define DDL_RCM_PLL2B_DIV_37            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 37 */
#define DDL_RCM_PLL2B_DIV_38            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 38 */
#define DDL_RCM_PLL2B_DIV_39            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 39 */
#define DDL_RCM_PLL2B_DIV_40            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3) /*!< PLLI2S division factor for PLL2B output by 40 */
#define DDL_RCM_PLL2B_DIV_41            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 41 */
#define DDL_RCM_PLL2B_DIV_42            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 42 */
#define DDL_RCM_PLL2B_DIV_43            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 43 */
#define DDL_RCM_PLL2B_DIV_44            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 44 */
#define DDL_RCM_PLL2B_DIV_45            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 45 */
#define DDL_RCM_PLL2B_DIV_46            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 46 */
#define DDL_RCM_PLL2B_DIV_47            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 47 */
#define DDL_RCM_PLL2B_DIV_48            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4) /*!< PLLI2S division factor for PLL2B output by 48 */
#define DDL_RCM_PLL2B_DIV_49            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 49 */
#define DDL_RCM_PLL2B_DIV_50            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 50 */
#define DDL_RCM_PLL2B_DIV_51            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 51 */
#define DDL_RCM_PLL2B_DIV_52            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 52 */
#define DDL_RCM_PLL2B_DIV_53            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 53 */
#define DDL_RCM_PLL2B_DIV_54            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 54 */
#define DDL_RCM_PLL2B_DIV_55            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 55 */
#define DDL_RCM_PLL2B_DIV_56            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3) /*!< PLLI2S division factor for PLL2B output by 56 */
#define DDL_RCM_PLL2B_DIV_57            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 57 */
#define DDL_RCM_PLL2B_DIV_58            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 58 */
#define DDL_RCM_PLL2B_DIV_59            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 59 */
#define DDL_RCM_PLL2B_DIV_60            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2) /*!< PLLI2S division factor for PLL2B output by 60 */
#define DDL_RCM_PLL2B_DIV_61            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 61 */
#define DDL_RCM_PLL2B_DIV_62            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1) /*!< PLLI2S division factor for PLL2B output by 62 */
#define DDL_RCM_PLL2B_DIV_63            (RCM_PLL2CFG_PLL2B_5 | RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2B_3 | RCM_PLL2CFG_PLL2B_2 | RCM_PLL2CFG_PLL2B_1 | RCM_PLL2CFG_PLL2B_0) /*!< PLLI2S division factor for PLL2B output by 63 */
#else
#define DDL_RCM_PLL2B_DIV_2              DDL_RCM_PLLB_DIV_2      /*!< PLLI2S division factor for PLL2B output by 2 */
#define DDL_RCM_PLL2B_DIV_3              DDL_RCM_PLLB_DIV_3      /*!< PLLI2S division factor for PLL2B output by 3 */
#define DDL_RCM_PLL2B_DIV_4              DDL_RCM_PLLB_DIV_4      /*!< PLLI2S division factor for PLL2B output by 4 */
#define DDL_RCM_PLL2B_DIV_5              DDL_RCM_PLLB_DIV_5      /*!< PLLI2S division factor for PLL2B output by 5 */
#define DDL_RCM_PLL2B_DIV_6              DDL_RCM_PLLB_DIV_6      /*!< PLLI2S division factor for PLL2B output by 6 */
#define DDL_RCM_PLL2B_DIV_7              DDL_RCM_PLLB_DIV_7      /*!< PLLI2S division factor for PLL2B output by 7 */
#define DDL_RCM_PLL2B_DIV_8              DDL_RCM_PLLB_DIV_8      /*!< PLLI2S division factor for PLL2B output by 8 */
#define DDL_RCM_PLL2B_DIV_9              DDL_RCM_PLLB_DIV_9      /*!< PLLI2S division factor for PLL2B output by 9 */
#define DDL_RCM_PLL2B_DIV_10             DDL_RCM_PLLB_DIV_10     /*!< PLLI2S division factor for PLL2B output by 10 */
#define DDL_RCM_PLL2B_DIV_11             DDL_RCM_PLLB_DIV_11     /*!< PLLI2S division factor for PLL2B output by 11 */
#define DDL_RCM_PLL2B_DIV_12             DDL_RCM_PLLB_DIV_12     /*!< PLLI2S division factor for PLL2B output by 12 */
#define DDL_RCM_PLL2B_DIV_13             DDL_RCM_PLLB_DIV_13     /*!< PLLI2S division factor for PLL2B output by 13 */
#define DDL_RCM_PLL2B_DIV_14             DDL_RCM_PLLB_DIV_14     /*!< PLLI2S division factor for PLL2B output by 14 */
#define DDL_RCM_PLL2B_DIV_15             DDL_RCM_PLLB_DIV_15     /*!< PLLI2S division factor for PLL2B output by 15 */
#define DDL_RCM_PLL2B_DIV_16             DDL_RCM_PLLB_DIV_16     /*!< PLLI2S division factor for PLL2B output by 16 */
#define DDL_RCM_PLL2B_DIV_17             DDL_RCM_PLLB_DIV_17     /*!< PLLI2S division factor for PLL2B output by 17 */
#define DDL_RCM_PLL2B_DIV_18             DDL_RCM_PLLB_DIV_18     /*!< PLLI2S division factor for PLL2B output by 18 */
#define DDL_RCM_PLL2B_DIV_19             DDL_RCM_PLLB_DIV_19     /*!< PLLI2S division factor for PLL2B output by 19 */
#define DDL_RCM_PLL2B_DIV_20             DDL_RCM_PLLB_DIV_20     /*!< PLLI2S division factor for PLL2B output by 20 */
#define DDL_RCM_PLL2B_DIV_21             DDL_RCM_PLLB_DIV_21     /*!< PLLI2S division factor for PLL2B output by 21 */
#define DDL_RCM_PLL2B_DIV_22             DDL_RCM_PLLB_DIV_22     /*!< PLLI2S division factor for PLL2B output by 22 */
#define DDL_RCM_PLL2B_DIV_23             DDL_RCM_PLLB_DIV_23     /*!< PLLI2S division factor for PLL2B output by 23 */
#define DDL_RCM_PLL2B_DIV_24             DDL_RCM_PLLB_DIV_24     /*!< PLLI2S division factor for PLL2B output by 24 */
#define DDL_RCM_PLL2B_DIV_25             DDL_RCM_PLLB_DIV_25     /*!< PLLI2S division factor for PLL2B output by 25 */
#define DDL_RCM_PLL2B_DIV_26             DDL_RCM_PLLB_DIV_26     /*!< PLLI2S division factor for PLL2B output by 26 */
#define DDL_RCM_PLL2B_DIV_27             DDL_RCM_PLLB_DIV_27     /*!< PLLI2S division factor for PLL2B output by 27 */
#define DDL_RCM_PLL2B_DIV_28             DDL_RCM_PLLB_DIV_28     /*!< PLLI2S division factor for PLL2B output by 28 */
#define DDL_RCM_PLL2B_DIV_29             DDL_RCM_PLLB_DIV_29     /*!< PLLI2S division factor for PLL2B output by 29 */
#define DDL_RCM_PLL2B_DIV_30             DDL_RCM_PLLB_DIV_30     /*!< PLLI2S division factor for PLL2B output by 30 */
#define DDL_RCM_PLL2B_DIV_31             DDL_RCM_PLLB_DIV_31     /*!< PLLI2S division factor for PLL2B output by 31 */
#define DDL_RCM_PLL2B_DIV_32             DDL_RCM_PLLB_DIV_32     /*!< PLLI2S division factor for PLL2B output by 32 */
#define DDL_RCM_PLL2B_DIV_33             DDL_RCM_PLLB_DIV_33     /*!< PLLI2S division factor for PLL2B output by 33 */
#define DDL_RCM_PLL2B_DIV_34             DDL_RCM_PLLB_DIV_34     /*!< PLLI2S division factor for PLL2B output by 34 */
#define DDL_RCM_PLL2B_DIV_35             DDL_RCM_PLLB_DIV_35     /*!< PLLI2S division factor for PLL2B output by 35 */
#define DDL_RCM_PLL2B_DIV_36             DDL_RCM_PLLB_DIV_36     /*!< PLLI2S division factor for PLL2B output by 36 */
#define DDL_RCM_PLL2B_DIV_37             DDL_RCM_PLLB_DIV_37     /*!< PLLI2S division factor for PLL2B output by 37 */
#define DDL_RCM_PLL2B_DIV_38             DDL_RCM_PLLB_DIV_38     /*!< PLLI2S division factor for PLL2B output by 38 */
#define DDL_RCM_PLL2B_DIV_39             DDL_RCM_PLLB_DIV_39     /*!< PLLI2S division factor for PLL2B output by 39 */
#define DDL_RCM_PLL2B_DIV_40             DDL_RCM_PLLB_DIV_40     /*!< PLLI2S division factor for PLL2B output by 40 */
#define DDL_RCM_PLL2B_DIV_41             DDL_RCM_PLLB_DIV_41     /*!< PLLI2S division factor for PLL2B output by 41 */
#define DDL_RCM_PLL2B_DIV_42             DDL_RCM_PLLB_DIV_42     /*!< PLLI2S division factor for PLL2B output by 42 */
#define DDL_RCM_PLL2B_DIV_43             DDL_RCM_PLLB_DIV_43     /*!< PLLI2S division factor for PLL2B output by 43 */
#define DDL_RCM_PLL2B_DIV_44             DDL_RCM_PLLB_DIV_44     /*!< PLLI2S division factor for PLL2B output by 44 */
#define DDL_RCM_PLL2B_DIV_45             DDL_RCM_PLLB_DIV_45     /*!< PLLI2S division factor for PLL2B output by 45 */
#define DDL_RCM_PLL2B_DIV_46             DDL_RCM_PLLB_DIV_46     /*!< PLLI2S division factor for PLL2B output by 46 */
#define DDL_RCM_PLL2B_DIV_47             DDL_RCM_PLLB_DIV_47     /*!< PLLI2S division factor for PLL2B output by 47 */
#define DDL_RCM_PLL2B_DIV_48             DDL_RCM_PLLB_DIV_48     /*!< PLLI2S division factor for PLL2B output by 48 */
#define DDL_RCM_PLL2B_DIV_49             DDL_RCM_PLLB_DIV_49     /*!< PLLI2S division factor for PLL2B output by 49 */
#define DDL_RCM_PLL2B_DIV_50             DDL_RCM_PLLB_DIV_50     /*!< PLLI2S division factor for PLL2B output by 50 */
#define DDL_RCM_PLL2B_DIV_51             DDL_RCM_PLLB_DIV_51     /*!< PLLI2S division factor for PLL2B output by 51 */
#define DDL_RCM_PLL2B_DIV_52             DDL_RCM_PLLB_DIV_52     /*!< PLLI2S division factor for PLL2B output by 52 */
#define DDL_RCM_PLL2B_DIV_53             DDL_RCM_PLLB_DIV_53     /*!< PLLI2S division factor for PLL2B output by 53 */
#define DDL_RCM_PLL2B_DIV_54             DDL_RCM_PLLB_DIV_54     /*!< PLLI2S division factor for PLL2B output by 54 */
#define DDL_RCM_PLL2B_DIV_55             DDL_RCM_PLLB_DIV_55     /*!< PLLI2S division factor for PLL2B output by 55 */
#define DDL_RCM_PLL2B_DIV_56             DDL_RCM_PLLB_DIV_56     /*!< PLLI2S division factor for PLL2B output by 56 */
#define DDL_RCM_PLL2B_DIV_57             DDL_RCM_PLLB_DIV_57     /*!< PLLI2S division factor for PLL2B output by 57 */
#define DDL_RCM_PLL2B_DIV_58             DDL_RCM_PLLB_DIV_58     /*!< PLLI2S division factor for PLL2B output by 58 */
#define DDL_RCM_PLL2B_DIV_59             DDL_RCM_PLLB_DIV_59     /*!< PLLI2S division factor for PLL2B output by 59 */
#define DDL_RCM_PLL2B_DIV_60             DDL_RCM_PLLB_DIV_60     /*!< PLLI2S division factor for PLL2B output by 60 */
#define DDL_RCM_PLL2B_DIV_61             DDL_RCM_PLLB_DIV_61     /*!< PLLI2S division factor for PLL2B output by 61 */
#define DDL_RCM_PLL2B_DIV_62             DDL_RCM_PLLB_DIV_62     /*!< PLLI2S division factor for PLL2B output by 62 */
#define DDL_RCM_PLL2B_DIV_63             DDL_RCM_PLLB_DIV_63     /*!< PLLI2S division factor for PLL2B output by 63 */
#endif /* RCM_PLL2CFG_PLL2B */
/**
  * @}
  */

/** @defgroup RCM_DDL_EC_PLL2C  PLL2C division factor (PLL2C)
  * @{
  */
#define DDL_RCM_PLL2C_DIV_2              RCM_PLL2CFG_PLL2C_1                                     /*!< PLLI2S division factor for PLL2C output by 2 */
#define DDL_RCM_PLL2C_DIV_3              (RCM_PLL2CFG_PLL2C_1 | RCM_PLL2CFG_PLL2C_0)        /*!< PLLI2S division factor for PLL2C output by 3 */
#define DDL_RCM_PLL2C_DIV_4              RCM_PLL2CFG_PLL2C_2                                     /*!< PLLI2S division factor for PLL2C output by 4 */
#define DDL_RCM_PLL2C_DIV_5              (RCM_PLL2CFG_PLL2C_2 | RCM_PLL2CFG_PLL2C_0)        /*!< PLLI2S division factor for PLL2C output by 5 */
#define DDL_RCM_PLL2C_DIV_6              (RCM_PLL2CFG_PLL2C_2 | RCM_PLL2CFG_PLL2C_1)        /*!< PLLI2S division factor for PLL2C output by 6 */
#define DDL_RCM_PLL2C_DIV_7              (RCM_PLL2CFG_PLL2C_2 | RCM_PLL2CFG_PLL2C_1 | RCM_PLL2CFG_PLL2C_0)        /*!< PLLI2S division factor for PLL2C output by 7 */
/**
  * @}
  */

#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCM_DDL_Exported_Macros RCM Exported Macros
  * @{
  */

/** @defgroup RCM_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RCM register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_RCM_WriteReg(__REG__, __VALUE__) WRITE_REG(RCM->__REG__, (__VALUE__))

/**
  * @brief  Read a value in RCM register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_RCM_ReadReg(__REG__) READ_REG(RCM->__REG__)
/**
  * @}
  */

/** @defgroup RCM_DDL_EM_CALC_FREQ Calculate frequencies
  * @{
  */

/**
  * @brief  Helper macro to calculate the PLLCLK frequency on system domain
  * @note ex: @ref __DDL_RCM_CALC_PLLCLK_FREQ (HSE_VALUE,@ref DDL_RCM_PLL_GetDivider (),
  *             @ref DDL_RCM_PLL_GetN (), @ref DDL_RCM_PLL_GetP ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLB__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLB_DIV_2
  *         @arg @ref DDL_RCM_PLLB_DIV_3
  *         @arg @ref DDL_RCM_PLLB_DIV_4
  *         @arg @ref DDL_RCM_PLLB_DIV_5
  *         @arg @ref DDL_RCM_PLLB_DIV_6
  *         @arg @ref DDL_RCM_PLLB_DIV_7
  *         @arg @ref DDL_RCM_PLLB_DIV_8
  *         @arg @ref DDL_RCM_PLLB_DIV_9
  *         @arg @ref DDL_RCM_PLLB_DIV_10
  *         @arg @ref DDL_RCM_PLLB_DIV_11
  *         @arg @ref DDL_RCM_PLLB_DIV_12
  *         @arg @ref DDL_RCM_PLLB_DIV_13
  *         @arg @ref DDL_RCM_PLLB_DIV_14
  *         @arg @ref DDL_RCM_PLLB_DIV_15
  *         @arg @ref DDL_RCM_PLLB_DIV_16
  *         @arg @ref DDL_RCM_PLLB_DIV_17
  *         @arg @ref DDL_RCM_PLLB_DIV_18
  *         @arg @ref DDL_RCM_PLLB_DIV_19
  *         @arg @ref DDL_RCM_PLLB_DIV_20
  *         @arg @ref DDL_RCM_PLLB_DIV_21
  *         @arg @ref DDL_RCM_PLLB_DIV_22
  *         @arg @ref DDL_RCM_PLLB_DIV_23
  *         @arg @ref DDL_RCM_PLLB_DIV_24
  *         @arg @ref DDL_RCM_PLLB_DIV_25
  *         @arg @ref DDL_RCM_PLLB_DIV_26
  *         @arg @ref DDL_RCM_PLLB_DIV_27
  *         @arg @ref DDL_RCM_PLLB_DIV_28
  *         @arg @ref DDL_RCM_PLLB_DIV_29
  *         @arg @ref DDL_RCM_PLLB_DIV_30
  *         @arg @ref DDL_RCM_PLLB_DIV_31
  *         @arg @ref DDL_RCM_PLLB_DIV_32
  *         @arg @ref DDL_RCM_PLLB_DIV_33
  *         @arg @ref DDL_RCM_PLLB_DIV_34
  *         @arg @ref DDL_RCM_PLLB_DIV_35
  *         @arg @ref DDL_RCM_PLLB_DIV_36
  *         @arg @ref DDL_RCM_PLLB_DIV_37
  *         @arg @ref DDL_RCM_PLLB_DIV_38
  *         @arg @ref DDL_RCM_PLLB_DIV_39
  *         @arg @ref DDL_RCM_PLLB_DIV_40
  *         @arg @ref DDL_RCM_PLLB_DIV_41
  *         @arg @ref DDL_RCM_PLLB_DIV_42
  *         @arg @ref DDL_RCM_PLLB_DIV_43
  *         @arg @ref DDL_RCM_PLLB_DIV_44
  *         @arg @ref DDL_RCM_PLLB_DIV_45
  *         @arg @ref DDL_RCM_PLLB_DIV_46
  *         @arg @ref DDL_RCM_PLLB_DIV_47
  *         @arg @ref DDL_RCM_PLLB_DIV_48
  *         @arg @ref DDL_RCM_PLLB_DIV_49
  *         @arg @ref DDL_RCM_PLLB_DIV_50
  *         @arg @ref DDL_RCM_PLLB_DIV_51
  *         @arg @ref DDL_RCM_PLLB_DIV_52
  *         @arg @ref DDL_RCM_PLLB_DIV_53
  *         @arg @ref DDL_RCM_PLLB_DIV_54
  *         @arg @ref DDL_RCM_PLLB_DIV_55
  *         @arg @ref DDL_RCM_PLLB_DIV_56
  *         @arg @ref DDL_RCM_PLLB_DIV_57
  *         @arg @ref DDL_RCM_PLLB_DIV_58
  *         @arg @ref DDL_RCM_PLLB_DIV_59
  *         @arg @ref DDL_RCM_PLLB_DIV_60
  *         @arg @ref DDL_RCM_PLLB_DIV_61
  *         @arg @ref DDL_RCM_PLLB_DIV_62
  *         @arg @ref DDL_RCM_PLLB_DIV_63
  * @param  __PLL1A__ Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  * @param  __PLL1C__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLL1C_DIV_2
  *         @arg @ref DDL_RCM_PLL1C_DIV_4
  *         @arg @ref DDL_RCM_PLL1C_DIV_6
  *         @arg @ref DDL_RCM_PLL1C_DIV_8
  * @retval PLL clock frequency (in Hz)
  */
#define __DDL_RCM_CALC_PLLCLK_FREQ(__INPUTFREQ__, __PLLB__, __PLL1A__, __PLL1C__) ((__INPUTFREQ__) / (__PLLB__) * (__PLL1A__) / \
                   ((((__PLL1C__) >> RCM_PLL1CFG_PLL1C_Pos ) + 1U) * 2U))

/**
  * @brief  Helper macro to calculate the PLLCLK frequency used on 48M domain
  * @note ex: @ref __DDL_RCM_CALC_PLLCLK_48M_FREQ (HSE_VALUE,@ref DDL_RCM_PLL_GetDivider (),
  *             @ref DDL_RCM_PLL_GetN (), @ref DDL_RCM_PLL_GetQ ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLB__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLB_DIV_2
  *         @arg @ref DDL_RCM_PLLB_DIV_3
  *         @arg @ref DDL_RCM_PLLB_DIV_4
  *         @arg @ref DDL_RCM_PLLB_DIV_5
  *         @arg @ref DDL_RCM_PLLB_DIV_6
  *         @arg @ref DDL_RCM_PLLB_DIV_7
  *         @arg @ref DDL_RCM_PLLB_DIV_8
  *         @arg @ref DDL_RCM_PLLB_DIV_9
  *         @arg @ref DDL_RCM_PLLB_DIV_10
  *         @arg @ref DDL_RCM_PLLB_DIV_11
  *         @arg @ref DDL_RCM_PLLB_DIV_12
  *         @arg @ref DDL_RCM_PLLB_DIV_13
  *         @arg @ref DDL_RCM_PLLB_DIV_14
  *         @arg @ref DDL_RCM_PLLB_DIV_15
  *         @arg @ref DDL_RCM_PLLB_DIV_16
  *         @arg @ref DDL_RCM_PLLB_DIV_17
  *         @arg @ref DDL_RCM_PLLB_DIV_18
  *         @arg @ref DDL_RCM_PLLB_DIV_19
  *         @arg @ref DDL_RCM_PLLB_DIV_20
  *         @arg @ref DDL_RCM_PLLB_DIV_21
  *         @arg @ref DDL_RCM_PLLB_DIV_22
  *         @arg @ref DDL_RCM_PLLB_DIV_23
  *         @arg @ref DDL_RCM_PLLB_DIV_24
  *         @arg @ref DDL_RCM_PLLB_DIV_25
  *         @arg @ref DDL_RCM_PLLB_DIV_26
  *         @arg @ref DDL_RCM_PLLB_DIV_27
  *         @arg @ref DDL_RCM_PLLB_DIV_28
  *         @arg @ref DDL_RCM_PLLB_DIV_29
  *         @arg @ref DDL_RCM_PLLB_DIV_30
  *         @arg @ref DDL_RCM_PLLB_DIV_31
  *         @arg @ref DDL_RCM_PLLB_DIV_32
  *         @arg @ref DDL_RCM_PLLB_DIV_33
  *         @arg @ref DDL_RCM_PLLB_DIV_34
  *         @arg @ref DDL_RCM_PLLB_DIV_35
  *         @arg @ref DDL_RCM_PLLB_DIV_36
  *         @arg @ref DDL_RCM_PLLB_DIV_37
  *         @arg @ref DDL_RCM_PLLB_DIV_38
  *         @arg @ref DDL_RCM_PLLB_DIV_39
  *         @arg @ref DDL_RCM_PLLB_DIV_40
  *         @arg @ref DDL_RCM_PLLB_DIV_41
  *         @arg @ref DDL_RCM_PLLB_DIV_42
  *         @arg @ref DDL_RCM_PLLB_DIV_43
  *         @arg @ref DDL_RCM_PLLB_DIV_44
  *         @arg @ref DDL_RCM_PLLB_DIV_45
  *         @arg @ref DDL_RCM_PLLB_DIV_46
  *         @arg @ref DDL_RCM_PLLB_DIV_47
  *         @arg @ref DDL_RCM_PLLB_DIV_48
  *         @arg @ref DDL_RCM_PLLB_DIV_49
  *         @arg @ref DDL_RCM_PLLB_DIV_50
  *         @arg @ref DDL_RCM_PLLB_DIV_51
  *         @arg @ref DDL_RCM_PLLB_DIV_52
  *         @arg @ref DDL_RCM_PLLB_DIV_53
  *         @arg @ref DDL_RCM_PLLB_DIV_54
  *         @arg @ref DDL_RCM_PLLB_DIV_55
  *         @arg @ref DDL_RCM_PLLB_DIV_56
  *         @arg @ref DDL_RCM_PLLB_DIV_57
  *         @arg @ref DDL_RCM_PLLB_DIV_58
  *         @arg @ref DDL_RCM_PLLB_DIV_59
  *         @arg @ref DDL_RCM_PLLB_DIV_60
  *         @arg @ref DDL_RCM_PLLB_DIV_61
  *         @arg @ref DDL_RCM_PLLB_DIV_62
  *         @arg @ref DDL_RCM_PLLB_DIV_63
  * @param  __PLL1A__ Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  * @param  __PLLD__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLD_DIV_2
  *         @arg @ref DDL_RCM_PLLD_DIV_3
  *         @arg @ref DDL_RCM_PLLD_DIV_4
  *         @arg @ref DDL_RCM_PLLD_DIV_5
  *         @arg @ref DDL_RCM_PLLD_DIV_6
  *         @arg @ref DDL_RCM_PLLD_DIV_7
  *         @arg @ref DDL_RCM_PLLD_DIV_8
  *         @arg @ref DDL_RCM_PLLD_DIV_9
  *         @arg @ref DDL_RCM_PLLD_DIV_10
  *         @arg @ref DDL_RCM_PLLD_DIV_11
  *         @arg @ref DDL_RCM_PLLD_DIV_12
  *         @arg @ref DDL_RCM_PLLD_DIV_13
  *         @arg @ref DDL_RCM_PLLD_DIV_14
  *         @arg @ref DDL_RCM_PLLD_DIV_15
  * @retval PLL clock frequency (in Hz)
  */
#define __DDL_RCM_CALC_PLLCLK_48M_FREQ(__INPUTFREQ__, __PLLB__, __PLL1A__, __PLLD__) ((__INPUTFREQ__) / (__PLLB__) * (__PLL1A__) / \
                   ((__PLLD__) >> RCM_PLL1CFG_PLLD_Pos ))


#if defined(RCM_PLLI2S_SUPPORT)
/**
  * @brief  Helper macro to calculate the PLLI2S frequency used for I2S domain
  * @note ex: @ref __DDL_RCM_CALC_PLLI2S_I2S_FREQ (HSE_VALUE,@ref DDL_RCM_PLLI2S_GetDivider (),
  *             @ref DDL_RCM_PLLI2S_GetN (), @ref DDL_RCM_PLLI2S_GetR ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLB__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLL2B_DIV_2
  *         @arg @ref DDL_RCM_PLL2B_DIV_3
  *         @arg @ref DDL_RCM_PLL2B_DIV_4
  *         @arg @ref DDL_RCM_PLL2B_DIV_5
  *         @arg @ref DDL_RCM_PLL2B_DIV_6
  *         @arg @ref DDL_RCM_PLL2B_DIV_7
  *         @arg @ref DDL_RCM_PLL2B_DIV_8
  *         @arg @ref DDL_RCM_PLL2B_DIV_9
  *         @arg @ref DDL_RCM_PLL2B_DIV_10
  *         @arg @ref DDL_RCM_PLL2B_DIV_11
  *         @arg @ref DDL_RCM_PLL2B_DIV_12
  *         @arg @ref DDL_RCM_PLL2B_DIV_13
  *         @arg @ref DDL_RCM_PLL2B_DIV_14
  *         @arg @ref DDL_RCM_PLL2B_DIV_15
  *         @arg @ref DDL_RCM_PLL2B_DIV_16
  *         @arg @ref DDL_RCM_PLL2B_DIV_17
  *         @arg @ref DDL_RCM_PLL2B_DIV_18
  *         @arg @ref DDL_RCM_PLL2B_DIV_19
  *         @arg @ref DDL_RCM_PLL2B_DIV_20
  *         @arg @ref DDL_RCM_PLL2B_DIV_21
  *         @arg @ref DDL_RCM_PLL2B_DIV_22
  *         @arg @ref DDL_RCM_PLL2B_DIV_23
  *         @arg @ref DDL_RCM_PLL2B_DIV_24
  *         @arg @ref DDL_RCM_PLL2B_DIV_25
  *         @arg @ref DDL_RCM_PLL2B_DIV_26
  *         @arg @ref DDL_RCM_PLL2B_DIV_27
  *         @arg @ref DDL_RCM_PLL2B_DIV_28
  *         @arg @ref DDL_RCM_PLL2B_DIV_29
  *         @arg @ref DDL_RCM_PLL2B_DIV_30
  *         @arg @ref DDL_RCM_PLL2B_DIV_31
  *         @arg @ref DDL_RCM_PLL2B_DIV_32
  *         @arg @ref DDL_RCM_PLL2B_DIV_33
  *         @arg @ref DDL_RCM_PLL2B_DIV_34
  *         @arg @ref DDL_RCM_PLL2B_DIV_35
  *         @arg @ref DDL_RCM_PLL2B_DIV_36
  *         @arg @ref DDL_RCM_PLL2B_DIV_37
  *         @arg @ref DDL_RCM_PLL2B_DIV_38
  *         @arg @ref DDL_RCM_PLL2B_DIV_39
  *         @arg @ref DDL_RCM_PLL2B_DIV_40
  *         @arg @ref DDL_RCM_PLL2B_DIV_41
  *         @arg @ref DDL_RCM_PLL2B_DIV_42
  *         @arg @ref DDL_RCM_PLL2B_DIV_43
  *         @arg @ref DDL_RCM_PLL2B_DIV_44
  *         @arg @ref DDL_RCM_PLL2B_DIV_45
  *         @arg @ref DDL_RCM_PLL2B_DIV_46
  *         @arg @ref DDL_RCM_PLL2B_DIV_47
  *         @arg @ref DDL_RCM_PLL2B_DIV_48
  *         @arg @ref DDL_RCM_PLL2B_DIV_49
  *         @arg @ref DDL_RCM_PLL2B_DIV_50
  *         @arg @ref DDL_RCM_PLL2B_DIV_51
  *         @arg @ref DDL_RCM_PLL2B_DIV_52
  *         @arg @ref DDL_RCM_PLL2B_DIV_53
  *         @arg @ref DDL_RCM_PLL2B_DIV_54
  *         @arg @ref DDL_RCM_PLL2B_DIV_55
  *         @arg @ref DDL_RCM_PLL2B_DIV_56
  *         @arg @ref DDL_RCM_PLL2B_DIV_57
  *         @arg @ref DDL_RCM_PLL2B_DIV_58
  *         @arg @ref DDL_RCM_PLL2B_DIV_59
  *         @arg @ref DDL_RCM_PLL2B_DIV_60
  *         @arg @ref DDL_RCM_PLL2B_DIV_61
  *         @arg @ref DDL_RCM_PLL2B_DIV_62
  *         @arg @ref DDL_RCM_PLL2B_DIV_63
  * @param  __PLL2A__ Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  * @param  __PLL2C__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLL2C_DIV_2
  *         @arg @ref DDL_RCM_PLL2C_DIV_3
  *         @arg @ref DDL_RCM_PLL2C_DIV_4
  *         @arg @ref DDL_RCM_PLL2C_DIV_5
  *         @arg @ref DDL_RCM_PLL2C_DIV_6
  *         @arg @ref DDL_RCM_PLL2C_DIV_7
  * @retval PLLI2S clock frequency (in Hz)
  */
#define __DDL_RCM_CALC_PLLI2S_I2S_FREQ(__INPUTFREQ__, __PLLB__, __PLL2A__, __PLL2C__) (((__INPUTFREQ__) / (__PLLB__)) * (__PLL2A__) / \
                   ((__PLL2C__) >> RCM_PLL2CFG_PLL2C_Pos))
#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @brief  Helper macro to calculate the HCLK frequency
  * @param  __SYSCLKFREQ__ SYSCLK frequency (based on HSE/HSI/PLLCLK)
  * @param  __AHBPRESCALER__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_SYSCLK_DIV_1
  *         @arg @ref DDL_RCM_SYSCLK_DIV_2
  *         @arg @ref DDL_RCM_SYSCLK_DIV_4
  *         @arg @ref DDL_RCM_SYSCLK_DIV_8
  *         @arg @ref DDL_RCM_SYSCLK_DIV_16
  *         @arg @ref DDL_RCM_SYSCLK_DIV_64
  *         @arg @ref DDL_RCM_SYSCLK_DIV_128
  *         @arg @ref DDL_RCM_SYSCLK_DIV_256
  *         @arg @ref DDL_RCM_SYSCLK_DIV_512
  * @retval HCLK clock frequency (in Hz)
  */
#define __DDL_RCM_CALC_HCLK_FREQ(__SYSCLKFREQ__, __AHBPRESCALER__) ((__SYSCLKFREQ__) >> AHBPrescTable[((__AHBPRESCALER__) & RCM_CFG_AHBPSC) >>  RCM_CFG_AHBPSC_Pos])

/**
  * @brief  Helper macro to calculate the PCLK1 frequency (ABP1)
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APB1PRESCALER__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_APB1_DIV_1
  *         @arg @ref DDL_RCM_APB1_DIV_2
  *         @arg @ref DDL_RCM_APB1_DIV_4
  *         @arg @ref DDL_RCM_APB1_DIV_8
  *         @arg @ref DDL_RCM_APB1_DIV_16
  * @retval PCLK1 clock frequency (in Hz)
  */
#define __DDL_RCM_CALC_PCLK1_FREQ(__HCLKFREQ__, __APB1PRESCALER__) ((__HCLKFREQ__) >> APBPrescTable[(__APB1PRESCALER__) >>  RCM_CFG_APB1PSC_Pos])

/**
  * @brief  Helper macro to calculate the PCLK2 frequency (ABP2)
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APB2PRESCALER__ This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_APB2_DIV_1
  *         @arg @ref DDL_RCM_APB2_DIV_2
  *         @arg @ref DDL_RCM_APB2_DIV_4
  *         @arg @ref DDL_RCM_APB2_DIV_8
  *         @arg @ref DDL_RCM_APB2_DIV_16
  * @retval PCLK2 clock frequency (in Hz)
  */
#define __DDL_RCM_CALC_PCLK2_FREQ(__HCLKFREQ__, __APB2PRESCALER__) ((__HCLKFREQ__) >> APBPrescTable[(__APB2PRESCALER__) >>  RCM_CFG_APB2PSC_Pos])

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RCM_DDL_Exported_Functions RCM Exported Functions
  * @{
  */

/** @defgroup RCM_DDL_EF_HSE HSE
  * @{
  */

/**
  * @brief  Enable the Clock Security System.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSE_EnableCSS(void)
{
  SET_BIT(RCM->CTRL, RCM_CTRL_CSSEN);
}

/**
  * @brief  Enable HSE external oscillator (HSE Bypass)
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSE_EnableBypass(void)
{
  SET_BIT(RCM->CTRL, RCM_CTRL_HSEBCFG);
}

/**
  * @brief  Disable HSE external oscillator (HSE Bypass)
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSE_DisableBypass(void)
{
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_HSEBCFG);
}

/**
  * @brief  Enable HSE crystal oscillator (HSE ON)
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSE_Enable(void)
{
  SET_BIT(RCM->CTRL, RCM_CTRL_HSEEN);
}

/**
  * @brief  Disable HSE crystal oscillator (HSE ON)
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSE_Disable(void)
{
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_HSEEN);
}

/**
  * @brief  Check if HSE oscillator Ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_HSE_IsReady(void)
{
  return (READ_BIT(RCM->CTRL, RCM_CTRL_HSERDYFLG) == (RCM_CTRL_HSERDYFLG));
}

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_HSI HSI
  * @{
  */

/**
  * @brief  Enable HSI oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSI_Enable(void)
{
  SET_BIT(RCM->CTRL, RCM_CTRL_HSIEN);
}

/**
  * @brief  Disable HSI oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSI_Disable(void)
{
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_HSIEN);
}

/**
  * @brief  Check if HSI clock is ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_HSI_IsReady(void)
{
  return (READ_BIT(RCM->CTRL, RCM_CTRL_HSIRDYFLG) == (RCM_CTRL_HSIRDYFLG));
}

/**
  * @brief  Get HSI Calibration value
  * @note When HSITRIM is written, HSICAL is updated with the sum of
  *       HSITRIM and the factory trim value
  * @retval Between Min_Data = 0x00 and Max_Data = 0xFF
  */
__STATIC_INLINE uint32_t DDL_RCM_HSI_GetCalibration(void)
{
  return (uint32_t)(READ_BIT(RCM->CTRL, RCM_CTRL_HSICAL) >> RCM_CTRL_HSICAL_Pos);
}

/**
  * @brief  Set HSI Calibration trimming
  * @note user-programmable trimming value that is added to the HSICAL
  * @note Default value is 16, which, when added to the HSICAL value,
  *       should trim the HSI to 16 MHz +/- 1 %
  * @param  Value Between Min_Data = 0 and Max_Data = 31
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_HSI_SetCalibTrimming(uint32_t Value)
{
  MODIFY_REG(RCM->CTRL, RCM_CTRL_HSITRM, Value << RCM_CTRL_HSITRM_Pos);
}

/**
  * @brief  Get HSI Calibration trimming
  * @retval Between Min_Data = 0 and Max_Data = 31
  */
__STATIC_INLINE uint32_t DDL_RCM_HSI_GetCalibTrimming(void)
{
  return (uint32_t)(READ_BIT(RCM->CTRL, RCM_CTRL_HSITRM) >> RCM_CTRL_HSITRM_Pos);
}

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_LSE LSE
  * @{
  */

/**
  * @brief  Enable  Low Speed External (LSE) crystal.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSE_Enable(void)
{
  SET_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEEN);
}

/**
  * @brief  Disable  Low Speed External (LSE) crystal.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSE_Disable(void)
{
  CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEEN);
}

/**
  * @brief  Enable external clock source (LSE bypass).
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSE_EnableBypass(void)
{
  SET_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEBCFG);
}

/**
  * @brief  Disable external clock source (LSE bypass).
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSE_DisableBypass(void)
{
  CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEBCFG);
}

/**
  * @brief  Check if LSE oscillator Ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_LSE_IsReady(void)
{
  return (READ_BIT(RCM->BDCTRL, RCM_BDCTRL_LSERDYFLG) == (RCM_BDCTRL_LSERDYFLG));
}

#if defined(RCM_BDCTRL_LSEMOD)
/**
  * @brief  Enable LSE high drive mode.
  * @note LSE high drive mode can be enabled only when the LSE clock is disabled
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSE_EnableHighDriveMode(void)
{
  SET_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEMOD);
}

/**
  * @brief  Disable LSE high drive mode.
  * @note LSE high drive mode can be disabled only when the LSE clock is disabled
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSE_DisableHighDriveMode(void)
{
  CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEMOD);
}
#endif /* RCM_BDCTRL_LSEMOD */

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_LSI LSI
  * @{
  */

/**
  * @brief  Enable LSI Oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSI_Enable(void)
{
  SET_BIT(RCM->CSTS, RCM_CSTS_LSIEN);
}

/**
  * @brief  Disable LSI Oscillator
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_LSI_Disable(void)
{
  CLEAR_BIT(RCM->CSTS, RCM_CSTS_LSIEN);
}

/**
  * @brief  Check if LSI is Ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_LSI_IsReady(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_LSIRDYFLG) == (RCM_CSTS_LSIRDYFLG));
}

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_System System
  * @{
  */

/**
  * @brief  Configure the system clock source
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_HSI
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_HSE
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_PLL
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_PLLR (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetSysClkSource(uint32_t Source)
{
  MODIFY_REG(RCM->CFG, RCM_CFG_SCLKSEL, Source);
}

/**
  * @brief  Get the system clock source
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_STATUS_HSI
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_STATUS_HSE
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_STATUS_PLL
  *         @arg @ref DDL_RCM_SYS_CLKSOURCE_STATUS_PLLR (*)
  *
  *         (*) value not defined in all devices.
  */
__STATIC_INLINE uint32_t DDL_RCM_GetSysClkSource(void)
{
  return (uint32_t)(READ_BIT(RCM->CFG, RCM_CFG_SCLKSWSTS));
}

/**
  * @brief  Set AHB prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_SYSCLK_DIV_1
  *         @arg @ref DDL_RCM_SYSCLK_DIV_2
  *         @arg @ref DDL_RCM_SYSCLK_DIV_4
  *         @arg @ref DDL_RCM_SYSCLK_DIV_8
  *         @arg @ref DDL_RCM_SYSCLK_DIV_16
  *         @arg @ref DDL_RCM_SYSCLK_DIV_64
  *         @arg @ref DDL_RCM_SYSCLK_DIV_128
  *         @arg @ref DDL_RCM_SYSCLK_DIV_256
  *         @arg @ref DDL_RCM_SYSCLK_DIV_512
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetAHBPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCM->CFG, RCM_CFG_AHBPSC, Prescaler);
}

/**
  * @brief  Set APB1 prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_APB1_DIV_1
  *         @arg @ref DDL_RCM_APB1_DIV_2
  *         @arg @ref DDL_RCM_APB1_DIV_4
  *         @arg @ref DDL_RCM_APB1_DIV_8
  *         @arg @ref DDL_RCM_APB1_DIV_16
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetAPB1Prescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCM->CFG, RCM_CFG_APB1PSC, Prescaler);
}

/**
  * @brief  Set APB2 prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_APB2_DIV_1
  *         @arg @ref DDL_RCM_APB2_DIV_2
  *         @arg @ref DDL_RCM_APB2_DIV_4
  *         @arg @ref DDL_RCM_APB2_DIV_8
  *         @arg @ref DDL_RCM_APB2_DIV_16
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetAPB2Prescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCM->CFG, RCM_CFG_APB2PSC, Prescaler);
}

/**
  * @brief  Get AHB prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_SYSCLK_DIV_1
  *         @arg @ref DDL_RCM_SYSCLK_DIV_2
  *         @arg @ref DDL_RCM_SYSCLK_DIV_4
  *         @arg @ref DDL_RCM_SYSCLK_DIV_8
  *         @arg @ref DDL_RCM_SYSCLK_DIV_16
  *         @arg @ref DDL_RCM_SYSCLK_DIV_64
  *         @arg @ref DDL_RCM_SYSCLK_DIV_128
  *         @arg @ref DDL_RCM_SYSCLK_DIV_256
  *         @arg @ref DDL_RCM_SYSCLK_DIV_512
  */
__STATIC_INLINE uint32_t DDL_RCM_GetAHBPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCM->CFG, RCM_CFG_AHBPSC));
}

/**
  * @brief  Get APB1 prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_APB1_DIV_1
  *         @arg @ref DDL_RCM_APB1_DIV_2
  *         @arg @ref DDL_RCM_APB1_DIV_4
  *         @arg @ref DDL_RCM_APB1_DIV_8
  *         @arg @ref DDL_RCM_APB1_DIV_16
  */
__STATIC_INLINE uint32_t DDL_RCM_GetAPB1Prescaler(void)
{
  return (uint32_t)(READ_BIT(RCM->CFG, RCM_CFG_APB1PSC));
}

/**
  * @brief  Get APB2 prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_APB2_DIV_1
  *         @arg @ref DDL_RCM_APB2_DIV_2
  *         @arg @ref DDL_RCM_APB2_DIV_4
  *         @arg @ref DDL_RCM_APB2_DIV_8
  *         @arg @ref DDL_RCM_APB2_DIV_16
  */
__STATIC_INLINE uint32_t DDL_RCM_GetAPB2Prescaler(void)
{
  return (uint32_t)(READ_BIT(RCM->CFG, RCM_CFG_APB2PSC));
}

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_MCO MCO
  * @{
  */

#if defined(RCM_CFG_MCO1EN)
/**
  * @brief  Enable MCO1 output
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_MCO1_Enable(void)
{
  SET_BIT(RCM->CFG, RCM_CFG_MCO1EN);
}

/**
  * @brief  Disable MCO1 output
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_MCO1_Disable(void)
{
  CLEAR_BIT(RCM->CFG, RCM_CFG_MCO1EN);
}
#endif /* RCM_CFG_MCO1EN */

#if defined(RCM_CFG_MCO2EN)
/**
  * @brief  Enable MCO2 output
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_MCO2_Enable(void)
{
  SET_BIT(RCM->CFG, RCM_CFG_MCO2EN);
}

/**
  * @brief  Disable MCO2 output
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_MCO2_Disable(void)
{
  CLEAR_BIT(RCM->CFG, RCM_CFG_MCO2EN);
}
#endif /* RCM_CFG_MCO2EN */

/**
  * @brief  Configure MCOx
  * @param  MCOxSource This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_MCO1SOURCE_HSI
  *         @arg @ref DDL_RCM_MCO1SOURCE_LSE
  *         @arg @ref DDL_RCM_MCO1SOURCE_HSE
  *         @arg @ref DDL_RCM_MCO1SOURCE_PLLCLK
  *         @arg @ref DDL_RCM_MCO2SOURCE_SYSCLK
  *         @arg @ref DDL_RCM_MCO2SOURCE_PLLI2S
  *         @arg @ref DDL_RCM_MCO2SOURCE_HSE
  *         @arg @ref DDL_RCM_MCO2SOURCE_PLLCLK
  * @param  MCOxPrescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_MCO1_DIV_1
  *         @arg @ref DDL_RCM_MCO1_DIV_2
  *         @arg @ref DDL_RCM_MCO1_DIV_3
  *         @arg @ref DDL_RCM_MCO1_DIV_4
  *         @arg @ref DDL_RCM_MCO1_DIV_5
  *         @arg @ref DDL_RCM_MCO2_DIV_1
  *         @arg @ref DDL_RCM_MCO2_DIV_2
  *         @arg @ref DDL_RCM_MCO2_DIV_3
  *         @arg @ref DDL_RCM_MCO2_DIV_4
  *         @arg @ref DDL_RCM_MCO2_DIV_5
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ConfigMCO(uint32_t MCOxSource, uint32_t MCOxPrescaler)
{
  MODIFY_REG(RCM->CFG, (MCOxSource & 0xFFFF0000U) | (MCOxPrescaler & 0xFFFF0000U),  (MCOxSource << 16U) | (MCOxPrescaler << 16U));
}

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_Peripheral_Clock_Source Peripheral Clock Source
  * @{
  */

/**
  * @brief  Configure I2S clock source
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PLLI2S (*)
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PIN
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PLL (*)
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PLLSRC (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PLLI2S (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PIN (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PLL (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PLLSRC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetI2SClockSource(uint32_t Source)
{
#if defined(RCM_CFG_I2SSEL)
  MODIFY_REG(RCM->CFG, RCM_CFG_I2SSEL, Source);
#else
  MODIFY_REG(RCM->DCKCFGR, (Source & 0xFFFF0000U), (Source << 16U));
#endif /* RCM_CFG_I2SSEL */
}

/**
  * @brief  Get I2S Clock Source
  * @param  I2Sx This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE (*)
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PLLI2S (*)
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PIN
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PLL (*)
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE_PLLSRC (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PLLI2S (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PIN (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PLL (*)
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE_PLLSRC (*)
  *
  *         (*) value not defined in all devices.
  */
__STATIC_INLINE uint32_t DDL_RCM_GetI2SClockSource(uint32_t I2Sx)
{
#if defined(RCM_CFG_I2SSEL)
  return (uint32_t)(READ_BIT(RCM->CFG, I2Sx));
#else
  return (uint32_t)(READ_BIT(RCM->DCKCFGR, I2Sx) >> 16U | I2Sx);
#endif /* RCM_CFG_I2SSEL */
}

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_RTC RTC
  * @{
  */

/**
  * @brief  Set RTC Clock Source
  * @note Once the RTC clock source has been selected, it cannot be changed anymore unless
  *       the Backup domain is reset, or unless a failure is detected on LSE (LSECSSD is
  *       set). The BDRST bit can be used to reset them.
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_NONE
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_LSE
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_LSI
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_HSE
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetRTCClockSource(uint32_t Source)
{
  MODIFY_REG(RCM->BDCTRL, RCM_BDCTRL_RTCSRCSEL, Source);
}

/**
  * @brief  Get RTC Clock Source
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_NONE
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_LSE
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_LSI
  *         @arg @ref DDL_RCM_RTC_CLKSOURCE_HSE
  */
__STATIC_INLINE uint32_t DDL_RCM_GetRTCClockSource(void)
{
  return (uint32_t)(READ_BIT(RCM->BDCTRL, RCM_BDCTRL_RTCSRCSEL));
}

/**
  * @brief  Enable RTC
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_EnableRTC(void)
{
  SET_BIT(RCM->BDCTRL, RCM_BDCTRL_RTCCLKEN);
}

/**
  * @brief  Disable RTC
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_DisableRTC(void)
{
  CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_RTCCLKEN);
}

/**
  * @brief  Check if RTC has been enabled or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsEnabledRTC(void)
{
  return (READ_BIT(RCM->BDCTRL, RCM_BDCTRL_RTCCLKEN) == (RCM_BDCTRL_RTCCLKEN));
}

/**
  * @brief  Force the Backup domain reset
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ForceBackupDomainReset(void)
{
  SET_BIT(RCM->BDCTRL, RCM_BDCTRL_BDRST);
}

/**
  * @brief  Release the Backup domain reset
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ReleaseBackupDomainReset(void)
{
  CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_BDRST);
}

/**
  * @brief  Set HSE Prescalers for RTC Clock
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_RTC_NOCLOCK
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_2
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_3
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_4
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_5
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_6
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_7
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_8
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_9
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_10
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_11
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_12
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_13
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_14
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_15
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_16
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_17
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_18
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_19
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_20
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_21
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_22
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_23
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_24
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_25
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_26
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_27
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_28
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_29
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_30
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_31
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetRTC_HSEPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCM->CFG, RCM_CFG_RTCPSC, Prescaler);
}

/**
  * @brief  Get HSE Prescalers for RTC Clock
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_RTC_NOCLOCK
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_2
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_3
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_4
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_5
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_6
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_7
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_8
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_9
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_10
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_11
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_12
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_13
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_14
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_15
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_16
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_17
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_18
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_19
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_20
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_21
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_22
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_23
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_24
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_25
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_26
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_27
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_28
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_29
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_30
  *         @arg @ref DDL_RCM_RTC_HSE_DIV_31
  */
__STATIC_INLINE uint32_t DDL_RCM_GetRTC_HSEPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCM->CFG, RCM_CFG_RTCPSC));
}

/**
  * @}
  */

#if defined(RCM_CFGSEL_CLKPSEL)
/** @defgroup RCM_DDL_EF_TMR_CLOCK_PRESCALER TMR
  * @{
  */

/**
  * @brief  Set Timers Clock Prescalers
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_TMR_PRESCALER_TWICE
  *         @arg @ref DDL_RCM_TMR_PRESCALER_FOUR_TIMES
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_SetTMRPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCM->CFGSEL, RCM_CFGSEL_CLKPSEL, Prescaler);
}

/**
  * @brief  Get Timers Clock Prescalers
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_TMR_PRESCALER_TWICE
  *         @arg @ref DDL_RCM_TMR_PRESCALER_FOUR_TIMES
  */
__STATIC_INLINE uint32_t DDL_RCM_GetTMRPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCM->CFGSEL, RCM_CFGSEL_CLKPSEL));
}

/**
  * @}
  */
#endif /* RCM_CFGSEL_CLKPSEL */

/** @defgroup RCM_DDL_EF_PLL PLL
  * @{
  */

/**
  * @brief  Enable PLL
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_Enable(void)
{
  SET_BIT(RCM->CTRL, RCM_CTRL_PLL1EN);
}

/**
  * @brief  Disable PLL
  * @note Cannot be disabled if the PLL clock is used as the system clock
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_Disable(void)
{
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_PLL1EN);
}

/**
  * @brief  Check if PLL Ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_IsReady(void)
{
  return (READ_BIT(RCM->CTRL, RCM_CTRL_PLL1RDYFLG) == (RCM_CTRL_PLL1RDYFLG));
}

/**
  * @brief  Configure PLL used for SYSCLK Domain
  * @note PLL Source and PLLB Divider can be written only when PLL,
  *       PLLI2S and PLLSAI(*) are disabled
  * @note PLL1A/PLL1C can be written only when PLL is disabled
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLSOURCE_HSI
  *         @arg @ref DDL_RCM_PLLSOURCE_HSE
  * @param  PLLB This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLB_DIV_2
  *         @arg @ref DDL_RCM_PLLB_DIV_3
  *         @arg @ref DDL_RCM_PLLB_DIV_4
  *         @arg @ref DDL_RCM_PLLB_DIV_5
  *         @arg @ref DDL_RCM_PLLB_DIV_6
  *         @arg @ref DDL_RCM_PLLB_DIV_7
  *         @arg @ref DDL_RCM_PLLB_DIV_8
  *         @arg @ref DDL_RCM_PLLB_DIV_9
  *         @arg @ref DDL_RCM_PLLB_DIV_10
  *         @arg @ref DDL_RCM_PLLB_DIV_11
  *         @arg @ref DDL_RCM_PLLB_DIV_12
  *         @arg @ref DDL_RCM_PLLB_DIV_13
  *         @arg @ref DDL_RCM_PLLB_DIV_14
  *         @arg @ref DDL_RCM_PLLB_DIV_15
  *         @arg @ref DDL_RCM_PLLB_DIV_16
  *         @arg @ref DDL_RCM_PLLB_DIV_17
  *         @arg @ref DDL_RCM_PLLB_DIV_18
  *         @arg @ref DDL_RCM_PLLB_DIV_19
  *         @arg @ref DDL_RCM_PLLB_DIV_20
  *         @arg @ref DDL_RCM_PLLB_DIV_21
  *         @arg @ref DDL_RCM_PLLB_DIV_22
  *         @arg @ref DDL_RCM_PLLB_DIV_23
  *         @arg @ref DDL_RCM_PLLB_DIV_24
  *         @arg @ref DDL_RCM_PLLB_DIV_25
  *         @arg @ref DDL_RCM_PLLB_DIV_26
  *         @arg @ref DDL_RCM_PLLB_DIV_27
  *         @arg @ref DDL_RCM_PLLB_DIV_28
  *         @arg @ref DDL_RCM_PLLB_DIV_29
  *         @arg @ref DDL_RCM_PLLB_DIV_30
  *         @arg @ref DDL_RCM_PLLB_DIV_31
  *         @arg @ref DDL_RCM_PLLB_DIV_32
  *         @arg @ref DDL_RCM_PLLB_DIV_33
  *         @arg @ref DDL_RCM_PLLB_DIV_34
  *         @arg @ref DDL_RCM_PLLB_DIV_35
  *         @arg @ref DDL_RCM_PLLB_DIV_36
  *         @arg @ref DDL_RCM_PLLB_DIV_37
  *         @arg @ref DDL_RCM_PLLB_DIV_38
  *         @arg @ref DDL_RCM_PLLB_DIV_39
  *         @arg @ref DDL_RCM_PLLB_DIV_40
  *         @arg @ref DDL_RCM_PLLB_DIV_41
  *         @arg @ref DDL_RCM_PLLB_DIV_42
  *         @arg @ref DDL_RCM_PLLB_DIV_43
  *         @arg @ref DDL_RCM_PLLB_DIV_44
  *         @arg @ref DDL_RCM_PLLB_DIV_45
  *         @arg @ref DDL_RCM_PLLB_DIV_46
  *         @arg @ref DDL_RCM_PLLB_DIV_47
  *         @arg @ref DDL_RCM_PLLB_DIV_48
  *         @arg @ref DDL_RCM_PLLB_DIV_49
  *         @arg @ref DDL_RCM_PLLB_DIV_50
  *         @arg @ref DDL_RCM_PLLB_DIV_51
  *         @arg @ref DDL_RCM_PLLB_DIV_52
  *         @arg @ref DDL_RCM_PLLB_DIV_53
  *         @arg @ref DDL_RCM_PLLB_DIV_54
  *         @arg @ref DDL_RCM_PLLB_DIV_55
  *         @arg @ref DDL_RCM_PLLB_DIV_56
  *         @arg @ref DDL_RCM_PLLB_DIV_57
  *         @arg @ref DDL_RCM_PLLB_DIV_58
  *         @arg @ref DDL_RCM_PLLB_DIV_59
  *         @arg @ref DDL_RCM_PLLB_DIV_60
  *         @arg @ref DDL_RCM_PLLB_DIV_61
  *         @arg @ref DDL_RCM_PLLB_DIV_62
  *         @arg @ref DDL_RCM_PLLB_DIV_63
  * @param  PLL1A Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  * @param  PLL1C_R This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLL1C_DIV_2
  *         @arg @ref DDL_RCM_PLL1C_DIV_4
  *         @arg @ref DDL_RCM_PLL1C_DIV_6
  *         @arg @ref DDL_RCM_PLL1C_DIV_8
  *         @arg @ref DDL_RCM_PLLR_DIV_2 (*)
  *         @arg @ref DDL_RCM_PLLR_DIV_3 (*)
  *         @arg @ref DDL_RCM_PLLR_DIV_4 (*)
  *         @arg @ref DDL_RCM_PLLR_DIV_5 (*)
  *         @arg @ref DDL_RCM_PLLR_DIV_6 (*)
  *         @arg @ref DDL_RCM_PLLR_DIV_7 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_ConfigDomain_SYS(uint32_t Source, uint32_t PLLB, uint32_t PLL1A, uint32_t PLL1C_R)
{
  MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLL1CLKS | RCM_PLL1CFG_PLLB | RCM_PLL1CFG_PLL1A,
             Source | PLLB | PLL1A << RCM_PLL1CFG_PLL1A_Pos);
  MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLL1C, PLL1C_R);
#if defined(RCM_PLLR_SYSCLK_SUPPORT)
  MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLLR, PLL1C_R);
#endif /* RCM_PLLR_SYSCLK_SUPPORT */
}

/**
  * @brief  Configure PLL used for 48Mhz domain clock
  * @note PLL Source and PLLB Divider can be written only when PLL,
  *       PLLI2S and PLLSAI(*) are disabled
  * @note PLL1A/PLLD can be written only when PLL is disabled
  * @note This  can be selected for USB, RNG, SDIO
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLSOURCE_HSI
  *         @arg @ref DDL_RCM_PLLSOURCE_HSE
  * @param  PLLB This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLB_DIV_2
  *         @arg @ref DDL_RCM_PLLB_DIV_3
  *         @arg @ref DDL_RCM_PLLB_DIV_4
  *         @arg @ref DDL_RCM_PLLB_DIV_5
  *         @arg @ref DDL_RCM_PLLB_DIV_6
  *         @arg @ref DDL_RCM_PLLB_DIV_7
  *         @arg @ref DDL_RCM_PLLB_DIV_8
  *         @arg @ref DDL_RCM_PLLB_DIV_9
  *         @arg @ref DDL_RCM_PLLB_DIV_10
  *         @arg @ref DDL_RCM_PLLB_DIV_11
  *         @arg @ref DDL_RCM_PLLB_DIV_12
  *         @arg @ref DDL_RCM_PLLB_DIV_13
  *         @arg @ref DDL_RCM_PLLB_DIV_14
  *         @arg @ref DDL_RCM_PLLB_DIV_15
  *         @arg @ref DDL_RCM_PLLB_DIV_16
  *         @arg @ref DDL_RCM_PLLB_DIV_17
  *         @arg @ref DDL_RCM_PLLB_DIV_18
  *         @arg @ref DDL_RCM_PLLB_DIV_19
  *         @arg @ref DDL_RCM_PLLB_DIV_20
  *         @arg @ref DDL_RCM_PLLB_DIV_21
  *         @arg @ref DDL_RCM_PLLB_DIV_22
  *         @arg @ref DDL_RCM_PLLB_DIV_23
  *         @arg @ref DDL_RCM_PLLB_DIV_24
  *         @arg @ref DDL_RCM_PLLB_DIV_25
  *         @arg @ref DDL_RCM_PLLB_DIV_26
  *         @arg @ref DDL_RCM_PLLB_DIV_27
  *         @arg @ref DDL_RCM_PLLB_DIV_28
  *         @arg @ref DDL_RCM_PLLB_DIV_29
  *         @arg @ref DDL_RCM_PLLB_DIV_30
  *         @arg @ref DDL_RCM_PLLB_DIV_31
  *         @arg @ref DDL_RCM_PLLB_DIV_32
  *         @arg @ref DDL_RCM_PLLB_DIV_33
  *         @arg @ref DDL_RCM_PLLB_DIV_34
  *         @arg @ref DDL_RCM_PLLB_DIV_35
  *         @arg @ref DDL_RCM_PLLB_DIV_36
  *         @arg @ref DDL_RCM_PLLB_DIV_37
  *         @arg @ref DDL_RCM_PLLB_DIV_38
  *         @arg @ref DDL_RCM_PLLB_DIV_39
  *         @arg @ref DDL_RCM_PLLB_DIV_40
  *         @arg @ref DDL_RCM_PLLB_DIV_41
  *         @arg @ref DDL_RCM_PLLB_DIV_42
  *         @arg @ref DDL_RCM_PLLB_DIV_43
  *         @arg @ref DDL_RCM_PLLB_DIV_44
  *         @arg @ref DDL_RCM_PLLB_DIV_45
  *         @arg @ref DDL_RCM_PLLB_DIV_46
  *         @arg @ref DDL_RCM_PLLB_DIV_47
  *         @arg @ref DDL_RCM_PLLB_DIV_48
  *         @arg @ref DDL_RCM_PLLB_DIV_49
  *         @arg @ref DDL_RCM_PLLB_DIV_50
  *         @arg @ref DDL_RCM_PLLB_DIV_51
  *         @arg @ref DDL_RCM_PLLB_DIV_52
  *         @arg @ref DDL_RCM_PLLB_DIV_53
  *         @arg @ref DDL_RCM_PLLB_DIV_54
  *         @arg @ref DDL_RCM_PLLB_DIV_55
  *         @arg @ref DDL_RCM_PLLB_DIV_56
  *         @arg @ref DDL_RCM_PLLB_DIV_57
  *         @arg @ref DDL_RCM_PLLB_DIV_58
  *         @arg @ref DDL_RCM_PLLB_DIV_59
  *         @arg @ref DDL_RCM_PLLB_DIV_60
  *         @arg @ref DDL_RCM_PLLB_DIV_61
  *         @arg @ref DDL_RCM_PLLB_DIV_62
  *         @arg @ref DDL_RCM_PLLB_DIV_63
  * @param  PLL1A Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  * @param  PLLD This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLD_DIV_2
  *         @arg @ref DDL_RCM_PLLD_DIV_3
  *         @arg @ref DDL_RCM_PLLD_DIV_4
  *         @arg @ref DDL_RCM_PLLD_DIV_5
  *         @arg @ref DDL_RCM_PLLD_DIV_6
  *         @arg @ref DDL_RCM_PLLD_DIV_7
  *         @arg @ref DDL_RCM_PLLD_DIV_8
  *         @arg @ref DDL_RCM_PLLD_DIV_9
  *         @arg @ref DDL_RCM_PLLD_DIV_10
  *         @arg @ref DDL_RCM_PLLD_DIV_11
  *         @arg @ref DDL_RCM_PLLD_DIV_12
  *         @arg @ref DDL_RCM_PLLD_DIV_13
  *         @arg @ref DDL_RCM_PLLD_DIV_14
  *         @arg @ref DDL_RCM_PLLD_DIV_15
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_ConfigDomain_48M(uint32_t Source, uint32_t PLLB, uint32_t PLL1A, uint32_t PLLD)
{
  MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLL1CLKS | RCM_PLL1CFG_PLLB | RCM_PLL1CFG_PLL1A | RCM_PLL1CFG_PLLD,
             Source | PLLB | PLL1A << RCM_PLL1CFG_PLL1A_Pos | PLLD);
}

/**
  * @brief  Configure PLL clock source
  * @param PLLSource This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLSOURCE_HSI
  *         @arg @ref DDL_RCM_PLLSOURCE_HSE
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_SetMainSource(uint32_t PLLSource)
{
  MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLL1CLKS, PLLSource);
}

/**
  * @brief  Get the oscillator used as PLL clock source.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_PLLSOURCE_HSI
  *         @arg @ref DDL_RCM_PLLSOURCE_HSE
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetMainSource(void)
{
  return (uint32_t)(READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLL1CLKS));
}

/**
  * @brief  Get Main PLL multiplication factor for VCO
  * @retval Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetN(void)
{
  return (uint32_t)(READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLL1A) >>  RCM_PLL1CFG_PLL1A_Pos);
}

/**
  * @brief  Get Main PLL division factor for PLL1C 
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_PLL1C_DIV_2
  *         @arg @ref DDL_RCM_PLL1C_DIV_4
  *         @arg @ref DDL_RCM_PLL1C_DIV_6
  *         @arg @ref DDL_RCM_PLL1C_DIV_8
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetP(void)
{
  return (uint32_t)(READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLL1C));
}

/**
  * @brief  Get Main PLL division factor for PLLD
  * @note used for PLL48MCLK selected for USB, RNG, SDIO (48 MHz clock)
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_PLLD_DIV_2
  *         @arg @ref DDL_RCM_PLLD_DIV_3
  *         @arg @ref DDL_RCM_PLLD_DIV_4
  *         @arg @ref DDL_RCM_PLLD_DIV_5
  *         @arg @ref DDL_RCM_PLLD_DIV_6
  *         @arg @ref DDL_RCM_PLLD_DIV_7
  *         @arg @ref DDL_RCM_PLLD_DIV_8
  *         @arg @ref DDL_RCM_PLLD_DIV_9
  *         @arg @ref DDL_RCM_PLLD_DIV_10
  *         @arg @ref DDL_RCM_PLLD_DIV_11
  *         @arg @ref DDL_RCM_PLLD_DIV_12
  *         @arg @ref DDL_RCM_PLLD_DIV_13
  *         @arg @ref DDL_RCM_PLLD_DIV_14
  *         @arg @ref DDL_RCM_PLLD_DIV_15
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetQ(void)
{
  return (uint32_t)(READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLLD));
}

/**
  * @brief  Get Division factor for the main PLL and other PLL
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_PLLB_DIV_2
  *         @arg @ref DDL_RCM_PLLB_DIV_3
  *         @arg @ref DDL_RCM_PLLB_DIV_4
  *         @arg @ref DDL_RCM_PLLB_DIV_5
  *         @arg @ref DDL_RCM_PLLB_DIV_6
  *         @arg @ref DDL_RCM_PLLB_DIV_7
  *         @arg @ref DDL_RCM_PLLB_DIV_8
  *         @arg @ref DDL_RCM_PLLB_DIV_9
  *         @arg @ref DDL_RCM_PLLB_DIV_10
  *         @arg @ref DDL_RCM_PLLB_DIV_11
  *         @arg @ref DDL_RCM_PLLB_DIV_12
  *         @arg @ref DDL_RCM_PLLB_DIV_13
  *         @arg @ref DDL_RCM_PLLB_DIV_14
  *         @arg @ref DDL_RCM_PLLB_DIV_15
  *         @arg @ref DDL_RCM_PLLB_DIV_16
  *         @arg @ref DDL_RCM_PLLB_DIV_17
  *         @arg @ref DDL_RCM_PLLB_DIV_18
  *         @arg @ref DDL_RCM_PLLB_DIV_19
  *         @arg @ref DDL_RCM_PLLB_DIV_20
  *         @arg @ref DDL_RCM_PLLB_DIV_21
  *         @arg @ref DDL_RCM_PLLB_DIV_22
  *         @arg @ref DDL_RCM_PLLB_DIV_23
  *         @arg @ref DDL_RCM_PLLB_DIV_24
  *         @arg @ref DDL_RCM_PLLB_DIV_25
  *         @arg @ref DDL_RCM_PLLB_DIV_26
  *         @arg @ref DDL_RCM_PLLB_DIV_27
  *         @arg @ref DDL_RCM_PLLB_DIV_28
  *         @arg @ref DDL_RCM_PLLB_DIV_29
  *         @arg @ref DDL_RCM_PLLB_DIV_30
  *         @arg @ref DDL_RCM_PLLB_DIV_31
  *         @arg @ref DDL_RCM_PLLB_DIV_32
  *         @arg @ref DDL_RCM_PLLB_DIV_33
  *         @arg @ref DDL_RCM_PLLB_DIV_34
  *         @arg @ref DDL_RCM_PLLB_DIV_35
  *         @arg @ref DDL_RCM_PLLB_DIV_36
  *         @arg @ref DDL_RCM_PLLB_DIV_37
  *         @arg @ref DDL_RCM_PLLB_DIV_38
  *         @arg @ref DDL_RCM_PLLB_DIV_39
  *         @arg @ref DDL_RCM_PLLB_DIV_40
  *         @arg @ref DDL_RCM_PLLB_DIV_41
  *         @arg @ref DDL_RCM_PLLB_DIV_42
  *         @arg @ref DDL_RCM_PLLB_DIV_43
  *         @arg @ref DDL_RCM_PLLB_DIV_44
  *         @arg @ref DDL_RCM_PLLB_DIV_45
  *         @arg @ref DDL_RCM_PLLB_DIV_46
  *         @arg @ref DDL_RCM_PLLB_DIV_47
  *         @arg @ref DDL_RCM_PLLB_DIV_48
  *         @arg @ref DDL_RCM_PLLB_DIV_49
  *         @arg @ref DDL_RCM_PLLB_DIV_50
  *         @arg @ref DDL_RCM_PLLB_DIV_51
  *         @arg @ref DDL_RCM_PLLB_DIV_52
  *         @arg @ref DDL_RCM_PLLB_DIV_53
  *         @arg @ref DDL_RCM_PLLB_DIV_54
  *         @arg @ref DDL_RCM_PLLB_DIV_55
  *         @arg @ref DDL_RCM_PLLB_DIV_56
  *         @arg @ref DDL_RCM_PLLB_DIV_57
  *         @arg @ref DDL_RCM_PLLB_DIV_58
  *         @arg @ref DDL_RCM_PLLB_DIV_59
  *         @arg @ref DDL_RCM_PLLB_DIV_60
  *         @arg @ref DDL_RCM_PLLB_DIV_61
  *         @arg @ref DDL_RCM_PLLB_DIV_62
  *         @arg @ref DDL_RCM_PLLB_DIV_63
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetDivider(void)
{
  return (uint32_t)(READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLLB));
}

/**
  * @brief  Configure Spread Spectrum used for PLL
  * @note These bits must be written before enabling PLL
  * @param  Mod Between Min_Data=0 and Max_Data=8191
  * @param  Inc Between Min_Data=0 and Max_Data=32767
  * @param  Sel This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_SPREAD_SELECT_CENTER
  *         @arg @ref DDL_RCM_SPREAD_SELECT_DOWN
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_ConfigSpreadSpectrum(uint32_t Mod, uint32_t Inc, uint32_t Sel)
{
  MODIFY_REG(RCM->SSCCFG, RCM_SSCCFG_MODPCFG | RCM_SSCCFG_STEP | RCM_SSCCFG_SSSEL, Mod | (Inc << RCM_SSCCFG_STEP_Pos) | Sel);
}

/**
  * @brief  Get Spread Spectrum Modulation Period for PLL
  * @retval Between Min_Data=0 and Max_Data=8191
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetPeriodModulation(void)
{
  return (uint32_t)(READ_BIT(RCM->SSCCFG, RCM_SSCCFG_MODPCFG));
}

/**
  * @brief  Get Spread Spectrum Incrementation Step for PLL
  * @note Must be written before enabling PLL
  * @retval Between Min_Data=0 and Max_Data=32767
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetStepIncrementation(void)
{
  return (uint32_t)(READ_BIT(RCM->SSCCFG, RCM_SSCCFG_STEP) >> RCM_SSCCFG_STEP_Pos);
}

/**
  * @brief  Get Spread Spectrum Selection for PLL
  * @note Must be written before enabling PLL
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_SPREAD_SELECT_CENTER
  *         @arg @ref DDL_RCM_SPREAD_SELECT_DOWN
  */
__STATIC_INLINE uint32_t DDL_RCM_PLL_GetSpreadSelection(void)
{
  return (uint32_t)(READ_BIT(RCM->SSCCFG, RCM_SSCCFG_SSSEL));
}

/**
  * @brief  Enable Spread Spectrum for PLL.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_SpreadSpectrum_Enable(void)
{
  SET_BIT(RCM->SSCCFG, RCM_SSCCFG_SSEN);
}

/**
  * @brief  Disable Spread Spectrum for PLL.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLL_SpreadSpectrum_Disable(void)
{
  CLEAR_BIT(RCM->SSCCFG, RCM_SSCCFG_SSEN);
}

/**
  * @}
  */

#if defined(RCM_PLLI2S_SUPPORT)
/** @defgroup RCM_DDL_EF_PLLI2S PLLI2S
  * @{
  */

/**
  * @brief  Enable PLLI2S
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLLI2S_Enable(void)
{
  SET_BIT(RCM->CTRL, RCM_CTRL_PLL2EN);
}

/**
  * @brief  Disable PLLI2S
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLLI2S_Disable(void)
{
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_PLL2EN);
}

/**
  * @brief  Check if PLLI2S Ready
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_PLLI2S_IsReady(void)
{
  return (READ_BIT(RCM->CTRL, RCM_CTRL_PLL2RDYFLG) == (RCM_CTRL_PLL2RDYFLG));
}

/**
  * @brief  Configure PLLI2S used for I2S1 domain clock
  * @note PLL Source and PLLB Divider can be written only when PLL,
  *       PLLI2S and PLLSAI(*) are disabled
  * @note PLL1A/PLLR can be written only when PLLI2S is disabled
  * @note This  can be selected for I2S
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLLSOURCE_HSI
  *         @arg @ref DDL_RCM_PLLSOURCE_HSE
  *         @arg @ref DDL_RCM_PLLI2SSOURCE_PIN (*)
  *
  *         (*) value not defined in all devices.
  * @param  PLLB This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLL2B_DIV_2
  *         @arg @ref DDL_RCM_PLL2B_DIV_3
  *         @arg @ref DDL_RCM_PLL2B_DIV_4
  *         @arg @ref DDL_RCM_PLL2B_DIV_5
  *         @arg @ref DDL_RCM_PLL2B_DIV_6
  *         @arg @ref DDL_RCM_PLL2B_DIV_7
  *         @arg @ref DDL_RCM_PLL2B_DIV_8
  *         @arg @ref DDL_RCM_PLL2B_DIV_9
  *         @arg @ref DDL_RCM_PLL2B_DIV_10
  *         @arg @ref DDL_RCM_PLL2B_DIV_11
  *         @arg @ref DDL_RCM_PLL2B_DIV_12
  *         @arg @ref DDL_RCM_PLL2B_DIV_13
  *         @arg @ref DDL_RCM_PLL2B_DIV_14
  *         @arg @ref DDL_RCM_PLL2B_DIV_15
  *         @arg @ref DDL_RCM_PLL2B_DIV_16
  *         @arg @ref DDL_RCM_PLL2B_DIV_17
  *         @arg @ref DDL_RCM_PLL2B_DIV_18
  *         @arg @ref DDL_RCM_PLL2B_DIV_19
  *         @arg @ref DDL_RCM_PLL2B_DIV_20
  *         @arg @ref DDL_RCM_PLL2B_DIV_21
  *         @arg @ref DDL_RCM_PLL2B_DIV_22
  *         @arg @ref DDL_RCM_PLL2B_DIV_23
  *         @arg @ref DDL_RCM_PLL2B_DIV_24
  *         @arg @ref DDL_RCM_PLL2B_DIV_25
  *         @arg @ref DDL_RCM_PLL2B_DIV_26
  *         @arg @ref DDL_RCM_PLL2B_DIV_27
  *         @arg @ref DDL_RCM_PLL2B_DIV_28
  *         @arg @ref DDL_RCM_PLL2B_DIV_29
  *         @arg @ref DDL_RCM_PLL2B_DIV_30
  *         @arg @ref DDL_RCM_PLL2B_DIV_31
  *         @arg @ref DDL_RCM_PLL2B_DIV_32
  *         @arg @ref DDL_RCM_PLL2B_DIV_33
  *         @arg @ref DDL_RCM_PLL2B_DIV_34
  *         @arg @ref DDL_RCM_PLL2B_DIV_35
  *         @arg @ref DDL_RCM_PLL2B_DIV_36
  *         @arg @ref DDL_RCM_PLL2B_DIV_37
  *         @arg @ref DDL_RCM_PLL2B_DIV_38
  *         @arg @ref DDL_RCM_PLL2B_DIV_39
  *         @arg @ref DDL_RCM_PLL2B_DIV_40
  *         @arg @ref DDL_RCM_PLL2B_DIV_41
  *         @arg @ref DDL_RCM_PLL2B_DIV_42
  *         @arg @ref DDL_RCM_PLL2B_DIV_43
  *         @arg @ref DDL_RCM_PLL2B_DIV_44
  *         @arg @ref DDL_RCM_PLL2B_DIV_45
  *         @arg @ref DDL_RCM_PLL2B_DIV_46
  *         @arg @ref DDL_RCM_PLL2B_DIV_47
  *         @arg @ref DDL_RCM_PLL2B_DIV_48
  *         @arg @ref DDL_RCM_PLL2B_DIV_49
  *         @arg @ref DDL_RCM_PLL2B_DIV_50
  *         @arg @ref DDL_RCM_PLL2B_DIV_51
  *         @arg @ref DDL_RCM_PLL2B_DIV_52
  *         @arg @ref DDL_RCM_PLL2B_DIV_53
  *         @arg @ref DDL_RCM_PLL2B_DIV_54
  *         @arg @ref DDL_RCM_PLL2B_DIV_55
  *         @arg @ref DDL_RCM_PLL2B_DIV_56
  *         @arg @ref DDL_RCM_PLL2B_DIV_57
  *         @arg @ref DDL_RCM_PLL2B_DIV_58
  *         @arg @ref DDL_RCM_PLL2B_DIV_59
  *         @arg @ref DDL_RCM_PLL2B_DIV_60
  *         @arg @ref DDL_RCM_PLL2B_DIV_61
  *         @arg @ref DDL_RCM_PLL2B_DIV_62
  *         @arg @ref DDL_RCM_PLL2B_DIV_63
  * @param  PLL1A Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  * @param  PLLR This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_PLL2C_DIV_2
  *         @arg @ref DDL_RCM_PLL2C_DIV_3
  *         @arg @ref DDL_RCM_PLL2C_DIV_4
  *         @arg @ref DDL_RCM_PLL2C_DIV_5
  *         @arg @ref DDL_RCM_PLL2C_DIV_6
  *         @arg @ref DDL_RCM_PLL2C_DIV_7
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_PLLI2S_ConfigDomain_I2S(uint32_t Source, uint32_t PLLB, uint32_t PLL1A, uint32_t PLLR)
{
  __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&RCM->PLL1CFG) + (Source & 0x80U)));
  MODIFY_REG(*pReg, RCM_PLL1CFG_PLL1CLKS, (Source & (~0x80U)));
#if defined(RCM_PLL2CFG_PLL2B)
  MODIFY_REG(RCM->PLL2CFG, RCM_PLL2CFG_PLL2B, PLLB);
#else
  MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLLB, PLLB);
#endif /* RCM_PLL2CFG_PLL2B */
  MODIFY_REG(RCM->PLL2CFG, RCM_PLL2CFG_PLL2A | RCM_PLL2CFG_PLL2C, PLL1A << RCM_PLL2CFG_PLL2A_Pos | PLLR);
}

/**
  * @brief  Get I2SPLL multiplication factor for VCO
  * @retval Between 50/192(*) and 432
  *
  *         (*) value not defined in all devices.
  */
__STATIC_INLINE uint32_t DDL_RCM_PLLI2S_GetN(void)
{
  return (uint32_t)(READ_BIT(RCM->PLL2CFG, RCM_PLL2CFG_PLL2A) >> RCM_PLL2CFG_PLL2A_Pos);
}

/**
  * @brief  Get I2SPLL division factor for PLL2C
  * @note used for PLLI2SCLK (I2S clock)
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_PLL2C_DIV_2
  *         @arg @ref DDL_RCM_PLL2C_DIV_3
  *         @arg @ref DDL_RCM_PLL2C_DIV_4
  *         @arg @ref DDL_RCM_PLL2C_DIV_5
  *         @arg @ref DDL_RCM_PLL2C_DIV_6
  *         @arg @ref DDL_RCM_PLL2C_DIV_7
  */
__STATIC_INLINE uint32_t DDL_RCM_PLLI2S_GetR(void)
{
  return (uint32_t)(READ_BIT(RCM->PLL2CFG, RCM_PLL2CFG_PLL2C));
}

/**
  * @brief  Get division factor for PLLI2S input clock
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_PLL2B_DIV_2
  *         @arg @ref DDL_RCM_PLL2B_DIV_3
  *         @arg @ref DDL_RCM_PLL2B_DIV_4
  *         @arg @ref DDL_RCM_PLL2B_DIV_5
  *         @arg @ref DDL_RCM_PLL2B_DIV_6
  *         @arg @ref DDL_RCM_PLL2B_DIV_7
  *         @arg @ref DDL_RCM_PLL2B_DIV_8
  *         @arg @ref DDL_RCM_PLL2B_DIV_9
  *         @arg @ref DDL_RCM_PLL2B_DIV_10
  *         @arg @ref DDL_RCM_PLL2B_DIV_11
  *         @arg @ref DDL_RCM_PLL2B_DIV_12
  *         @arg @ref DDL_RCM_PLL2B_DIV_13
  *         @arg @ref DDL_RCM_PLL2B_DIV_14
  *         @arg @ref DDL_RCM_PLL2B_DIV_15
  *         @arg @ref DDL_RCM_PLL2B_DIV_16
  *         @arg @ref DDL_RCM_PLL2B_DIV_17
  *         @arg @ref DDL_RCM_PLL2B_DIV_18
  *         @arg @ref DDL_RCM_PLL2B_DIV_19
  *         @arg @ref DDL_RCM_PLL2B_DIV_20
  *         @arg @ref DDL_RCM_PLL2B_DIV_21
  *         @arg @ref DDL_RCM_PLL2B_DIV_22
  *         @arg @ref DDL_RCM_PLL2B_DIV_23
  *         @arg @ref DDL_RCM_PLL2B_DIV_24
  *         @arg @ref DDL_RCM_PLL2B_DIV_25
  *         @arg @ref DDL_RCM_PLL2B_DIV_26
  *         @arg @ref DDL_RCM_PLL2B_DIV_27
  *         @arg @ref DDL_RCM_PLL2B_DIV_28
  *         @arg @ref DDL_RCM_PLL2B_DIV_29
  *         @arg @ref DDL_RCM_PLL2B_DIV_30
  *         @arg @ref DDL_RCM_PLL2B_DIV_31
  *         @arg @ref DDL_RCM_PLL2B_DIV_32
  *         @arg @ref DDL_RCM_PLL2B_DIV_33
  *         @arg @ref DDL_RCM_PLL2B_DIV_34
  *         @arg @ref DDL_RCM_PLL2B_DIV_35
  *         @arg @ref DDL_RCM_PLL2B_DIV_36
  *         @arg @ref DDL_RCM_PLL2B_DIV_37
  *         @arg @ref DDL_RCM_PLL2B_DIV_38
  *         @arg @ref DDL_RCM_PLL2B_DIV_39
  *         @arg @ref DDL_RCM_PLL2B_DIV_40
  *         @arg @ref DDL_RCM_PLL2B_DIV_41
  *         @arg @ref DDL_RCM_PLL2B_DIV_42
  *         @arg @ref DDL_RCM_PLL2B_DIV_43
  *         @arg @ref DDL_RCM_PLL2B_DIV_44
  *         @arg @ref DDL_RCM_PLL2B_DIV_45
  *         @arg @ref DDL_RCM_PLL2B_DIV_46
  *         @arg @ref DDL_RCM_PLL2B_DIV_47
  *         @arg @ref DDL_RCM_PLL2B_DIV_48
  *         @arg @ref DDL_RCM_PLL2B_DIV_49
  *         @arg @ref DDL_RCM_PLL2B_DIV_50
  *         @arg @ref DDL_RCM_PLL2B_DIV_51
  *         @arg @ref DDL_RCM_PLL2B_DIV_52
  *         @arg @ref DDL_RCM_PLL2B_DIV_53
  *         @arg @ref DDL_RCM_PLL2B_DIV_54
  *         @arg @ref DDL_RCM_PLL2B_DIV_55
  *         @arg @ref DDL_RCM_PLL2B_DIV_56
  *         @arg @ref DDL_RCM_PLL2B_DIV_57
  *         @arg @ref DDL_RCM_PLL2B_DIV_58
  *         @arg @ref DDL_RCM_PLL2B_DIV_59
  *         @arg @ref DDL_RCM_PLL2B_DIV_60
  *         @arg @ref DDL_RCM_PLL2B_DIV_61
  *         @arg @ref DDL_RCM_PLL2B_DIV_62
  *         @arg @ref DDL_RCM_PLL2B_DIV_63
  */
__STATIC_INLINE uint32_t DDL_RCM_PLLI2S_GetDivider(void)
{
#if defined(RCM_PLL2CFG_PLL2B)
  return (uint32_t)(READ_BIT(RCM->PLL2CFG, RCM_PLL2CFG_PLL2B));
#else
  return (uint32_t)(READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLLB));
#endif /* RCM_PLL2CFG_PLL2B */
}

/**
  * @brief  Get the oscillator used as PLL clock source.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_RCM_PLLSOURCE_HSI
  *         @arg @ref DDL_RCM_PLLSOURCE_HSE
  *         @arg @ref DDL_RCM_PLLI2SSOURCE_PIN (*)
  *
  *         (*) value not defined in all devices.
  */
__STATIC_INLINE uint32_t DDL_RCM_PLLI2S_GetMainSource(void)
{
#if defined(RCM_PLL2CFG_PLLI2SSRC)
  uint32_t pllsrc = READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLL1CLKS);
  uint32_t plli2sssrc0 = READ_BIT(RCM->PLL2CFG, RCM_PLL2CFG_PLLI2SSRC);
  uint32_t plli2sssrc1 = READ_BIT(RCM->PLL2CFG, RCM_PLL2CFG_PLLI2SSRC) >> 15U;
  return (uint32_t)(pllsrc | plli2sssrc0 | plli2sssrc1);
#else
  return (uint32_t)(READ_BIT(RCM->PLL1CFG, RCM_PLL1CFG_PLL1CLKS));
#endif /* RCM_PLL2CFG_PLLI2SSRC */
}

/**
  * @}
  */
#endif /* RCM_PLLI2S_SUPPORT */

/** @defgroup RCM_DDL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Clear LSI ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearFlag_LSIRDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_LSIRDYCLR);
}

/**
  * @brief  Clear LSE ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearFlag_LSERDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_LSERDYCLR);
}

/**
  * @brief  Clear HSI ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearFlag_HSIRDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_HSIRDYCLR);
}

/**
  * @brief  Clear HSE ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearFlag_HSERDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_HSERDYCLR);
}

/**
  * @brief  Clear PLL ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearFlag_PLLRDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_PLL1RDYCLR);
}

#if defined(RCM_PLLI2S_SUPPORT)
/**
  * @brief  Clear PLLI2S ready interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearFlag_PLL2RDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_PLL2RDYCLR);
}

#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @brief  Clear Clock security system interrupt flag
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearFlag_HSECSS(void)
{
  SET_BIT(RCM->INT, RCM_INT_CSSCLR);
}

/**
  * @brief  Check if LSI ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_LSIRDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_LSIRDYFLG) == (RCM_INT_LSIRDYFLG));
}

/**
  * @brief  Check if LSE ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_LSERDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_LSERDYFLG) == (RCM_INT_LSERDYFLG));
}

/**
  * @brief  Check if HSI ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_HSIRDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_HSIRDYFLG) == (RCM_INT_HSIRDYFLG));
}

/**
  * @brief  Check if HSE ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_HSERDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_HSERDYFLG) == (RCM_INT_HSERDYFLG));
}

/**
  * @brief  Check if PLL ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_PLLRDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_PLL1RDYFLG) == (RCM_INT_PLL1RDYFLG));
}

#if defined(RCM_PLLI2S_SUPPORT)
/**
  * @brief  Check if PLLI2S ready interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_PLL2RDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_PLL2RDYFLG) == (RCM_INT_PLL2RDYFLG));
}
#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @brief  Check if Clock security system interrupt occurred or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_HSECSS(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_CSSFLG) == (RCM_INT_CSSFLG));
}

/**
  * @brief  Check if RCM flag Independent Watchdog reset is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_IWDTRST(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_IWDTRSTFLG) == (RCM_CSTS_IWDTRSTFLG));
}

/**
  * @brief  Check if RCM flag Low Power reset is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_LPWRRST(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_LPWRRSTFLG) == (RCM_CSTS_LPWRRSTFLG));
}

/**
  * @brief  Check if RCM flag Pin reset is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_PINRST(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_PINRSTFLG) == (RCM_CSTS_PINRSTFLG));
}

/**
  * @brief  Check if RCM flag POR/PDR reset is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_PORRST(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_PODRSTFLG) == (RCM_CSTS_PODRSTFLG));
}

/**
  * @brief  Check if RCM flag Software reset is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_SFTRST(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_SWRSTFLG) == (RCM_CSTS_SWRSTFLG));
}

/**
  * @brief  Check if RCM flag Window Watchdog reset is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_WWDTRST(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_WWDTRSTFLG) == (RCM_CSTS_WWDTRSTFLG));
}

#if defined(RCM_CSTS_BORRSTFLG)
/**
  * @brief  Check if RCM flag BOR reset is set or not.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsActiveFlag_BORRST(void)
{
  return (READ_BIT(RCM->CSTS, RCM_CSTS_BORRSTFLG) == (RCM_CSTS_BORRSTFLG));
}
#endif /* RCM_CSTS_BORRSTFLG */

/**
  * @brief  Set RMVF bit to clear the reset flags.
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_ClearResetFlags(void)
{
  SET_BIT(RCM->CSTS, RCM_CSTS_RSTFLGCLR);
}

/**
  * @}
  */

/** @defgroup RCM_DDL_EF_IT_Management IT Management
  * @{
  */

/**
  * @brief  Enable LSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_EnableIT_LSIRDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_LSIRDYEN);
}

/**
  * @brief  Enable LSE ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_EnableIT_LSERDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_LSERDYEN);
}

/**
  * @brief  Enable HSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_EnableIT_HSIRDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_HSIRDYEN);
}

/**
  * @brief  Enable HSE ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_EnableIT_HSERDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_HSERDYEN);
}

/**
  * @brief  Enable PLL ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_EnableIT_PLLRDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_PLL1RDYEN);
}

#if defined(RCM_PLLI2S_SUPPORT)
/**
  * @brief  Enable PLLI2S ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_EnableIT_PLL2RDY(void)
{
  SET_BIT(RCM->INT, RCM_INT_PLL2RDYEN);
}
#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @brief  Disable LSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_DisableIT_LSIRDY(void)
{
  CLEAR_BIT(RCM->INT, RCM_INT_LSIRDYEN);
}

/**
  * @brief  Disable LSE ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_DisableIT_LSERDY(void)
{
  CLEAR_BIT(RCM->INT, RCM_INT_LSERDYEN);
}

/**
  * @brief  Disable HSI ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_DisableIT_HSIRDY(void)
{
  CLEAR_BIT(RCM->INT, RCM_INT_HSIRDYEN);
}

/**
  * @brief  Disable HSE ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_DisableIT_HSERDY(void)
{
  CLEAR_BIT(RCM->INT, RCM_INT_HSERDYEN);
}

/**
  * @brief  Disable PLL ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_DisableIT_PLLRDY(void)
{
  CLEAR_BIT(RCM->INT, RCM_INT_PLL1RDYEN);
}

#if defined(RCM_PLLI2S_SUPPORT)
/**
  * @brief  Disable PLLI2S ready interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_RCM_DisableIT_PLL2RDY(void)
{
  CLEAR_BIT(RCM->INT, RCM_INT_PLL2RDYEN);
}

#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @brief  Checks if LSI ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsEnabledIT_LSIRDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_LSIRDYEN) == (RCM_INT_LSIRDYEN));
}

/**
  * @brief  Checks if LSE ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsEnabledIT_LSERDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_LSERDYEN) == (RCM_INT_LSERDYEN));
}

/**
  * @brief  Checks if HSI ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsEnabledIT_HSIRDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_HSIRDYEN) == (RCM_INT_HSIRDYEN));
}

/**
  * @brief  Checks if HSE ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsEnabledIT_HSERDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_HSERDYEN) == (RCM_INT_HSERDYEN));
}

/**
  * @brief  Checks if PLL ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsEnabledIT_PLLRDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_PLL1RDYEN) == (RCM_INT_PLL1RDYEN));
}

#if defined(RCM_PLLI2S_SUPPORT)
/**
  * @brief  Checks if PLLI2S ready interrupt source is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RCM_IsEnabledIT_PLL2RDY(void)
{
  return (READ_BIT(RCM->INT, RCM_INT_PLL2RDYEN) == (RCM_INT_PLL2RDYEN));
}

#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RCM_DDL_EF_Init De-initialization function
  * @{
  */
ErrorStatus DDL_RCM_DeInit(void);
/**
  * @}
  */

/** @defgroup RCM_DDL_EF_Get_Freq Get system and peripherals clocks frequency functions
  * @{
  */
void        DDL_RCM_GetSystemClocksFreq(DDL_RCM_ClocksTypeDef *RCM_Clocks);
#if defined(SDIO)
uint32_t    DDL_RCM_GetSDIOClockFreq(uint32_t SDIOxSource);
#endif /* SDIO */
#if defined(RNG)
uint32_t    DDL_RCM_GetRNGClockFreq(uint32_t RNGxSource);
#endif /* RNG */
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
uint32_t    DDL_RCM_GetUSBClockFreq(uint32_t USBxSource);
#endif /* USB_OTG_FS || USB_OTG_HS */
uint32_t    DDL_RCM_GetI2SClockFreq(uint32_t I2SxSource);
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

#endif /* defined(RCM) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_RCM_H */

