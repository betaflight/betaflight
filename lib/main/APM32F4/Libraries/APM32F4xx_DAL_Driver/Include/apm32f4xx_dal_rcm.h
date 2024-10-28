/**
  *
  * @file    apm32f4xx_dal_rcm.h
  * @brief   Header file of RCM DAL module.
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
#ifndef APM32F4xx_DAL_RCM_H
#define APM32F4xx_DAL_RCM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/* Include RCM DAL Extended module */
/* (include on top of file since RCM structures are defined in extended file) */
#include "apm32f4xx_dal_rcm_ex.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup RCM
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup RCM_Exported_Types RCM Exported Types
  * @{
  */

/**
  * @brief  RCM Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCM_Oscillator_Type                   */

  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCM_HSE_Config                        */

  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCM_LSE_Config                        */

  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCM_HSI_Config                        */

  uint32_t HSICalibrationValue;  /*!< The HSI calibration trimming value (default is RCM_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCM_LSI_Config                        */

  RCM_PLLInitTypeDef PLL;        /*!< PLL structure parameters                                                    */
}RCM_OscInitTypeDef;

/**
  * @brief  RCM System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCM_System_Clock_Type      */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCM_System_Clock_Source    */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCM_AHB_Clock_Source       */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCM_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCM_APB1_APB2_Clock_Source */

}RCM_ClkInitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCM_Exported_Constants RCM Exported Constants
  * @{
  */

/** @defgroup RCM_Oscillator_Type Oscillator Type
  * @{
  */
#define RCM_OSCILLATORTYPE_NONE            0x00000000U
#define RCM_OSCILLATORTYPE_HSE             0x00000001U
#define RCM_OSCILLATORTYPE_HSI             0x00000002U
#define RCM_OSCILLATORTYPE_LSE             0x00000004U
#define RCM_OSCILLATORTYPE_LSI             0x00000008U
/**
  * @}
  */

/** @defgroup RCM_HSE_Config HSE Config
  * @{
  */
#define RCM_HSE_OFF                      0x00000000U
#define RCM_HSE_ON                       RCM_CTRL_HSEEN
#define RCM_HSE_BYPASS                   ((uint32_t)(RCM_CTRL_HSEBCFG | RCM_CTRL_HSEEN))
/**
  * @}
  */

/** @defgroup RCM_LSE_Config LSE Config
  * @{
  */
#define RCM_LSE_OFF                    0x00000000U
#define RCM_LSE_ON                     RCM_BDCTRL_LSEEN
#define RCM_LSE_BYPASS                 ((uint32_t)(RCM_BDCTRL_LSEBCFG | RCM_BDCTRL_LSEEN))
/**
  * @}
  */

/** @defgroup RCM_HSI_Config HSI Config
  * @{
  */
#define RCM_HSI_OFF                      ((uint8_t)0x00)
#define RCM_HSI_ON                       ((uint8_t)0x01)

#define RCM_HSICALIBRATION_DEFAULT       0x10U         /* Default HSI calibration trimming value */
/**
  * @}
  */

/** @defgroup RCM_LSI_Config LSI Config
  * @{
  */
#define RCM_LSI_OFF                      ((uint8_t)0x00)
#define RCM_LSI_ON                       ((uint8_t)0x01)
/**
  * @}
  */

/** @defgroup RCM_PLL_Config PLL Config
  * @{
  */
#define RCM_PLL_NONE                      ((uint8_t)0x00)
#define RCM_PLL_OFF                       ((uint8_t)0x01)
#define RCM_PLL_ON                        ((uint8_t)0x02)
/**
  * @}
  */

/** @defgroup RCM_PLL1C_Clock_Divider PLL1C Clock Divider
  * @{
  */
#define RCM_PLL1C_DIV2                  0x00000002U
#define RCM_PLL1C_DIV4                  0x00000004U
#define RCM_PLL1C_DIV6                  0x00000006U
#define RCM_PLL1C_DIV8                  0x00000008U
/**
  * @}
  */

/** @defgroup RCM_PLL_Clock_Source PLL Clock Source
  * @{
  */
#define RCM_PLLSOURCE_HSI                RCM_PLL1CFG_PLL1CLKS_HSI
#define RCM_PLLSOURCE_HSE                RCM_PLL1CFG_PLL1CLKS_HSE
/**
  * @}
  */

/** @defgroup RCM_System_Clock_Type System Clock Type
  * @{
  */
#define RCM_CLOCKTYPE_SYSCLK             0x00000001U
#define RCM_CLOCKTYPE_HCLK               0x00000002U
#define RCM_CLOCKTYPE_PCLK1              0x00000004U
#define RCM_CLOCKTYPE_PCLK2              0x00000008U
/**
  * @}
  */

/** @defgroup RCM_System_Clock_Source System Clock Source
  * @note     The RCM_SYSCLKSOURCE_PLLRCLK parameter is available only for
  *           APM32F446xx devices.
  * @{
  */
#define RCM_SYSCLKSOURCE_HSI             RCM_CFG_SCLKSEL_HSI
#define RCM_SYSCLKSOURCE_HSE             RCM_CFG_SCLKSEL_HSE
#define RCM_SYSCLKSOURCE_PLLCLK          RCM_CFG_SCLKSEL_PLL
#define RCM_SYSCLKSOURCE_PLLRCLK         ((uint32_t)(RCM_CFG_SCLKSEL_0 | RCM_CFG_SCLKSEL_1))
/**
  * @}
  */

/** @defgroup RCM_System_Clock_Source_Status System Clock Source Status
  * @note     The RCM_SYSCLKSOURCE_STATUS_PLLRCLK parameter is available only for
  *           APM32F446xx devices.
  * @{
  */
#define RCM_SYSCLKSOURCE_STATUS_HSI     RCM_CFG_SCLKSWSTS_HSI   /*!< HSI used as system clock */
#define RCM_SYSCLKSOURCE_STATUS_HSE     RCM_CFG_SCLKSWSTS_HSE   /*!< HSE used as system clock */
#define RCM_SYSCLKSOURCE_STATUS_PLLCLK  RCM_CFG_SCLKSWSTS_PLL   /*!< PLL used as system clock */
#define RCM_SYSCLKSOURCE_STATUS_PLLRCLK ((uint32_t)(RCM_CFG_SCLKSWSTS_0 | RCM_CFG_SCLKSWSTS_1))   /*!< PLLR used as system clock */
/**
  * @}
  */

/** @defgroup RCM_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCM_SYSCLK_DIV1                  RCM_CFG_AHBPSC_DIV1
#define RCM_SYSCLK_DIV2                  RCM_CFG_AHBPSC_DIV2
#define RCM_SYSCLK_DIV4                  RCM_CFG_AHBPSC_DIV4
#define RCM_SYSCLK_DIV8                  RCM_CFG_AHBPSC_DIV8
#define RCM_SYSCLK_DIV16                 RCM_CFG_AHBPSC_DIV16
#define RCM_SYSCLK_DIV64                 RCM_CFG_AHBPSC_DIV64
#define RCM_SYSCLK_DIV128                RCM_CFG_AHBPSC_DIV128
#define RCM_SYSCLK_DIV256                RCM_CFG_AHBPSC_DIV256
#define RCM_SYSCLK_DIV512                RCM_CFG_AHBPSC_DIV512
/**
  * @}
  */

/** @defgroup RCM_APB1_APB2_Clock_Source APB1/APB2 Clock Source
  * @{
  */
#define RCM_HCLK_DIV1                    RCM_CFG_APB1PSC_DIV1
#define RCM_HCLK_DIV2                    RCM_CFG_APB1PSC_DIV2
#define RCM_HCLK_DIV4                    RCM_CFG_APB1PSC_DIV4
#define RCM_HCLK_DIV8                    RCM_CFG_APB1PSC_DIV8
#define RCM_HCLK_DIV16                   RCM_CFG_APB1PSC_DIV16
/**
  * @}
  */

/** @defgroup RCM_RTC_Clock_Source RTC Clock Source
  * @{
  */
#define RCM_RTCCLKSOURCE_NO_CLK          0x00000000U
#define RCM_RTCCLKSOURCE_LSE             0x00000100U
#define RCM_RTCCLKSOURCE_LSI             0x00000200U
#define RCM_RTCCLKSOURCE_HSE_DIVX        0x00000300U
#define RCM_RTCCLKSOURCE_HSE_DIV2        0x00020300U
#define RCM_RTCCLKSOURCE_HSE_DIV3        0x00030300U
#define RCM_RTCCLKSOURCE_HSE_DIV4        0x00040300U
#define RCM_RTCCLKSOURCE_HSE_DIV5        0x00050300U
#define RCM_RTCCLKSOURCE_HSE_DIV6        0x00060300U
#define RCM_RTCCLKSOURCE_HSE_DIV7        0x00070300U
#define RCM_RTCCLKSOURCE_HSE_DIV8        0x00080300U
#define RCM_RTCCLKSOURCE_HSE_DIV9        0x00090300U
#define RCM_RTCCLKSOURCE_HSE_DIV10       0x000A0300U
#define RCM_RTCCLKSOURCE_HSE_DIV11       0x000B0300U
#define RCM_RTCCLKSOURCE_HSE_DIV12       0x000C0300U
#define RCM_RTCCLKSOURCE_HSE_DIV13       0x000D0300U
#define RCM_RTCCLKSOURCE_HSE_DIV14       0x000E0300U
#define RCM_RTCCLKSOURCE_HSE_DIV15       0x000F0300U
#define RCM_RTCCLKSOURCE_HSE_DIV16       0x00100300U
#define RCM_RTCCLKSOURCE_HSE_DIV17       0x00110300U
#define RCM_RTCCLKSOURCE_HSE_DIV18       0x00120300U
#define RCM_RTCCLKSOURCE_HSE_DIV19       0x00130300U
#define RCM_RTCCLKSOURCE_HSE_DIV20       0x00140300U
#define RCM_RTCCLKSOURCE_HSE_DIV21       0x00150300U
#define RCM_RTCCLKSOURCE_HSE_DIV22       0x00160300U
#define RCM_RTCCLKSOURCE_HSE_DIV23       0x00170300U
#define RCM_RTCCLKSOURCE_HSE_DIV24       0x00180300U
#define RCM_RTCCLKSOURCE_HSE_DIV25       0x00190300U
#define RCM_RTCCLKSOURCE_HSE_DIV26       0x001A0300U
#define RCM_RTCCLKSOURCE_HSE_DIV27       0x001B0300U
#define RCM_RTCCLKSOURCE_HSE_DIV28       0x001C0300U
#define RCM_RTCCLKSOURCE_HSE_DIV29       0x001D0300U
#define RCM_RTCCLKSOURCE_HSE_DIV30       0x001E0300U
#define RCM_RTCCLKSOURCE_HSE_DIV31       0x001F0300U
/**
  * @}
  */

/** @defgroup RCM_MCO_Index MCO Index
  * @{
  */
#define RCM_MCO1                         0x00000000U
#define RCM_MCO2                         0x00000001U
/**
  * @}
  */

/** @defgroup RCM_MCO1_Clock_Source MCO1 Clock Source
  * @{
  */
#define RCM_MCO1SOURCE_HSI               0x00000000U
#define RCM_MCO1SOURCE_LSE               RCM_CFG_MCO1SEL_0
#define RCM_MCO1SOURCE_HSE               RCM_CFG_MCO1SEL_1
#define RCM_MCO1SOURCE_PLLCLK            RCM_CFG_MCO1SEL
/**
  * @}
  */

/** @defgroup RCM_MCOx_Clock_Prescaler MCOx Clock Prescaler
  * @{
  */
#define RCM_MCODIV_1                    0x00000000U
#define RCM_MCODIV_2                    RCM_CFG_MCO1PSC_2
#define RCM_MCODIV_3                    ((uint32_t)RCM_CFG_MCO1PSC_0 | RCM_CFG_MCO1PSC_2)
#define RCM_MCODIV_4                    ((uint32_t)RCM_CFG_MCO1PSC_1 | RCM_CFG_MCO1PSC_2)
#define RCM_MCODIV_5                    RCM_CFG_MCO1PSC
/**
  * @}
  */

/** @defgroup RCM_Interrupt Interrupts
  * @{
  */
#define RCM_IT_LSIRDY                    ((uint8_t)0x01)
#define RCM_IT_LSERDY                    ((uint8_t)0x02)
#define RCM_IT_HSIRDY                    ((uint8_t)0x04)
#define RCM_IT_HSERDY                    ((uint8_t)0x08)
#define RCM_IT_PLLRDY                    ((uint8_t)0x10)
#define RCM_IT_PLL2RDY                   ((uint8_t)0x20)
#define RCM_IT_CSS                       ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup RCM_Flag Flags
  *        Elements values convention: 0XXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - 0XX  : Register index
  *                 - 01: CTRL register
  *                 - 10: BDCTRL register
  *                 - 11: CSTS register
  * @{
  */
/* Flags in the CTRL register */
#define RCM_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCM_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCM_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCM_FLAG_PLL2RDY                 ((uint8_t)0x3B)

/* Flags in the BDCTRL register */
#define RCM_FLAG_LSERDY                  ((uint8_t)0x41)

/* Flags in the CSTS register */
#define RCM_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCM_FLAG_BORRST                  ((uint8_t)0x79)
#define RCM_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCM_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCM_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCM_FLAG_IWDTRST                 ((uint8_t)0x7D)
#define RCM_FLAG_WWDTRST                 ((uint8_t)0x7E)
#define RCM_FLAG_LPWRRST                 ((uint8_t)0x7F)
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCM_Exported_Macros RCM Exported Macros
  * @{
  */

/** @defgroup RCM_AHB1_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_GPIOA_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PAEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PAEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_GPIOB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PBEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PBEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_GPIOC_CLK_ENABLE()  do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PCEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PCEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_GPIOH_CLK_ENABLE()  do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PHEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PHEN);\
                                        UNUSED(tmpreg); \
                                         } while(0U)
#define __DAL_RCM_DMA1_CLK_ENABLE()  do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DMA1EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DMA1EN);\
                                        UNUSED(tmpreg); \
                                         } while(0U)
#define __DAL_RCM_DMA2_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DMA2EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DMA2EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)

#define __DAL_RCM_GPIOA_CLK_DISABLE()        (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PAEN))
#define __DAL_RCM_GPIOB_CLK_DISABLE()        (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PBEN))
#define __DAL_RCM_GPIOC_CLK_DISABLE()        (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PCEN))
#define __DAL_RCM_GPIOH_CLK_DISABLE()        (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PHEN))
#define __DAL_RCM_DMA1_CLK_DISABLE()         (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_DMA1EN))
#define __DAL_RCM_DMA2_CLK_DISABLE()         (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_DMA2EN))
/**
  * @}
  */

/** @defgroup RCM_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_GPIOA_IS_CLK_ENABLED()        ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PAEN)) != RESET)
#define __DAL_RCM_GPIOB_IS_CLK_ENABLED()        ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PBEN)) != RESET)
#define __DAL_RCM_GPIOC_IS_CLK_ENABLED()        ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PCEN)) != RESET)
#define __DAL_RCM_GPIOH_IS_CLK_ENABLED()        ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PHEN)) != RESET)
#define __DAL_RCM_DMA1_IS_CLK_ENABLED()         ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_DMA1EN)) != RESET)
#define __DAL_RCM_DMA2_IS_CLK_ENABLED()         ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_DMA2EN)) != RESET)

#define __DAL_RCM_GPIOA_IS_CLK_DISABLED()       ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PAEN)) == RESET)
#define __DAL_RCM_GPIOB_IS_CLK_DISABLED()       ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PBEN)) == RESET)
#define __DAL_RCM_GPIOC_IS_CLK_DISABLED()       ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PCEN)) == RESET)
#define __DAL_RCM_GPIOH_IS_CLK_DISABLED()       ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_PHEN)) == RESET)
#define __DAL_RCM_DMA1_IS_CLK_DISABLED()        ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_DMA1EN)) == RESET)
#define __DAL_RCM_DMA2_IS_CLK_DISABLED()        ((RCM->AHB1CLKEN &(RCM_AHB1CLKEN_DMA2EN)) == RESET)
/**
  * @}
  */

/** @defgroup RCM_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_TMR5_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR5EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR5EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_WWDT_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_WWDTEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_WWDTEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_SPI2_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI2EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI2EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_USART2_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_USART2EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_USART2EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_I2C1_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C1EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C1EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_I2C2_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C2EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C2EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_PMU_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_PMUEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_PMUEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)

#define __DAL_RCM_TMR5_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR5EN))
#define __DAL_RCM_WWDT_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_WWDTEN))
#define __DAL_RCM_SPI2_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_SPI2EN))
#define __DAL_RCM_USART2_CLK_DISABLE() (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_USART2EN))
#define __DAL_RCM_I2C1_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_I2C1EN))
#define __DAL_RCM_I2C2_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_I2C2EN))
#define __DAL_RCM_PMU_CLK_DISABLE()    (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_PMUEN))
/**
  * @}
  */

/** @defgroup RCM_APB1_Peripheral_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_TMR5_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR5EN)) != RESET)
#define __DAL_RCM_WWDT_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_WWDTEN)) != RESET)
#define __DAL_RCM_SPI2_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_SPI2EN)) != RESET)
#define __DAL_RCM_USART2_IS_CLK_ENABLED() ((RCM->APB1CLKEN & (RCM_APB1CLKEN_USART2EN)) != RESET)
#define __DAL_RCM_I2C1_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C1EN)) != RESET)
#define __DAL_RCM_I2C2_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C2EN)) != RESET)
#define __DAL_RCM_PMU_IS_CLK_ENABLED()    ((RCM->APB1CLKEN & (RCM_APB1CLKEN_PMUEN)) != RESET)

#define __DAL_RCM_TMR5_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR5EN)) == RESET)
#define __DAL_RCM_WWDT_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_WWDTEN)) == RESET)
#define __DAL_RCM_SPI2_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_SPI2EN)) == RESET)
#define __DAL_RCM_USART2_IS_CLK_DISABLED() ((RCM->APB1CLKEN & (RCM_APB1CLKEN_USART2EN)) == RESET)
#define __DAL_RCM_I2C1_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C1EN)) == RESET)
#define __DAL_RCM_I2C2_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C2EN)) == RESET)
#define __DAL_RCM_PMU_IS_CLK_DISABLED()    ((RCM->APB1CLKEN & (RCM_APB1CLKEN_PMUEN)) == RESET)
/**
  * @}
  */

/** @defgroup RCM_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_TMR1_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR1EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR1EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_USART1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_USART1EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_USART1EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_USART6_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_USART6EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_USART6EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_ADC1_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC1EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC1EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_SPI1_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SPI1EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SPI1EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_SYSCFG_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SYSCFGEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SYSCFGEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_TMR9_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR9EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR9EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
#define __DAL_RCM_TMR11_CLK_ENABLE()    do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR11EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR11EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)

#define __DAL_RCM_TMR1_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_TMR1EN))
#define __DAL_RCM_USART1_CLK_DISABLE() (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_USART1EN))
#define __DAL_RCM_USART6_CLK_DISABLE() (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_USART6EN))
#define __DAL_RCM_ADC1_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_ADC1EN))
#define __DAL_RCM_SPI1_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_SPI1EN))
#define __DAL_RCM_SYSCFG_CLK_DISABLE() (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_SYSCFGEN))
#define __DAL_RCM_TMR9_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_TMR9EN))
#define __DAL_RCM_TMR11_CLK_DISABLE()  (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_TMR11EN))
/**
  * @}
  */

/** @defgroup RCM_APB2_Peripheral_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_TMR1_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR1EN)) != RESET)
#define __DAL_RCM_USART1_IS_CLK_ENABLED() ((RCM->APB2CLKEN & (RCM_APB2CLKEN_USART1EN)) != RESET)
#define __DAL_RCM_USART6_IS_CLK_ENABLED() ((RCM->APB2CLKEN & (RCM_APB2CLKEN_USART6EN)) != RESET)
#define __DAL_RCM_ADC1_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC1EN)) != RESET)
#define __DAL_RCM_SPI1_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SPI1EN)) != RESET)
#define __DAL_RCM_SYSCFG_IS_CLK_ENABLED() ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SYSCFGEN)) != RESET)
#define __DAL_RCM_TMR9_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR9EN)) != RESET)
#define __DAL_RCM_TMR11_IS_CLK_ENABLED()  ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR11EN)) != RESET)

#define __DAL_RCM_TMR1_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR1EN)) == RESET)
#define __DAL_RCM_USART1_IS_CLK_DISABLED() ((RCM->APB2CLKEN & (RCM_APB2CLKEN_USART1EN)) == RESET)
#define __DAL_RCM_USART6_IS_CLK_DISABLED() ((RCM->APB2CLKEN & (RCM_APB2CLKEN_USART6EN)) == RESET)
#define __DAL_RCM_ADC1_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC1EN)) == RESET)
#define __DAL_RCM_SPI1_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SPI1EN)) == RESET)
#define __DAL_RCM_SYSCFG_IS_CLK_DISABLED() ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SYSCFGEN)) == RESET)
#define __DAL_RCM_TMR9_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR9EN)) == RESET)
#define __DAL_RCM_TMR11_IS_CLK_DISABLED()  ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR11EN)) == RESET)
/**
  * @}
  */

/** @defgroup RCM_AHB1_Force_Release_Reset AHB1 Force Release Reset
  * @brief  Force or release AHB1 peripheral reset.
  * @{
  */
#define __DAL_RCM_AHB1_FORCE_RESET()    (RCM->AHB1RST = 0xFFFFFFFFU)
#define __DAL_RCM_GPIOA_FORCE_RESET()   (RCM->AHB1RST |= (RCM_AHB1RST_PARST))
#define __DAL_RCM_GPIOB_FORCE_RESET()   (RCM->AHB1RST |= (RCM_AHB1RST_PBRST))
#define __DAL_RCM_GPIOC_FORCE_RESET()   (RCM->AHB1RST |= (RCM_AHB1RST_PCRST))
#define __DAL_RCM_GPIOH_FORCE_RESET()   (RCM->AHB1RST |= (RCM_AHB1RST_PHRST))
#define __DAL_RCM_DMA1_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_DMA1RST))
#define __DAL_RCM_DMA2_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_DMA2RST))

#define __DAL_RCM_AHB1_RELEASE_RESET()  (RCM->AHB1RST = 0x00U)
#define __DAL_RCM_GPIOA_RELEASE_RESET() (RCM->AHB1RST &= ~(RCM_AHB1RST_PARST))
#define __DAL_RCM_GPIOB_RELEASE_RESET() (RCM->AHB1RST &= ~(RCM_AHB1RST_PBRST))
#define __DAL_RCM_GPIOC_RELEASE_RESET() (RCM->AHB1RST &= ~(RCM_AHB1RST_PCRST))
#define __DAL_RCM_GPIOH_RELEASE_RESET() (RCM->AHB1RST &= ~(RCM_AHB1RST_PHRST))
#define __DAL_RCM_DMA1_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_DMA1RST))
#define __DAL_RCM_DMA2_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_DMA2RST))
/**
  * @}
  */

/** @defgroup RCM_APB1_Force_Release_Reset APB1 Force Release Reset
  * @brief  Force or release APB1 peripheral reset.
  * @{
  */
#define __DAL_RCM_APB1_FORCE_RESET()     (RCM->APB1RST = 0xFFFFFFFFU)
#define __DAL_RCM_TMR5_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR5RST))
#define __DAL_RCM_WWDT_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_WWDTRST))
#define __DAL_RCM_SPI2_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_SPI2RST))
#define __DAL_RCM_USART2_FORCE_RESET()   (RCM->APB1RST |= (RCM_APB1RST_USART2RST))
#define __DAL_RCM_I2C1_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_I2C1RST))
#define __DAL_RCM_I2C2_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_I2C2RST))
#define __DAL_RCM_PMU_FORCE_RESET()      (RCM->APB1RST |= (RCM_APB1RST_PMURST))

#define __DAL_RCM_APB1_RELEASE_RESET()   (RCM->APB1RST = 0x00U)
#define __DAL_RCM_TMR5_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR5RST))
#define __DAL_RCM_WWDT_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_WWDTRST))
#define __DAL_RCM_SPI2_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_SPI2RST))
#define __DAL_RCM_USART2_RELEASE_RESET() (RCM->APB1RST &= ~(RCM_APB1RST_USART2RST))
#define __DAL_RCM_I2C1_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_I2C1RST))
#define __DAL_RCM_I2C2_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_I2C2RST))
#define __DAL_RCM_PMU_RELEASE_RESET()    (RCM->APB1RST &= ~(RCM_APB1RST_PMURST))
/**
  * @}
  */

/** @defgroup RCM_APB2_Force_Release_Reset APB2 Force Release Reset
  * @brief  Force or release APB2 peripheral reset.
  * @{
  */
#define __DAL_RCM_APB2_FORCE_RESET()     (RCM->APB2RST = 0xFFFFFFFFU)
#define __DAL_RCM_TMR1_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_TMR1RST))
#define __DAL_RCM_USART1_FORCE_RESET()   (RCM->APB2RST |= (RCM_APB2RST_USART1RST))
#define __DAL_RCM_USART6_FORCE_RESET()   (RCM->APB2RST |= (RCM_APB2RST_USART6RST))
#define __DAL_RCM_ADC_FORCE_RESET()      (RCM->APB2RST |= (RCM_APB2RST_ADCRST))
#define __DAL_RCM_SPI1_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_SPI1RST))
#define __DAL_RCM_SYSCFG_FORCE_RESET()   (RCM->APB2RST |= (RCM_APB2RST_SYSCFGRST))
#define __DAL_RCM_TMR9_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_TMR9RST))
#define __DAL_RCM_TMR11_FORCE_RESET()    (RCM->APB2RST |= (RCM_APB2RST_TMR11RST))

#define __DAL_RCM_APB2_RELEASE_RESET()   (RCM->APB2RST = 0x00U)
#define __DAL_RCM_TMR1_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_TMR1RST))
#define __DAL_RCM_USART1_RELEASE_RESET() (RCM->APB2RST &= ~(RCM_APB2RST_USART1RST))
#define __DAL_RCM_USART6_RELEASE_RESET() (RCM->APB2RST &= ~(RCM_APB2RST_USART6RST))
#define __DAL_RCM_ADC_RELEASE_RESET()    (RCM->APB2RST &= ~(RCM_APB2RST_ADCRST))
#define __DAL_RCM_SPI1_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_SPI1RST))
#define __DAL_RCM_SYSCFG_RELEASE_RESET() (RCM->APB2RST &= ~(RCM_APB2RST_SYSCFGRST))
#define __DAL_RCM_TMR9_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_TMR9RST))
#define __DAL_RCM_TMR11_RELEASE_RESET()  (RCM->APB2RST &= ~(RCM_APB2RST_TMR11RST))
/**
  * @}
  */

/** @defgroup RCM_AHB1_LowPower_Enable_Disable AHB1 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_GPIOA_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PALPEN))
#define __DAL_RCM_GPIOB_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PBLPEN))
#define __DAL_RCM_GPIOC_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PCLPEN))
#define __DAL_RCM_GPIOH_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PHLPEN))
#define __DAL_RCM_DMA1_CLK_SLEEP_ENABLE()     (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_DMA1EN))
#define __DAL_RCM_DMA2_CLK_SLEEP_ENABLE()     (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_DMA2EN))

#define __DAL_RCM_GPIOA_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PALPEN))
#define __DAL_RCM_GPIOB_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PBLPEN))
#define __DAL_RCM_GPIOC_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PCLPEN))
#define __DAL_RCM_GPIOH_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PHLPEN))
#define __DAL_RCM_DMA1_CLK_SLEEP_DISABLE()    (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_DMA1EN))
#define __DAL_RCM_DMA2_CLK_SLEEP_DISABLE()    (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_DMA2EN))
/**
  * @}
  */

/** @defgroup RCM_APB1_LowPower_Enable_Disable APB1 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_TMR5_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR5EN))
#define __DAL_RCM_WWDT_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_WWDTEN))
#define __DAL_RCM_SPI2_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_SPI2EN))
#define __DAL_RCM_USART2_CLK_SLEEP_ENABLE()  (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_USART2EN))
#define __DAL_RCM_I2C1_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_I2C1EN))
#define __DAL_RCM_I2C2_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_I2C2EN))
#define __DAL_RCM_PMU_CLK_SLEEP_ENABLE()     (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_PMUEN))

#define __DAL_RCM_TMR5_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR5EN))
#define __DAL_RCM_WWDT_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_WWDTEN))
#define __DAL_RCM_SPI2_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_SPI2EN))
#define __DAL_RCM_USART2_CLK_SLEEP_DISABLE() (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_USART2EN))
#define __DAL_RCM_I2C1_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_I2C1EN))
#define __DAL_RCM_I2C2_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_I2C2EN))
#define __DAL_RCM_PMU_CLK_SLEEP_DISABLE()    (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_PMUEN))
/**
  * @}
  */

/** @defgroup RCM_APB2_LowPower_Enable_Disable APB2 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_TMR1_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_TMR1EN))
#define __DAL_RCM_USART1_CLK_SLEEP_ENABLE()  (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_USART1EN))
#define __DAL_RCM_USART6_CLK_SLEEP_ENABLE()  (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_USART6EN))
#define __DAL_RCM_ADC1_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_ADC1EN))
#define __DAL_RCM_SPI1_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_SPI1EN))
#define __DAL_RCM_SYSCFG_CLK_SLEEP_ENABLE()  (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_SYSCFGEN))
#define __DAL_RCM_TMR9_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_TMR9EN))
#define __DAL_RCM_TMR11_CLK_SLEEP_ENABLE()   (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_TMR11EN))

#define __DAL_RCM_TMR1_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_TMR1EN))
#define __DAL_RCM_USART1_CLK_SLEEP_DISABLE() (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_USART1EN))
#define __DAL_RCM_USART6_CLK_SLEEP_DISABLE() (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_USART6EN))
#define __DAL_RCM_ADC1_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_ADC1EN))
#define __DAL_RCM_SPI1_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_SPI1EN))
#define __DAL_RCM_SYSCFG_CLK_SLEEP_DISABLE() (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_SYSCFGEN))
#define __DAL_RCM_TMR9_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_TMR9EN))
#define __DAL_RCM_TMR11_CLK_SLEEP_DISABLE()  (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_TMR11EN))
/**
  * @}
  */

/** @defgroup RCM_HSI_Configuration HSI Configuration
  * @{
  */

/** @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
  * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
  *         It is used (enabled by hardware) as system clock source after startup
  *         from Reset, wake-up from STOP and STANDBY mode, or in case of failure
  *         of the HSE used directly or indirectly as system clock (if the Clock
  *         Security System CSS is enabled).
  * @note   HSI can not be stopped if it is used as system clock source. In this case,
  *         you have to select another source of the system clock then stop the HSI.
  * @note   After enabling the HSI, the application software should wait on HSIRDY
  *         flag to be set indicating that HSI clock is stable and can be used as
  *         system clock source.
  *         This parameter can be: ENABLE or DISABLE.
  * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
  *         clock cycles.
  */
#define __DAL_RCM_HSI_ENABLE() (*(__IO uint32_t *) RCM_CTRL_HSIEN_BB = ENABLE)
#define __DAL_RCM_HSI_DISABLE() (*(__IO uint32_t *) RCM_CTRL_HSIEN_BB = DISABLE)

/** @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
  * @note   The calibration is used to compensate for the variations in voltage
  *         and temperature that influence the frequency of the internal HSI RC.
  * @param  __HSICalibrationValue__ specifies the calibration trimming value.
  *         (default is RCM_HSICALIBRATION_DEFAULT).
  *         This parameter must be a number between 0 and 0x1F.
  */
#define __DAL_RCM_HSI_CALIBRATIONVALUE_ADJUST(__HSICalibrationValue__) (MODIFY_REG(RCM->CTRL,\
        RCM_CTRL_HSITRM, (uint32_t)(__HSICalibrationValue__) << RCM_CTRL_HSITRM_Pos))
/**
  * @}
  */

/** @defgroup RCM_LSI_Configuration LSI Configuration
  * @{
  */

/** @brief  Macros to enable or disable the Internal Low Speed oscillator (LSI).
  * @note   After enabling the LSI, the application software should wait on
  *         LSIRDY flag to be set indicating that LSI clock is stable and can
  *         be used to clock the IWDT and/or the RTC.
  * @note   LSI can not be disabled if the IWDT is running.
  * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
  *         clock cycles.
  */
#define __DAL_RCM_LSI_ENABLE() (*(__IO uint32_t *) RCM_CSTS_LSIEN_BB = ENABLE)
#define __DAL_RCM_LSI_DISABLE() (*(__IO uint32_t *) RCM_CSTS_LSIEN_BB = DISABLE)
/**
  * @}
  */

/** @defgroup RCM_HSE_Configuration HSE Configuration
  * @{
  */

/**
  * @brief  Macro to configure the External High Speed oscillator (HSE).
  * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not supported by this macro.
  *         User should request a transition to HSE Off first and then HSE On or HSE Bypass.
  * @note   After enabling the HSE (RCM_HSE_ON or RCM_HSE_Bypass), the application
  *         software should wait on HSERDY flag to be set indicating that HSE clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   HSE state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the HSE state (ex. disable it).
  * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
  * @note   This function reset the CSSON bit, so if the clock security system(CSS)
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  __STATE__ specifies the new state of the HSE.
  *         This parameter can be one of the following values:
  *            @arg RCM_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
  *                              6 HSE oscillator clock cycles.
  *            @arg RCM_HSE_ON: turn ON the HSE oscillator.
  *            @arg RCM_HSE_BYPASS: HSE oscillator bypassed with external clock.
  */
#define __DAL_RCM_HSE_CONFIG(__STATE__)                         \
                    do {                                        \
                      if ((__STATE__) == RCM_HSE_ON)            \
                      {                                         \
                        SET_BIT(RCM->CTRL, RCM_CTRL_HSEEN);         \
                      }                                         \
                      else if ((__STATE__) == RCM_HSE_BYPASS)   \
                      {                                         \
                        SET_BIT(RCM->CTRL, RCM_CTRL_HSEBCFG);        \
                        SET_BIT(RCM->CTRL, RCM_CTRL_HSEEN);         \
                      }                                         \
                      else                                      \
                      {                                         \
                        CLEAR_BIT(RCM->CTRL, RCM_CTRL_HSEEN);       \
                        CLEAR_BIT(RCM->CTRL, RCM_CTRL_HSEBCFG);      \
                      }                                         \
                    } while(0U)
/**
  * @}
  */

/** @defgroup RCM_LSE_Configuration LSE Configuration
  * @{
  */

/**
  * @brief  Macro to configure the External Low Speed oscillator (LSE).
  * @note   Transition LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro.
  *         User should request a transition to LSE Off first and then LSE On or LSE Bypass.
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using
  *         DAL_PMU_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).
  * @note   After enabling the LSE (RCM_LSE_ON or RCM_LSE_BYPASS), the application
  *         software should wait on LSERDY flag to be set indicating that LSE clock
  *         is stable and can be used to clock the RTC.
  * @param  __STATE__ specifies the new state of the LSE.
  *         This parameter can be one of the following values:
  *            @arg RCM_LSE_OFF: turn OFF the LSE oscillator, LSERDY flag goes low after
  *                              6 LSE oscillator clock cycles.
  *            @arg RCM_LSE_ON: turn ON the LSE oscillator.
  *            @arg RCM_LSE_BYPASS: LSE oscillator bypassed with external clock.
  */
#define __DAL_RCM_LSE_CONFIG(__STATE__) \
                    do {                                       \
                      if((__STATE__) == RCM_LSE_ON)            \
                      {                                        \
                        SET_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEEN);    \
                      }                                        \
                      else if((__STATE__) == RCM_LSE_BYPASS)   \
                      {                                        \
                        SET_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEBCFG);   \
                        SET_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEEN);    \
                      }                                        \
                      else                                     \
                      {                                        \
                        CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEEN);  \
                        CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEBCFG); \
                      }                                        \
                    } while(0U)
/**
  * @}
  */

/** @defgroup RCM_Internal_RTC_Clock_Configuration RTC Clock Configuration
  * @{
  */

/** @brief  Macros to enable or disable the RTC clock.
  * @note   These macros must be used only after the RTC clock source was selected.
  */
#define __DAL_RCM_RTC_ENABLE() (*(__IO uint32_t *) RCM_BDCTRL_RTCCLKEN_BB = ENABLE)
#define __DAL_RCM_RTC_DISABLE() (*(__IO uint32_t *) RCM_BDCTRL_RTCCLKEN_BB = DISABLE)

/** @brief  Macros to configure the RTC clock (RTCCLK).
  * @note   As the RTC clock configuration bits are in the Backup domain and write
  *         access is denied to this domain after reset, you have to enable write
  *         access using the Power Backup Access macro before to configure
  *         the RTC clock source (to be done once after reset).
  * @note   Once the RTC clock is configured it can't be changed unless the
  *         Backup domain is reset using __DAL_RCM_BackupReset_RELEASE() macro, or by
  *         a Power On Reset (POR).
  * @param  __RTCCLKSource__ specifies the RTC clock source.
  *         This parameter can be one of the following values:
  *            @arg @ref RCM_RTCCLKSOURCE_NO_CLK : No clock selected as RTC clock.
  *            @arg @ref RCM_RTCCLKSOURCE_LSE : LSE selected as RTC clock.
  *            @arg @ref RCM_RTCCLKSOURCE_LSI : LSI selected as RTC clock.
  *            @arg @ref RCM_RTCCLKSOURCE_HSE_DIVX HSE divided by X selected as RTC clock (X can be retrieved thanks to @ref __DAL_RCM_GET_RTC_HSE_PRESCALER()
  * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
  *         work in STOP and STANDBY modes, and can be used as wake-up source.
  *         However, when the HSE clock is used as RTC clock source, the RTC
  *         cannot be used in STOP and STANDBY modes.
  * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
  *         RTC clock source).
  */
#define __DAL_RCM_RTC_CLKPRESCALER(__RTCCLKSource__) (((__RTCCLKSource__) & RCM_BDCTRL_RTCSRCSEL) == RCM_BDCTRL_RTCSRCSEL) ?    \
                                                 MODIFY_REG(RCM->CFG, RCM_CFG_RTCPSC, ((__RTCCLKSource__) & 0xFFFFCFFU)) : CLEAR_BIT(RCM->CFG, RCM_CFG_RTCPSC)

#define __DAL_RCM_RTC_CONFIG(__RTCCLKSource__) do { __DAL_RCM_RTC_CLKPRESCALER(__RTCCLKSource__);    \
                                                    RCM->BDCTRL |= ((__RTCCLKSource__) & 0x00000FFFU);  \
                                                   } while(0U)

/** @brief Macro to get the RTC clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCM_RTCCLKSOURCE_NO_CLK No clock selected as RTC clock
  *            @arg @ref RCM_RTCCLKSOURCE_LSE LSE selected as RTC clock
  *            @arg @ref RCM_RTCCLKSOURCE_LSI LSI selected as RTC clock
  *            @arg @ref RCM_RTCCLKSOURCE_HSE_DIVX HSE divided by X selected as RTC clock (X can be retrieved thanks to @ref __DAL_RCM_GET_RTC_HSE_PRESCALER()
  */
#define __DAL_RCM_GET_RTC_SOURCE() (READ_BIT(RCM->BDCTRL, RCM_BDCTRL_RTCSRCSEL))

/**
  * @brief   Get the RTC and HSE clock divider (RTCPRE).
  * @retval Returned value can be one of the following values:
 *            @arg @ref RCM_RTCCLKSOURCE_HSE_DIVX HSE divided by X selected as RTC clock (X can be retrieved thanks to @ref __DAL_RCM_GET_RTC_HSE_PRESCALER()
  */
#define  __DAL_RCM_GET_RTC_HSE_PRESCALER() (READ_BIT(RCM->CFG, RCM_CFG_RTCPSC) | RCM_BDCTRL_RTCSRCSEL)

/** @brief  Macros to force or release the Backup domain reset.
  * @note   This function resets the RTC peripheral (including the backup registers)
  *         and the RTC clock source selection in RCM_CSTS register.
  * @note   The BKPSRAM is not affected by this reset.
  */
#define __DAL_RCM_BACKUPRESET_FORCE() (*(__IO uint32_t *) RCM_BDCTRL_BDRST_BB = ENABLE)
#define __DAL_RCM_BACKUPRESET_RELEASE() (*(__IO uint32_t *) RCM_BDCTRL_BDRST_BB = DISABLE)
/**
  * @}
  */

/** @defgroup RCM_PLL_Configuration PLL Configuration
  * @{
  */

/** @brief  Macros to enable or disable the main PLL.
  * @note   After enabling the main PLL, the application software should wait on
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The main PLL can not be disabled if it is used as system clock source
  * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
  */
#define __DAL_RCM_PLL_ENABLE() (*(__IO uint32_t *) RCM_CTRL_PLL1EN_BB = ENABLE)
#define __DAL_RCM_PLL_DISABLE() (*(__IO uint32_t *) RCM_CTRL_PLL1EN_BB = DISABLE)

/** @brief  Macro to configure the PLL clock source.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLSOURCE__ specifies the PLL entry clock source.
  *         This parameter can be one of the following values:
  *            @arg RCM_PLLSOURCE_HSI: HSI oscillator clock selected as PLL clock entry
  *            @arg RCM_PLLSOURCE_HSE: HSE oscillator clock selected as PLL clock entry
  *
  */
#define __DAL_RCM_PLL_PLLSOURCE_CONFIG(__PLLSOURCE__) MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLL1CLKS, (__PLLSOURCE__))

/** @brief  Macro to configure the PLL multiplication factor.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLB__ specifies the division factor for PLL VCO input clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 63.
  * @note   You have to set the PLLB parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLL jitter.
  *
  */
#define __DAL_RCM_PLL_PLLB_CONFIG(__PLLB__) MODIFY_REG(RCM->PLL1CFG, RCM_PLL1CFG_PLLB, (__PLLB__))
/**
  * @}
  */

/** @defgroup RCM_Get_Clock_source Get Clock source
  * @{
  */
/**
  * @brief Macro to configure the system clock source.
  * @param __RCM_SYSCLKSOURCE__ specifies the system clock source.
  * This parameter can be one of the following values:
  *              - RCM_SYSCLKSOURCE_HSI: HSI oscillator is used as system clock source.
  *              - RCM_SYSCLKSOURCE_HSE: HSE oscillator is used as system clock source.
  *              - RCM_SYSCLKSOURCE_PLLCLK: PLL output is used as system clock source.
  *              - RCM_SYSCLKSOURCE_PLLRCLK: PLLR output is used as system clock source. This
  *                parameter is available only for APM32F446xx devices.
  */
#define __DAL_RCM_SYSCLK_CONFIG(__RCM_SYSCLKSOURCE__) MODIFY_REG(RCM->CFG, RCM_CFG_SCLKSEL, (__RCM_SYSCLKSOURCE__))

/** @brief  Macro to get the clock source used as system clock.
  * @retval The clock source used as system clock. The returned value can be one
  *         of the following:
  *              - RCM_SYSCLKSOURCE_STATUS_HSI: HSI used as system clock.
  *              - RCM_SYSCLKSOURCE_STATUS_HSE: HSE used as system clock.
  *              - RCM_SYSCLKSOURCE_STATUS_PLLCLK: PLL used as system clock.
  *              - RCM_SYSCLKSOURCE_STATUS_PLLRCLK: PLLR used as system clock. This parameter
  *                is available only for APM32F446xx devices.
  */
#define __DAL_RCM_GET_SYSCLK_SOURCE() (RCM->CFG & RCM_CFG_SCLKSWSTS)

/** @brief  Macro to get the oscillator used as PLL clock source.
  * @retval The oscillator used as PLL clock source. The returned value can be one
  *         of the following:
  *              - RCM_PLLSOURCE_HSI: HSI oscillator is used as PLL clock source.
  *              - RCM_PLLSOURCE_HSE: HSE oscillator is used as PLL clock source.
  */
#define __DAL_RCM_GET_PLL_OSCSOURCE() ((uint32_t)(RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS))
/**
  * @}
  */

/** @defgroup RCMEx_MCOx_Clock_Config RCM Extended MCOx Clock Config
  * @{
  */

/** @brief  Macro to configure the MCO1 clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
  *            @arg RCM_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
  *            @arg RCM_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
  *            @arg RCM_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCODIV_1: no division applied to MCOx clock
  *            @arg RCM_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCM_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCM_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCM_MCODIV_5: division by 5 applied to MCOx clock
  */
#define __DAL_RCM_MCO1_CONFIG(__MCOCLKSOURCE__, __MCODIV__) \
                 MODIFY_REG(RCM->CFG, (RCM_CFG_MCO1SEL | RCM_CFG_MCO1PSC), ((__MCOCLKSOURCE__) | (__MCODIV__)))

/** @brief  Macro to configure the MCO2 clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
  *            @arg RCM_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source, available for all APM32F4 devices
  *            @arg RCM_MCO2SOURCE_I2SCLK: I2SCLK clock selected as MCO2 source, available only for APM32F410Rx devices
  *            @arg RCM_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
  *            @arg RCM_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCODIV_1: no division applied to MCOx clock
  *            @arg RCM_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCM_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCM_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCM_MCODIV_5: division by 5 applied to MCOx clock
  */
#define __DAL_RCM_MCO2_CONFIG(__MCOCLKSOURCE__, __MCODIV__) \
    MODIFY_REG(RCM->CFG, (RCM_CFG_MCO2SEL | RCM_CFG_MCO2PSC), ((__MCOCLKSOURCE__) | ((__MCODIV__) << 3U)));
/**
  * @}
  */

/** @defgroup RCM_Flags_Interrupts_Management Flags Interrupts Management
  * @brief macros to manage the specified RCM Flags and interrupts.
  * @{
  */

/** @brief  Enable RCM interrupt (Perform Byte access to RCM_INT[14:8] bits to enable
  *         the selected interrupts).
  * @param  __INTERRUPT__ specifies the RCM interrupt sources to be enabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCM_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCM_IT_LSERDY: LSE ready interrupt.
  *            @arg RCM_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCM_IT_HSERDY: HSE ready interrupt.
  *            @arg RCM_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCM_IT_PLL2RDY: PLLI2S ready interrupt.
  */
#define __DAL_RCM_ENABLE_IT(__INTERRUPT__) (*(__IO uint8_t *) RCM_INT_BYTE1_ADDRESS |= (__INTERRUPT__))

/** @brief Disable RCM interrupt (Perform Byte access to RCM_INT[14:8] bits to disable
  *        the selected interrupts).
  * @param  __INTERRUPT__ specifies the RCM interrupt sources to be disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCM_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCM_IT_LSERDY: LSE ready interrupt.
  *            @arg RCM_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCM_IT_HSERDY: HSE ready interrupt.
  *            @arg RCM_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCM_IT_PLL2RDY: PLLI2S ready interrupt.
  */
#define __DAL_RCM_DISABLE_IT(__INTERRUPT__) (*(__IO uint8_t *) RCM_INT_BYTE1_ADDRESS &= (uint8_t)(~(__INTERRUPT__)))

/** @brief  Clear the RCM's interrupt pending bits (Perform Byte access to RCM_INT[23:16]
  *         bits to clear the selected interrupt pending bits.
  * @param  __INTERRUPT__ specifies the interrupt pending bit to clear.
  *         This parameter can be any combination of the following values:
  *            @arg RCM_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCM_IT_LSERDY: LSE ready interrupt.
  *            @arg RCM_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCM_IT_HSERDY: HSE ready interrupt.
  *            @arg RCM_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCM_IT_PLL2RDY: PLLI2S ready interrupt.
  *            @arg RCM_IT_CSS: Clock Security System interrupt
  */
#define __DAL_RCM_CLEAR_IT(__INTERRUPT__) (*(__IO uint8_t *) RCM_INT_BYTE2_ADDRESS = (__INTERRUPT__))

/** @brief  Check the RCM's interrupt has occurred or not.
  * @param  __INTERRUPT__ specifies the RCM interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg RCM_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCM_IT_LSERDY: LSE ready interrupt.
  *            @arg RCM_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCM_IT_HSERDY: HSE ready interrupt.
  *            @arg RCM_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCM_IT_PLL2RDY: PLLI2S ready interrupt.
  *            @arg RCM_IT_CSS: Clock Security System interrupt
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __DAL_RCM_GET_IT(__INTERRUPT__) ((RCM->INT & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief Set RMVF bit to clear the reset flags: RCM_FLAG_PINRST, RCM_FLAG_PORRST,
  *        RCM_FLAG_SFTRST, RCM_FLAG_IWDTRST, RCM_FLAG_WWDTRST and RCM_FLAG_LPWRRST.
  */
#define __DAL_RCM_CLEAR_RESET_FLAGS() (RCM->CSTS |= RCM_CSTS_RSTFLGCLR)

/** @brief  Check RCM flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg RCM_FLAG_HSIRDY: HSI oscillator clock ready.
  *            @arg RCM_FLAG_HSERDY: HSE oscillator clock ready.
  *            @arg RCM_FLAG_PLLRDY: Main PLL clock ready.
  *            @arg RCM_FLAG_PLL2RDY: PLLI2S clock ready.
  *            @arg RCM_FLAG_LSERDY: LSE oscillator clock ready.
  *            @arg RCM_FLAG_LSIRDY: LSI oscillator clock ready.
  *            @arg RCM_FLAG_BORRST: POR/PDR or BOR reset.
  *            @arg RCM_FLAG_PINRST: Pin reset.
  *            @arg RCM_FLAG_PORRST: POR/PDR reset.
  *            @arg RCM_FLAG_SFTRST: Software reset.
  *            @arg RCM_FLAG_IWDTRST: Independent Watchdog reset.
  *            @arg RCM_FLAG_WWDTRST: Window Watchdog reset.
  *            @arg RCM_FLAG_LPWRRST: Low Power reset.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define RCM_FLAG_MASK  ((uint8_t)0x1FU)
#define __DAL_RCM_GET_FLAG(__FLAG__) (((((((__FLAG__) >> 5U) == 1U)? RCM->CTRL :((((__FLAG__) >> 5U) == 2U) ? RCM->BDCTRL :((((__FLAG__) >> 5U) == 3U)? RCM->CSTS :RCM->INT))) & (1U << ((__FLAG__) & RCM_FLAG_MASK)))!= 0U)? 1U : 0U)

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
 /** @addtogroup RCM_Exported_Functions
  * @{
  */

/** @addtogroup RCM_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ******************************/
DAL_StatusTypeDef DAL_RCM_DeInit(void);
DAL_StatusTypeDef DAL_RCM_OscConfig(RCM_OscInitTypeDef *RCM_OscInitStruct);
DAL_StatusTypeDef DAL_RCM_ClockConfig(RCM_ClkInitTypeDef *RCM_ClkInitStruct, uint32_t FLatency);
/**
  * @}
  */

/** @addtogroup RCM_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  ************************************************/
void     DAL_RCM_MCOConfig(uint32_t RCM_MCOx, uint32_t RCM_MCOSource, uint32_t RCM_MCODiv);
void     DAL_RCM_EnableCSS(void);
void     DAL_RCM_DisableCSS(void);
uint32_t DAL_RCM_GetSysClockFreq(void);
uint32_t DAL_RCM_GetHCLKFreq(void);
uint32_t DAL_RCM_GetPCLK1Freq(void);
uint32_t DAL_RCM_GetPCLK2Freq(void);
void     DAL_RCM_GetOscConfig(RCM_OscInitTypeDef *RCM_OscInitStruct);
void     DAL_RCM_GetClockConfig(RCM_ClkInitTypeDef *RCM_ClkInitStruct, uint32_t *pFLatency);

/* CSS NMI IRQ handler */
void DAL_RCM_NMI_IRQHandler(void);

/* User Callbacks in non blocking mode (IT mode) */
void DAL_RCM_CSSCallback(void);

/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup RCM_Private_Constants RCM Private Constants
  * @{
  */

/** @defgroup RCM_BitAddress_AliasRegion RCM BitAddress AliasRegion
  * @brief RCM registers bit address in the alias region
  * @{
  */
#define RCM_OFFSET                 (RCM_BASE - PERIPH_BASE)
/* --- CTRL Register --- */
/* Alias word address of HSIEN bit */
#define RCM_CTRL_OFFSET              (RCM_OFFSET + 0x00U)
#define RCM_HSIEN_BIT_NUMBER       0x00U
#define RCM_CTRL_HSIEN_BB            (PERIPH_BB_BASE + (RCM_CTRL_OFFSET * 32U) + (RCM_HSIEN_BIT_NUMBER * 4U))
/* Alias word address of CSSEN bit */
#define RCM_CSSEN_BIT_NUMBER       0x13U
#define RCM_CTRL_CSSEN_BB            (PERIPH_BB_BASE + (RCM_CTRL_OFFSET * 32U) + (RCM_CSSEN_BIT_NUMBER * 4U))
/* Alias word address of PLL1EN bit */
#define RCM_PLLEN_BIT_NUMBER       0x18U
#define RCM_CTRL_PLL1EN_BB            (PERIPH_BB_BASE + (RCM_CTRL_OFFSET * 32U) + (RCM_PLLEN_BIT_NUMBER * 4U))

/* --- BDCTRL Register --- */
/* Alias word address of RTCEN bit */
#define RCM_BDCTRL_OFFSET            (RCM_OFFSET + 0x70U)
#define RCM_RTCEN_BIT_NUMBER       0x0FU
#define RCM_BDCTRL_RTCCLKEN_BB          (PERIPH_BB_BASE + (RCM_BDCTRL_OFFSET * 32U) + (RCM_RTCEN_BIT_NUMBER * 4U))
/* Alias word address of BDRST bit */
#define RCM_BDRST_BIT_NUMBER       0x10U
#define RCM_BDCTRL_BDRST_BB          (PERIPH_BB_BASE + (RCM_BDCTRL_OFFSET * 32U) + (RCM_BDRST_BIT_NUMBER * 4U))

/* --- CSTS Register --- */
/* Alias word address of LSION bit */
#define RCM_CSTS_OFFSET             (RCM_OFFSET + 0x74U)
#define RCM_LSIEN_BIT_NUMBER        0x00U
#define RCM_CSTS_LSIEN_BB           (PERIPH_BB_BASE + (RCM_CSTS_OFFSET * 32U) + (RCM_LSIEN_BIT_NUMBER * 4U))

/* CTRL register byte 3 (Bits[23:16]) base address */
#define RCM_CTRL_BYTE2_ADDRESS       0x40023802U

/* INT register byte 2 (Bits[15:8]) base address */
#define RCM_INT_BYTE1_ADDRESS      ((uint32_t)(RCM_BASE + 0x0CU + 0x01U))

/* INT register byte 3 (Bits[23:16]) base address */
#define RCM_INT_BYTE2_ADDRESS      ((uint32_t)(RCM_BASE + 0x0CU + 0x02U))

/* BDCTRL register base address */
#define RCM_BDCTRL_BYTE0_ADDRESS     (PERIPH_BASE + RCM_BDCTRL_OFFSET)

#define RCM_DBP_TIMEOUT_VALUE      2U
#define RCM_LSE_TIMEOUT_VALUE      LSE_STARTUP_TIMEOUT

#define HSE_TIMEOUT_VALUE          HSE_STARTUP_TIMEOUT
#define HSI_TIMEOUT_VALUE          2U  /* 2 ms */
#define LSI_TIMEOUT_VALUE          2U  /* 2 ms */
#define CLOCKSWITCH_TIMEOUT_VALUE  5000U /* 5 s */

/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RCM_Private_Macros RCM Private Macros
  * @{
  */

/** @defgroup RCM_IS_RCM_Definitions RCM Private macros to check input parameters
  * @{
  */
#define IS_RCM_OSCILLATORTYPE(OSCILLATOR) ((OSCILLATOR) <= 15U)

#define IS_RCM_HSE(HSE) (((HSE) == RCM_HSE_OFF) || ((HSE) == RCM_HSE_ON) || \
                         ((HSE) == RCM_HSE_BYPASS))

#define IS_RCM_LSE(LSE) (((LSE) == RCM_LSE_OFF) || ((LSE) == RCM_LSE_ON) || \
                         ((LSE) == RCM_LSE_BYPASS))

#define IS_RCM_HSI(HSI) (((HSI) == RCM_HSI_OFF) || ((HSI) == RCM_HSI_ON))

#define IS_RCM_LSI(LSI) (((LSI) == RCM_LSI_OFF) || ((LSI) == RCM_LSI_ON))

#define IS_RCM_PLL(PLL) (((PLL) == RCM_PLL_NONE) ||((PLL) == RCM_PLL_OFF) || ((PLL) == RCM_PLL_ON))

#define IS_RCM_PLLSOURCE(SOURCE) (((SOURCE) == RCM_PLLSOURCE_HSI) || \
                                  ((SOURCE) == RCM_PLLSOURCE_HSE))

#define IS_RCM_SYSCLKSOURCE(SOURCE) (((SOURCE) == RCM_SYSCLKSOURCE_HSI) || \
                                     ((SOURCE) == RCM_SYSCLKSOURCE_HSE) || \
                                     ((SOURCE) == RCM_SYSCLKSOURCE_PLLCLK) || \
                                     ((SOURCE) == RCM_SYSCLKSOURCE_PLLRCLK))

#define IS_RCM_RTCCLKSOURCE(__SOURCE__) (((__SOURCE__) == RCM_RTCCLKSOURCE_LSE) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_LSI) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV2) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV3) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV4) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV5) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV6) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV7) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV8) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV9) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV10) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV11) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV12) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV13) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV14) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV15) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV16) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV17) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV18) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV19) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV20) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV21) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV22) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV23) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV24) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV25) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV26) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV27) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV28) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV29) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV30) || \
                                         ((__SOURCE__) == RCM_RTCCLKSOURCE_HSE_DIV31))

#define IS_RCM_PLLB_VALUE(VALUE) ((VALUE) <= 63U)

#define IS_RCM_PLL1C_VALUE(VALUE) (((VALUE) == 2U) || ((VALUE) == 4U) || ((VALUE) == 6U) || ((VALUE) == 8U))

#define IS_RCM_PLLD_VALUE(VALUE) ((2U <= (VALUE)) && ((VALUE) <= 15U))

#define IS_RCM_HCLK(HCLK) (((HCLK) == RCM_SYSCLK_DIV1)   || ((HCLK) == RCM_SYSCLK_DIV2)   || \
                           ((HCLK) == RCM_SYSCLK_DIV4)   || ((HCLK) == RCM_SYSCLK_DIV8)   || \
                           ((HCLK) == RCM_SYSCLK_DIV16)  || ((HCLK) == RCM_SYSCLK_DIV64)  || \
                           ((HCLK) == RCM_SYSCLK_DIV128) || ((HCLK) == RCM_SYSCLK_DIV256) || \
                           ((HCLK) == RCM_SYSCLK_DIV512))

#define IS_RCM_CLOCKTYPE(CLK) ((1U <= (CLK)) && ((CLK) <= 15U))

#define IS_RCM_PCLK(PCLK) (((PCLK) == RCM_HCLK_DIV1) || ((PCLK) == RCM_HCLK_DIV2) || \
                           ((PCLK) == RCM_HCLK_DIV4) || ((PCLK) == RCM_HCLK_DIV8) || \
                           ((PCLK) == RCM_HCLK_DIV16))

#define IS_RCM_MCO(MCOx) (((MCOx) == RCM_MCO1) || ((MCOx) == RCM_MCO2))

#define IS_RCM_MCO1SOURCE(SOURCE) (((SOURCE) == RCM_MCO1SOURCE_HSI) || ((SOURCE) == RCM_MCO1SOURCE_LSE) || \
                                   ((SOURCE) == RCM_MCO1SOURCE_HSE) || ((SOURCE) == RCM_MCO1SOURCE_PLLCLK))

#define IS_RCM_MCODIV(DIV) (((DIV) == RCM_MCODIV_1)  || ((DIV) == RCM_MCODIV_2) || \
                             ((DIV) == RCM_MCODIV_3) || ((DIV) == RCM_MCODIV_4) || \
                             ((DIV) == RCM_MCODIV_5))
#define IS_RCM_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x1FU)

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

#endif /* APM32F4xx_DAL_RCM_H */

