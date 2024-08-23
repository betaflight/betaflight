/**
  *
  * @file    apm32f4xx_dal_rcm_ex.h
  * @brief   Header file of RCM DAL Extension module.
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
#ifndef APM32F4xx_DAL_RCM_EX_H
#define APM32F4xx_DAL_RCM_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup RCMEx
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/** @defgroup RCMEx_Exported_Types RCMEx Exported Types
  * @{
  */

/**
  * @brief  RCM PLL configuration structure definition
  */
typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCM_PLL_Config                      */

  uint32_t PLLSource;  /*!< RCM_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCM_PLL_Clock_Source               */

  uint32_t PLLB;       /*!< PLLB: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 63    */

  uint32_t PLL1A;       /*!< PLL1A: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432 
                            except for APM32F411xx devices where the Min_Data = 192 */

  uint32_t PLL1C;       /*!< PLL1C: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCM_PLL1C_Clock_Divider             */

  uint32_t PLLD;       /*!< PLLD: Division factor for OTG FS, SDIO and RNG clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15    */

}RCM_PLLInitTypeDef;

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
/** 
  * @brief  PLLI2S Clock structure definition  
  */
typedef struct
{
#if defined(APM32F411xx)
  uint32_t PLL2B;    /*!< PLLB: Division factor for PLLI2S VCO input clock.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 62  */
#endif /* APM32F411xx */
                                
  uint32_t PLL2A;    /*!< Specifies the multiplication factor for PLLI2S VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432
                            Except for APM32F411xx devices where the Min_Data = 192. 
                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */

  uint32_t PLL2C;    /*!< Specifies the division factor for I2S clock.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 7. 
                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */

}RCM_PLLI2SInitTypeDef;
 
/** 
  * @brief  RCM extended clocks structure definition  
  */
typedef struct
{
  uint32_t PeriphClockSelection; /*!< The Extended Clock to be configured.
                                      This parameter can be a value of @ref RCMEx_Periph_Clock_Selection */
#if defined(APM32F407xx) || defined(APM32F417xx)
  uint32_t SDRAMClockDivision;  /*!< The SDRAM clock division.
                                      This parameter can be a value of @ref RCMEx_SDRAM_Clock_Division */
#endif /* APM32F407xx || APM32F417xx */
  RCM_PLLI2SInitTypeDef PLLI2S;  /*!< PLL I2S structure parameters.
                                      This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */

  uint32_t RTCClockSelection;      /*!< Specifies RTC Clock Prescalers Selection.
                                       This parameter can be a value of @ref RCM_RTC_Clock_Source */
#if defined(APM32F411xx) 
  uint8_t TMRPresSelection;        /*!< Specifies TMR Clock Source Selection. 
                                      This parameter can be a value of @ref RCMEx_TMR_PRescaler_Selection */
#endif /* APM32F411xx */
}RCM_PeriphCLKInitTypeDef;
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */
/**
  * @}
  */ 

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCMEx_Exported_Constants RCMEx Exported Constants
  * @{
  */

/** @defgroup RCMEx_Periph_Clock_Selection RCM Periph Clock Selection
  * @{
  */

/*-------- Peripheral Clock source for APM32F40xxx/APM32F41xxx/APM32F465xx ---------------*/
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx) 
#define RCM_PERIPHCLK_I2S             0x00000001U
#define RCM_PERIPHCLK_RTC             0x00000002U
#define RCM_PERIPHCLK_PLLI2S          0x00000004U
#if defined(APM32F407xx) || defined(APM32F417xx)
#define RCM_PERIPHCLK_SDRAM           0x00000008U
#endif /* APM32F407xx || APM32F417xx */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx ||  APM32F411xx */
#if defined(APM32F411xx)
#define RCM_PERIPHCLK_TMR             0x00000008U
#endif /* APM32F411xx */
/*----------------------------------------------------------------------------*/
/**
  * @}
  */

#if defined(APM32F407xx) || defined(APM32F417xx)
/** @defgroup RCMEx_SDRAM_Clock_Division SDRAM Clock Division
  * @{
  */
#define RCM_SDRAM_DIV_1                 0x00000000U
#define RCM_SDRAM_DIV_2                 0x00000001U
#define RCM_SDRAM_DIV_4                 0x00000002U

/**
  * @}
  */
#endif /* APM32F407xx || APM32F417xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx)
/** @defgroup RCMEx_I2S_Clock_Source I2S Clock Source
  * @{
  */
#define RCM_I2SCLKSOURCE_PLLI2S         0x00000000U
#define RCM_I2SCLKSOURCE_EXT            0x00000001U
/**
  * @}
  */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

#if defined(APM32F411xx)
/** @defgroup RCMEx_TMR_PRescaler_Selection  RCM TMR PRescaler Selection
  * @{
  */
#define RCM_TMRPRES_DESACTIVATED        ((uint8_t)0x00)
#define RCM_TMRPRES_ACTIVATED           ((uint8_t)0x01)
/**
  * @}
  */
#endif /* APM32F411xx */

#if defined(APM32F411xx)
/** @defgroup RCMEx_LSE_Dual_Mode_Selection  RCM LSE Dual Mode Selection
  * @{
  */
#define RCM_LSE_LOWPOWER_MODE           ((uint8_t)0x00)
#define RCM_LSE_HIGHDRIVE_MODE          ((uint8_t)0x01)
/**
  * @}
  */
#endif /* APM32F411xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx)
/** @defgroup RCM_MCO2_Clock_Source MCO2 Clock Source
  * @{
  */
#define RCM_MCO2SOURCE_SYSCLK            0x00000000U
#define RCM_MCO2SOURCE_PLLI2SCLK         RCM_CFG_MCO2SEL_0
#define RCM_MCO2SOURCE_HSE               RCM_CFG_MCO2SEL_1
#define RCM_MCO2SOURCE_PLLCLK            RCM_CFG_MCO2SEL
/**
  * @}
  */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx ||
          APM32F465xx || APM32F411xx */

/**
  * @}
  */
     
/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCMEx_Exported_Macros RCMEx Exported Macros
  * @{
  */

/*----------------------------------- APM32F40xxx/APM32F41xxx/APM32F465xx-----------------*/
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
/** @defgroup RCMEx_AHB1_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
  * @brief  Enables or disables the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */
#define __DAL_RCM_BKPSRAM_CLK_ENABLE() do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_BKPSRAMEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_BKPSRAMEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_CCMDATARAMEN_CLK_ENABLE() do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DRAMEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DRAMEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_CRC_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_CRCEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_CRCEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_GPIOD_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PDEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PDEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_GPIOE_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PEEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PEEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_GPIOI_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U; \
                                       SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PIEN);\
                                       /* Delay after an RCM peripheral clock enabling */ \
                                       tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PIEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#define __DAL_RCM_GPIOF_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U; \
                                       SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PFEN);\
                                       /* Delay after an RCM peripheral clock enabling */ \
                                       tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PFEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#define __DAL_RCM_GPIOG_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U; \
                                       SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PGEN);\
                                       /* Delay after an RCM peripheral clock enabling */ \
                                       tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PGEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#define __DAL_RCM_USB_OTG_HS_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U; \
                                       SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_OTGHS1EN);\
                                       /* Delay after an RCM peripheral clock enabling */ \
                                       tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_OTGHS1EN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#define __DAL_RCM_USB_OTG_HS_ULPI_CLK_ENABLE()   do { \
                                       __IO uint32_t tmpreg = 0x00U; \
                                       SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_OTGHSULPIEN);\
                                       /* Delay after an RCM peripheral clock enabling */ \
                                       tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_OTGHSULPIEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#define __DAL_RCM_GPIOD_CLK_DISABLE()           (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PDEN))
#define __DAL_RCM_GPIOE_CLK_DISABLE()           (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PEEN))
#define __DAL_RCM_GPIOF_CLK_DISABLE()           (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PFEN))
#define __DAL_RCM_GPIOG_CLK_DISABLE()           (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PGEN))
#define __DAL_RCM_GPIOI_CLK_DISABLE()           (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PIEN))
#define __DAL_RCM_USB_OTG_HS_CLK_DISABLE()      (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_OTGHS1EN))
#define __DAL_RCM_USB_OTG_HS_ULPI_CLK_DISABLE() (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_OTGHSULPIEN))
#define __DAL_RCM_BKPSRAM_CLK_DISABLE()         (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_BKPSRAMEN))
#define __DAL_RCM_CCMDATARAMEN_CLK_DISABLE()    (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_DRAMEN))
#define __DAL_RCM_CRC_CLK_DISABLE()             (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_CRCEN))
#if defined(APM32F407xx) || defined(APM32F417xx)
/**
  * @brief  Enable ETHERNET clock.
  */
#define __DAL_RCM_ETHMAC_CLK_ENABLE()  do { \
                                       __IO uint32_t tmpreg = 0x00U; \
                                       SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHEN);\
                                       /* Delay after an RCM peripheral clock enabling */ \
                                       tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHEN);\
                                       UNUSED(tmpreg); \
                                       } while(0U)
#define __DAL_RCM_ETHMACTX_CLK_ENABLE() do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHTXEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHTXEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_ETHMACRX_CLK_ENABLE() do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHRXEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHRXEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_ETHMACPTP_CLK_ENABLE() do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHPTPEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_ETHPTPEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_ETH_CLK_ENABLE()      do {                            \
                                        __DAL_RCM_ETHMAC_CLK_ENABLE();      \
                                        __DAL_RCM_ETHMACTX_CLK_ENABLE();    \
                                        __DAL_RCM_ETHMACRX_CLK_ENABLE();    \
                                        } while(0U)

/**
  * @brief  Disable ETHERNET clock.
  */
#define __DAL_RCM_ETHMAC_CLK_DISABLE()    (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_ETHEN))
#define __DAL_RCM_ETHMACTX_CLK_DISABLE()  (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_ETHTXEN))
#define __DAL_RCM_ETHMACRX_CLK_DISABLE()  (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_ETHRXEN))
#define __DAL_RCM_ETHMACPTP_CLK_DISABLE() (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_ETHPTPEN))  
#define __DAL_RCM_ETH_CLK_DISABLE()       do {                             \
                                           __DAL_RCM_ETHMACTX_CLK_DISABLE();    \
                                           __DAL_RCM_ETHMACRX_CLK_DISABLE();    \
                                           __DAL_RCM_ETHMAC_CLK_DISABLE();      \
                                          } while(0U)
#endif /* APM32F407xx || APM32F417xx */
/**
  * @}
  */
  
/** @defgroup RCMEx_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */  
#define __DAL_RCM_BKPSRAM_IS_CLK_ENABLED()          ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_BKPSRAMEN)) != RESET)
#define __DAL_RCM_CCMDATARAMEN_IS_CLK_ENABLED()     ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_DRAMEN)) != RESET)
#define __DAL_RCM_CRC_IS_CLK_ENABLED()              ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_CRCEN)) != RESET)
#define __DAL_RCM_GPIOD_IS_CLK_ENABLED()            ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PDEN)) != RESET)
#define __DAL_RCM_GPIOE_IS_CLK_ENABLED()            ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PEEN)) != RESET)
#define __DAL_RCM_GPIOI_IS_CLK_ENABLED()            ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PIEN)) != RESET)
#define __DAL_RCM_GPIOF_IS_CLK_ENABLED()            ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PFEN)) != RESET)
#define __DAL_RCM_GPIOG_IS_CLK_ENABLED()            ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PGEN)) != RESET)
#define __DAL_RCM_USB_OTG_HS_IS_CLK_ENABLED()       ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_OTGHS1EN)) != RESET)
#define __DAL_RCM_USB_OTG_HS_ULPI_IS_CLK_ENABLED()  ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_OTGHSULPIEN)) != RESET)

#define __DAL_RCM_GPIOD_IS_CLK_DISABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PDEN)) == RESET)
#define __DAL_RCM_GPIOE_IS_CLK_DISABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PEEN)) == RESET)
#define __DAL_RCM_GPIOF_IS_CLK_DISABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PFEN)) == RESET)
#define __DAL_RCM_GPIOG_IS_CLK_DISABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PGEN)) == RESET)
#define __DAL_RCM_GPIOI_IS_CLK_DISABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PIEN)) == RESET)
#define __DAL_RCM_USB_OTG_HS_IS_CLK_DISABLED()      ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_OTGHS1EN)) == RESET)
#define __DAL_RCM_USB_OTG_HS_ULPI_IS_CLK_DISABLED() ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_OTGHSULPIEN))== RESET)
#define __DAL_RCM_BKPSRAM_IS_CLK_DISABLED()         ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_BKPSRAMEN)) == RESET)
#define __DAL_RCM_CCMDATARAMEN_IS_CLK_DISABLED()    ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_DRAMEN)) == RESET)
#define __DAL_RCM_CRC_IS_CLK_DISABLED()             ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_CRCEN)) == RESET)
#if defined(APM32F407xx) || defined(APM32F417xx)
/**
  * @brief  Enable ETHERNET clock.
  */
#define __DAL_RCM_ETHMAC_IS_CLK_ENABLED()     ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHEN)) != RESET)
#define __DAL_RCM_ETHMACTX_IS_CLK_ENABLED()   ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHTXEN)) != RESET)
#define __DAL_RCM_ETHMACRX_IS_CLK_ENABLED()   ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHRXEN)) != RESET)
#define __DAL_RCM_ETHMACPTP_IS_CLK_ENABLED()  ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHPTPEN)) != RESET)
#define __DAL_RCM_ETH_IS_CLK_ENABLED()        (__DAL_RCM_ETHMAC_IS_CLK_ENABLED()   && \
                                               __DAL_RCM_ETHMACTX_IS_CLK_ENABLED() && \
                                               __DAL_RCM_ETHMACRX_IS_CLK_ENABLED())
/**
  * @brief  Disable ETHERNET clock.
  */
#define __DAL_RCM_ETHMAC_IS_CLK_DISABLED()    ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHEN)) == RESET)
#define __DAL_RCM_ETHMACTX_IS_CLK_DISABLED()  ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHTXEN)) == RESET)
#define __DAL_RCM_ETHMACRX_IS_CLK_DISABLED()  ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHRXEN)) == RESET)
#define __DAL_RCM_ETHMACPTP_IS_CLK_DISABLED() ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_ETHPTPEN)) == RESET)
#define __DAL_RCM_ETH_IS_CLK_DISABLED()        (__DAL_RCM_ETHMAC_IS_CLK_DISABLED()   && \
                                                __DAL_RCM_ETHMACTX_IS_CLK_DISABLED() && \
                                                __DAL_RCM_ETHMACRX_IS_CLK_DISABLED())
#endif /* APM32F407xx || APM32F417xx */
/**
  * @}
  */
  
/** @defgroup RCMEx_AHB2_Clock_Enable_Disable AHB2 Peripheral Clock Enable Disable 
  * @brief  Enable or disable the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */
#define __DAL_RCM_USB_OTG_FS_CLK_ENABLE()  do {(RCM->AHB2CLKEN |= (RCM_AHB2CLKEN_OTGFSEN));\
                                               __DAL_RCM_SYSCFG_CLK_ENABLE();\
                                              }while(0U)
                                        
#define __DAL_RCM_USB_OTG_FS_CLK_DISABLE() (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_OTGFSEN))

#define __DAL_RCM_RNG_CLK_ENABLE()    do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_RNGEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_RNGEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_RNG_CLK_DISABLE()   (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_RNGEN))

#if defined(APM32F407xx) || defined(APM32F417xx) 
#define __DAL_RCM_DCI_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_DCIEN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_DCIEN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_DCI_CLK_DISABLE()  (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_DCIEN))
#endif /* APM32F407xx || APM32F417xx */

#if defined(APM32F417xx)
#define __DAL_RCM_CRYP_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_CRYPEN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_CRYPEN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_HASH_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_HASHEN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_HASHEN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_CRYP_CLK_DISABLE()  (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_CRYPEN))
#define __DAL_RCM_HASH_CLK_DISABLE()  (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_HASHEN))
#endif /* APM32F417xx */
/**
  * @}
  */


/** @defgroup RCMEx_AHB2_Peripheral_Clock_Enable_Disable_Status AHB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_USB_OTG_FS_IS_CLK_ENABLED()  ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_OTGFSEN)) != RESET)
#define __DAL_RCM_USB_OTG_FS_IS_CLK_DISABLED() ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_OTGFSEN)) == RESET) 

#define __DAL_RCM_RNG_IS_CLK_ENABLED()   ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_RNGEN)) != RESET)   
#define __DAL_RCM_RNG_IS_CLK_DISABLED()  ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_RNGEN)) == RESET) 

#if defined(APM32F407xx) || defined(APM32F417xx)
#define __DAL_RCM_DCI_IS_CLK_ENABLED()  ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_DCIEN)) != RESET) 
#define __DAL_RCM_DCI_IS_CLK_DISABLED() ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_DCIEN)) == RESET) 
#endif /* APM32F407xx || APM32F417xx */

#if defined(APM32F417xx)
#define __DAL_RCM_CRYP_IS_CLK_ENABLED()   ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_CRYPEN)) != RESET) 
#define __DAL_RCM_HASH_IS_CLK_ENABLED()   ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_HASHEN)) != RESET) 

#define __DAL_RCM_CRYP_IS_CLK_DISABLED()  ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_CRYPEN)) == RESET) 
#define __DAL_RCM_HASH_IS_CLK_DISABLED()  ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_HASHEN)) == RESET) 
#endif /* APM32F417xx */  
/**
  * @}
  */  
  
/** @defgroup RCMEx_AHB3_Clock_Enable_Disable AHB3 Peripheral Clock Enable Disable
  * @brief  Enables or disables the AHB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{  
  */
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define __DAL_RCM_EMMC_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->AHB3CLKEN, RCM_AHB3CLKEN_EMMCEN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->AHB3CLKEN, RCM_AHB3CLKEN_EMMCEN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_EMMC_CLK_DISABLE() (RCM->AHB3CLKEN &= ~(RCM_AHB3CLKEN_EMMCEN))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */
/**
  * @}
  */

/** @defgroup RCMEx_AHB3_Peripheral_Clock_Enable_Disable_Status AHB3 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB3 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define __DAL_RCM_EMMC_IS_CLK_ENABLED()   ((RCM->AHB3CLKEN & (RCM_AHB3CLKEN_EMMCEN)) != RESET) 
#define __DAL_RCM_EMMC_IS_CLK_DISABLED()  ((RCM->AHB3CLKEN & (RCM_AHB3CLKEN_EMMCEN)) == RESET) 
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */
/**
  * @}
  */   
   
/** @defgroup RCMEx_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{  
  */
#define __DAL_RCM_TMR6_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR6EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR6EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR7_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR7EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR7EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR12_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR12EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR12EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR13_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR13EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR13EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR14_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR14EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR14EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_USART3_CLK_ENABLE() do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_USART3EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_USART3EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_UART4_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART4EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART4EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_UART5_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART5EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART5EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_CAN1_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN1EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN1EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_CAN2_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN2EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN2EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_DAC_CLK_ENABLE()    do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_DACEN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_DACEN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR2_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR2EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR2EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR3_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR3EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR3EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR4_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR4EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR4EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_SPI3_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI3EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI3EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_I2C3_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C3EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C3EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR2_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR2EN))
#define __DAL_RCM_TMR3_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR3EN))
#define __DAL_RCM_TMR4_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR4EN))
#define __DAL_RCM_SPI3_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_SPI3EN))
#define __DAL_RCM_I2C3_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_I2C3EN))
#define __DAL_RCM_TMR6_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR6EN))
#define __DAL_RCM_TMR7_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR7EN))
#define __DAL_RCM_TMR12_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR12EN))
#define __DAL_RCM_TMR13_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR13EN))
#define __DAL_RCM_TMR14_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR14EN))
#define __DAL_RCM_USART3_CLK_DISABLE() (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_USART3EN))
#define __DAL_RCM_UART4_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_UART4EN))
#define __DAL_RCM_UART5_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_UART5EN))
#define __DAL_RCM_CAN1_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_CAN1EN))
#define __DAL_RCM_CAN2_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_CAN2EN))
#define __DAL_RCM_DAC_CLK_DISABLE()    (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_DACEN))
/**
  * @}
  */
 
/** @defgroup RCMEx_APB1_Peripheral_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */ 
#define __DAL_RCM_TMR2_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR2EN)) != RESET)  
#define __DAL_RCM_TMR3_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR3EN)) != RESET) 
#define __DAL_RCM_TMR4_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR4EN)) != RESET)
#define __DAL_RCM_SPI3_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_SPI3EN)) != RESET) 
#define __DAL_RCM_I2C3_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C3EN)) != RESET) 
#define __DAL_RCM_TMR6_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR6EN)) != RESET) 
#define __DAL_RCM_TMR7_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR7EN)) != RESET) 
#define __DAL_RCM_TMR12_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR12EN)) != RESET) 
#define __DAL_RCM_TMR13_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR13EN)) != RESET) 
#define __DAL_RCM_TMR14_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR14EN)) != RESET) 
#define __DAL_RCM_USART3_IS_CLK_ENABLED() ((RCM->APB1CLKEN & (RCM_APB1CLKEN_USART3EN)) != RESET) 
#define __DAL_RCM_UART4_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART4EN)) != RESET) 
#define __DAL_RCM_UART5_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART5EN)) != RESET) 
#define __DAL_RCM_CAN1_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN1EN)) != RESET) 
#define __DAL_RCM_CAN2_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN2EN)) != RESET) 
#define __DAL_RCM_DAC_IS_CLK_ENABLED()    ((RCM->APB1CLKEN & (RCM_APB1CLKEN_DACEN)) != RESET) 

#define __DAL_RCM_TMR2_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR2EN)) == RESET)  
#define __DAL_RCM_TMR3_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR3EN)) == RESET) 
#define __DAL_RCM_TMR4_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR4EN)) == RESET)
#define __DAL_RCM_SPI3_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_SPI3EN)) == RESET) 
#define __DAL_RCM_I2C3_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C3EN)) == RESET) 
#define __DAL_RCM_TMR6_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR6EN)) == RESET) 
#define __DAL_RCM_TMR7_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR7EN)) == RESET) 
#define __DAL_RCM_TMR12_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR12EN)) == RESET) 
#define __DAL_RCM_TMR13_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR13EN)) == RESET) 
#define __DAL_RCM_TMR14_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR14EN)) == RESET) 
#define __DAL_RCM_USART3_IS_CLK_DISABLED() ((RCM->APB1CLKEN & (RCM_APB1CLKEN_USART3EN)) == RESET) 
#define __DAL_RCM_UART4_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART4EN)) == RESET) 
#define __DAL_RCM_UART5_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART5EN)) == RESET) 
#define __DAL_RCM_CAN1_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN1EN)) == RESET) 
#define __DAL_RCM_CAN2_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN2EN)) == RESET) 
#define __DAL_RCM_DAC_IS_CLK_DISABLED()    ((RCM->APB1CLKEN & (RCM_APB1CLKEN_DACEN)) == RESET) 
  /**
  * @}
  */
  
/** @defgroup RCMEx_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */ 
#define __DAL_RCM_TMR8_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR8EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR8EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_ADC2_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC2EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC2EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_ADC3_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC3EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC3EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_SDIO_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SDIOEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SDIOEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR10_CLK_ENABLE()    do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR10EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR10EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __DAL_RCM_SDIO_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_SDIOEN))
#define __DAL_RCM_TMR10_CLK_DISABLE()  (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_TMR10EN))
#define __DAL_RCM_TMR8_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_TMR8EN))
#define __DAL_RCM_ADC2_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_ADC2EN))
#define __DAL_RCM_ADC3_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_ADC3EN))
/**
  * @}
  */

/** @defgroup RCMEx_APB2_Peripheral_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_SDIO_IS_CLK_ENABLED()    ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SDIOEN)) != RESET)  
#define __DAL_RCM_TMR10_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR10EN)) != RESET) 
#define __DAL_RCM_TMR8_IS_CLK_ENABLED()    ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR8EN)) != RESET) 
#define __DAL_RCM_ADC2_IS_CLK_ENABLED()    ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC2EN)) != RESET) 
#define __DAL_RCM_ADC3_IS_CLK_ENABLED()    ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC3EN)) != RESET)
  
#define __DAL_RCM_SDIO_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SDIOEN)) == RESET)  
#define __DAL_RCM_TMR10_IS_CLK_DISABLED()  ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR10EN)) == RESET) 
#define __DAL_RCM_TMR8_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR8EN)) == RESET) 
#define __DAL_RCM_ADC2_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC2EN)) == RESET) 
#define __DAL_RCM_ADC3_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC3EN)) == RESET)
/**
  * @}
  */
    
/** @defgroup RCMEx_AHB1_Force_Release_Reset AHB1 Force Release Reset 
  * @brief  Force or release AHB1 peripheral reset.
  * @{
  */
#define __DAL_RCM_GPIOD_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_PDRST))
#define __DAL_RCM_GPIOE_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_PERST))
#define __DAL_RCM_GPIOF_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_PFRST))
#define __DAL_RCM_GPIOG_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_PGRST))
#define __DAL_RCM_GPIOI_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_PIRST))
#define __DAL_RCM_ETHMAC_FORCE_RESET()   (RCM->AHB1RST |= (RCM_AHB1RST_ETHRST))
#define __DAL_RCM_USB_OTG_HS_FORCE_RESET()    (RCM->AHB1RST |= (RCM_AHB1RST_OTGHS1RST))
#define __DAL_RCM_CRC_FORCE_RESET()     (RCM->AHB1RST |= (RCM_AHB1RST_CRCRST))

#define __DAL_RCM_GPIOD_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_PDRST))
#define __DAL_RCM_GPIOE_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_PERST))
#define __DAL_RCM_GPIOF_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_PFRST))
#define __DAL_RCM_GPIOG_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_PGRST))
#define __DAL_RCM_GPIOI_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_PIRST))
#define __DAL_RCM_ETHMAC_RELEASE_RESET() (RCM->AHB1RST &= ~(RCM_AHB1RST_ETHRST))
#define __DAL_RCM_USB_OTG_HS_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_OTGHS1RST))
#define __DAL_RCM_CRC_RELEASE_RESET()    (RCM->AHB1RST &= ~(RCM_AHB1RST_CRCRST))
/**
  * @}
  */

/** @defgroup RCMEx_AHB2_Force_Release_Reset AHB2 Force Release Reset 
  * @brief  Force or release AHB2 peripheral reset.
  * @{
  */
#define __DAL_RCM_AHB2_FORCE_RESET()         (RCM->AHB2RST = 0xFFFFFFFFU) 
#define __DAL_RCM_AHB2_RELEASE_RESET()       (RCM->AHB2RST = 0x00U)

#if defined(APM32F407xx) || defined(APM32F417xx)
#define __DAL_RCM_DCI_FORCE_RESET()   (RCM->AHB2RST |= (RCM_AHB2RST_DCIRST))
#define __DAL_RCM_DCI_RELEASE_RESET() (RCM->AHB2RST &= ~(RCM_AHB2RST_DCIRST))
#endif /* APM32F407xx || APM32F417xx */

#if defined(APM32F417xx) 
#define __DAL_RCM_CRYP_FORCE_RESET()   (RCM->AHB2RST |= (RCM_AHB2RST_CRYPRST))
#define __DAL_RCM_HASH_FORCE_RESET()   (RCM->AHB2RST |= (RCM_AHB2RST_HASHRST))

#define __DAL_RCM_CRYP_RELEASE_RESET() (RCM->AHB2RST &= ~(RCM_AHB2RST_CRYPRST))
#define __DAL_RCM_HASH_RELEASE_RESET() (RCM->AHB2RST &= ~(RCM_AHB2RST_HASHRST))
#endif /* APM32F417xx */
   
#define __DAL_RCM_USB_OTG_FS_FORCE_RESET()   (RCM->AHB2RST |= (RCM_AHB2RST_OTGFSRST))
#define __DAL_RCM_USB_OTG_FS_RELEASE_RESET() (RCM->AHB2RST &= ~(RCM_AHB2RST_OTGFSRST))

#define __DAL_RCM_RNG_FORCE_RESET()    (RCM->AHB2RST |= (RCM_AHB2RST_RNGRST))
#define __DAL_RCM_RNG_RELEASE_RESET()  (RCM->AHB2RST &= ~(RCM_AHB2RST_RNGRST))
/**
  * @}
  */

/** @defgroup RCMEx_AHB3_Force_Release_Reset AHB3 Force Release Reset 
  * @brief  Force or release AHB3 peripheral reset.
  * @{
  */ 
#define __DAL_RCM_AHB3_FORCE_RESET() (RCM->AHB3RST = 0xFFFFFFFFU)
#define __DAL_RCM_AHB3_RELEASE_RESET() (RCM->AHB3RST = 0x00U) 

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define __DAL_RCM_EMMC_FORCE_RESET()   (RCM->AHB3RST |= (RCM_AHB3RST_EMMCRST))
#define __DAL_RCM_EMMC_RELEASE_RESET() (RCM->AHB3RST &= ~(RCM_AHB3RST_EMMCRST))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

/**
  * @}
  */

/** @defgroup RCMEx_APB1_Force_Release_Reset APB1 Force Release Reset 
  * @brief  Force or release APB1 peripheral reset.
  * @{
  */
#define __DAL_RCM_TMR6_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR6RST))
#define __DAL_RCM_TMR7_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR7RST))
#define __DAL_RCM_TMR12_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_TMR12RST))
#define __DAL_RCM_TMR13_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_TMR13RST))
#define __DAL_RCM_TMR14_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_TMR14RST))
#define __DAL_RCM_USART3_FORCE_RESET()   (RCM->APB1RST |= (RCM_APB1RST_USART3RST))
#define __DAL_RCM_UART4_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_UART4RST))
#define __DAL_RCM_UART5_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_UART5RST))
#define __DAL_RCM_CAN1_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_CAN1RST))
#define __DAL_RCM_CAN2_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_CAN2RST))
#define __DAL_RCM_DAC_FORCE_RESET()      (RCM->APB1RST |= (RCM_APB1RST_DACRST))
#define __DAL_RCM_TMR2_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR2RST))
#define __DAL_RCM_TMR3_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR3RST))
#define __DAL_RCM_TMR4_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR4RST))
#define __DAL_RCM_SPI3_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_SPI3RST))
#define __DAL_RCM_I2C3_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_I2C3RST))

#define __DAL_RCM_TMR2_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR2RST))
#define __DAL_RCM_TMR3_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR3RST))
#define __DAL_RCM_TMR4_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR4RST))
#define __DAL_RCM_SPI3_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_SPI3RST))
#define __DAL_RCM_I2C3_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_I2C3RST))
#define __DAL_RCM_TMR6_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR6RST))
#define __DAL_RCM_TMR7_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR7RST))
#define __DAL_RCM_TMR12_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_TMR12RST))
#define __DAL_RCM_TMR13_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_TMR13RST))
#define __DAL_RCM_TMR14_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_TMR14RST))
#define __DAL_RCM_USART3_RELEASE_RESET() (RCM->APB1RST &= ~(RCM_APB1RST_USART3RST))
#define __DAL_RCM_UART4_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_UART4RST))
#define __DAL_RCM_UART5_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_UART5RST))
#define __DAL_RCM_CAN1_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_CAN1RST))
#define __DAL_RCM_CAN2_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_CAN2RST))
#define __DAL_RCM_DAC_RELEASE_RESET()    (RCM->APB1RST &= ~(RCM_APB1RST_DACRST))
/**
  * @}
  */

/** @defgroup RCMEx_APB2_Force_Release_Reset APB2 Force Release Reset 
  * @brief  Force or release APB2 peripheral reset.
  * @{
  */
#define __DAL_RCM_TMR8_FORCE_RESET()   (RCM->APB2RST |= (RCM_APB2RST_TMR8RST))
#define __DAL_RCM_SDIO_FORCE_RESET()   (RCM->APB2RST |= (RCM_APB2RST_SDIORST))
#define __DAL_RCM_TMR10_FORCE_RESET()  (RCM->APB2RST |= (RCM_APB2RST_TMR10RST))
                                          
#define __DAL_RCM_SDIO_RELEASE_RESET() (RCM->APB2RST &= ~(RCM_APB2RST_SDIORST))
#define __DAL_RCM_TMR10_RELEASE_RESET()(RCM->APB2RST &= ~(RCM_APB2RST_TMR10RST))
#define __DAL_RCM_TMR8_RELEASE_RESET() (RCM->APB2RST &= ~(RCM_APB2RST_TMR8RST))
/**
  * @}
  */
                                        
/** @defgroup RCMEx_AHB1_LowPower_Enable_Disable AHB1 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_GPIOD_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PDLPEN))
#define __DAL_RCM_GPIOE_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PELPEN))
#define __DAL_RCM_GPIOF_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PFLPEN))
#define __DAL_RCM_GPIOG_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PGLPEN))
#define __DAL_RCM_GPIOI_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PILPEN))
#define __DAL_RCM_SRAM2_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_SRAM2EN))

#if defined(APM32F407xx) || defined(APM32F417xx)
#define __DAL_RCM_ETHMAC_CLK_SLEEP_ENABLE()     (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_ETHEN))
#define __DAL_RCM_ETHMACTX_CLK_SLEEP_ENABLE()   (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_ETHTXEN))
#define __DAL_RCM_ETHMACRX_CLK_SLEEP_ENABLE()   (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_ETHRXEN))
#define __DAL_RCM_ETHMACPTP_CLK_SLEEP_ENABLE()  (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_ETHPTPEN))
#endif /* APM32F407xx || APM32F417xx */

#define __DAL_RCM_USB_OTG_HS_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_OTGHS1EN))
#define __DAL_RCM_USB_OTG_HS_ULPI_CLK_SLEEP_ENABLE()  (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_OTGHSULPIEN))
#define __DAL_RCM_CRC_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_CRCLPEN))
#define __DAL_RCM_FLITF_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_FMCEN))
#define __DAL_RCM_SRAM1_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_SRAM1EN))
#define __DAL_RCM_BKPSRAM_CLK_SLEEP_ENABLE()  (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_BKPSRAMEN))

#define __DAL_RCM_GPIOD_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PDLPEN))
#define __DAL_RCM_GPIOE_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PELPEN))
#define __DAL_RCM_GPIOF_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PFLPEN))
#define __DAL_RCM_GPIOG_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PGLPEN))
#define __DAL_RCM_GPIOI_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PILPEN))
#define __DAL_RCM_SRAM2_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_SRAM2EN))

#if defined(APM32F407xx) || defined(APM32F417xx)
#define __DAL_RCM_ETHMAC_CLK_SLEEP_DISABLE()    (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_ETHEN))
#define __DAL_RCM_ETHMACTX_CLK_SLEEP_DISABLE()  (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_ETHTXEN))
#define __DAL_RCM_ETHMACRX_CLK_SLEEP_DISABLE()  (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_ETHRXEN))
#define __DAL_RCM_ETHMACPTP_CLK_SLEEP_DISABLE() (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_ETHPTPEN))
#endif /* APM32F407xx || APM32F417xx */

#define __DAL_RCM_USB_OTG_HS_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_OTGHS1EN))
#define __DAL_RCM_USB_OTG_HS_ULPI_CLK_SLEEP_DISABLE() (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_OTGHSULPIEN))
#define __DAL_RCM_CRC_CLK_SLEEP_DISABLE()       (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_CRCLPEN))
#define __DAL_RCM_FLITF_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_FMCEN))
#define __DAL_RCM_SRAM1_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_SRAM1EN))
#define __DAL_RCM_BKPSRAM_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_BKPSRAMEN))
/**
  * @}
  */

/** @defgroup RCMEx_AHB2_LowPower_Enable_Disable AHB2 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the AHB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_USB_OTG_FS_CLK_SLEEP_ENABLE()  (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_OTGFSEN))
#define __DAL_RCM_USB_OTG_FS_CLK_SLEEP_DISABLE() (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_OTGFSEN))

#define __DAL_RCM_RNG_CLK_SLEEP_ENABLE()   (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_RNGEN))
#define __DAL_RCM_RNG_CLK_SLEEP_DISABLE()  (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_RNGEN))

#if defined(APM32F407xx) || defined(APM32F417xx) 
#define __DAL_RCM_DCI_CLK_SLEEP_ENABLE()  (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_DCIEN))
#define __DAL_RCM_DCI_CLK_SLEEP_DISABLE() (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_DCIEN))
#endif /* APM32F407xx || APM32F417xx */

#if defined(APM32F417xx) 
#define __DAL_RCM_CRYP_CLK_SLEEP_ENABLE()  (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_CRYPEN))
#define __DAL_RCM_HASH_CLK_SLEEP_ENABLE()  (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_HASHEN))

#define __DAL_RCM_CRYP_CLK_SLEEP_DISABLE() (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_CRYPEN))
#define __DAL_RCM_HASH_CLK_SLEEP_DISABLE() (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_HASHEN))
#endif /* APM32F417xx */
/**
  * @}
  */
                                        
/** @defgroup RCMEx_AHB3_LowPower_Enable_Disable AHB3 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the AHB3 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define __DAL_RCM_EMMC_CLK_SLEEP_ENABLE()  (RCM->LPAHB3CLKEN |= (RCM_LPAHB3CLKEN_EMMCEN))
#define __DAL_RCM_EMMC_CLK_SLEEP_DISABLE() (RCM->LPAHB3CLKEN &= ~(RCM_LPAHB3CLKEN_EMMCEN))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */
/**
  * @}
  */
                                        
/** @defgroup RCMEx_APB1_LowPower_Enable_Disable APB1 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_TMR6_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR6EN))
#define __DAL_RCM_TMR7_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR7EN))
#define __DAL_RCM_TMR12_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR12EN))
#define __DAL_RCM_TMR13_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR13EN))
#define __DAL_RCM_TMR14_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR14EN))
#define __DAL_RCM_USART3_CLK_SLEEP_ENABLE()  (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_USART3EN))
#define __DAL_RCM_UART4_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_UART4EN))
#define __DAL_RCM_UART5_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_UART5EN))
#define __DAL_RCM_CAN1_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_CAN1EN))
#define __DAL_RCM_CAN2_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_CAN2EN))
#define __DAL_RCM_DAC_CLK_SLEEP_ENABLE()     (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_DACEN))
#define __DAL_RCM_TMR2_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR2EN))
#define __DAL_RCM_TMR3_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR3EN))
#define __DAL_RCM_TMR4_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR4EN))
#define __DAL_RCM_SPI3_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_SPI3EN))
#define __DAL_RCM_I2C3_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_I2C3EN))

#define __DAL_RCM_TMR2_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR2EN))
#define __DAL_RCM_TMR3_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR3EN))
#define __DAL_RCM_TMR4_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR4EN))
#define __DAL_RCM_SPI3_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_SPI3EN))
#define __DAL_RCM_I2C3_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_I2C3EN))
#define __DAL_RCM_TMR6_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR6EN))
#define __DAL_RCM_TMR7_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR7EN))
#define __DAL_RCM_TMR12_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR12EN))
#define __DAL_RCM_TMR13_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR13EN))
#define __DAL_RCM_TMR14_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR14EN))
#define __DAL_RCM_USART3_CLK_SLEEP_DISABLE() (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_USART3EN))
#define __DAL_RCM_UART4_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_UART4EN))
#define __DAL_RCM_UART5_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_UART5EN))
#define __DAL_RCM_CAN1_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_CAN1EN))
#define __DAL_RCM_CAN2_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_CAN2EN))
#define __DAL_RCM_DAC_CLK_SLEEP_DISABLE()    (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_DACEN))
/**
  * @}
  */
                                        
/** @defgroup RCMEx_APB2_LowPower_Enable_Disable APB2 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_TMR8_CLK_SLEEP_ENABLE() (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_TMR8EN))
#define __DAL_RCM_ADC2_CLK_SLEEP_ENABLE() (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_ADC2EN))
#define __DAL_RCM_ADC3_CLK_SLEEP_ENABLE() (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_ADC3EN))
#define __DAL_RCM_SDIO_CLK_SLEEP_ENABLE() (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_SDIOEN))
#define __DAL_RCM_TMR10_CLK_SLEEP_ENABLE()(RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_TMR10EN))

#define __DAL_RCM_SDIO_CLK_SLEEP_DISABLE() (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_SDIOEN))
#define __DAL_RCM_TMR10_CLK_SLEEP_DISABLE()(RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_TMR10EN))
#define __DAL_RCM_TMR8_CLK_SLEEP_DISABLE() (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_TMR8EN))
#define __DAL_RCM_ADC2_CLK_SLEEP_DISABLE() (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_ADC2EN))
#define __DAL_RCM_ADC3_CLK_SLEEP_DISABLE() (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_ADC3EN))
/**
  * @}
  */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */
/*----------------------------------------------------------------------------*/

/*-------------------------------- APM32F411xx -------------------------------*/
#if defined(APM32F411xx)
/** @defgroup RCMEx_AHB1_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
  * @brief  Enables or disables the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */
#define __DAL_RCM_GPIOD_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PDEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PDEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_GPIOE_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PEEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_PEEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_CRC_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_CRCEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB1CLKEN, RCM_AHB1CLKEN_CRCEN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_GPIOD_CLK_DISABLE()           (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PDEN))
#define __DAL_RCM_GPIOE_CLK_DISABLE()           (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_PEEN))
#define __DAL_RCM_CRC_CLK_DISABLE()             (RCM->AHB1CLKEN &= ~(RCM_AHB1CLKEN_CRCEN))
/**
  * @}
  */

/** @defgroup RCMEx_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */  
#define __DAL_RCM_GPIOD_IS_CLK_ENABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PDEN)) != RESET) 
#define __DAL_RCM_GPIOE_IS_CLK_ENABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PEEN)) != RESET) 
#define __DAL_RCM_CRC_IS_CLK_ENABLED()             ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_CRCEN)) != RESET) 

#define __DAL_RCM_GPIOD_IS_CLK_DISABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PDEN)) == RESET) 
#define __DAL_RCM_GPIOE_IS_CLK_DISABLED()           ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_PEEN)) == RESET) 
#define __DAL_RCM_CRC_IS_CLK_DISABLED()             ((RCM->AHB1CLKEN & (RCM_AHB1CLKEN_CRCEN)) == RESET) 
/**
  * @}
  */
  
/** @defgroup RCMEX_AHB2_Clock_Enable_Disable AHB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */
#define __DAL_RCM_USB_OTG_FS_CLK_ENABLE()  do {(RCM->AHB2CLKEN |= (RCM_AHB2CLKEN_OTGFSEN));\
                                               __DAL_RCM_SYSCFG_CLK_ENABLE();\
                                              }while(0U)

#define __DAL_RCM_USB_OTG_FS_CLK_DISABLE() (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_OTGFSEN))

#define __DAL_RCM_RNG_CLK_ENABLE()    do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_RNGEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_RNGEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_RNG_CLK_DISABLE()   (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_RNGEN))

#define __DAL_RCM_SMC_CLK_ENABLE()    do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_SMCEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_SMCEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_SMC_CLK_DISABLE()   (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_SMCEN))

#define __DAL_RCM_QSPI_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_QSPIEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->AHB2CLKEN, RCM_AHB2CLKEN_QSPIEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_QSPI_CLK_DISABLE()  (RCM->AHB2CLKEN &= ~(RCM_AHB2CLKEN_QSPIEN))
/**
  * @}
  */

/** @defgroup RCMEx_AHB2_Peripheral_Clock_Enable_Disable_Status AHB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_USB_OTG_FS_IS_CLK_ENABLED()  ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_OTGFSEN)) != RESET)
#define __DAL_RCM_USB_OTG_FS_IS_CLK_DISABLED() ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_OTGFSEN)) == RESET)

#define __DAL_RCM_RNG_IS_CLK_ENABLED()         ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_RNGEN)) != RESET)
#define __DAL_RCM_RNG_IS_CLK_DISABLED()        ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_RNGEN)) == RESET)

#define __DAL_RCM_SMC_IS_CLK_ENABLED()         ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_SMCEN)) != RESET)
#define __DAL_RCM_SMC_IS_CLK_DISABLED()        ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_SMCEN)) == RESET)

#define __DAL_RCM_QSPI_IS_CLK_ENABLED()        ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_QSPIEN)) != RESET)
#define __DAL_RCM_QSPI_IS_CLK_DISABLED()       ((RCM->AHB2CLKEN & (RCM_AHB2CLKEN_QSPIEN)) == RESET)
/**
  * @}
  */  

/** @defgroup RCMEx_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it. 
  * @{
  */
#define __DAL_RCM_TMR12_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR12EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR12EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR13_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR13EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR13EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR14_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR14EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR14EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_USART3_CLK_ENABLE() do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_USART3EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_USART3EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_UART4_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART4EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART4EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_UART5_CLK_ENABLE()  do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART5EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_UART5EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_CAN1_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN1EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN1EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_CAN2_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN2EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN2EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR2_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR2EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR2EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_TMR3_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR3EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR3EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_TMR4_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR4EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR4EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_SPI3_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI3EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI3EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)
#define __DAL_RCM_I2C3_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C3EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB1CLKEN, RCM_APB1CLKEN_I2C3EN);\
                                        UNUSED(tmpreg); \
                                        } while(0U)

#define __DAL_RCM_TMR12_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR12EN))
#define __DAL_RCM_TMR13_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR13EN))
#define __DAL_RCM_TMR14_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR14EN))
#define __DAL_RCM_USART3_CLK_DISABLE() (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_USART3EN))
#define __DAL_RCM_UART4_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_UART4EN))
#define __DAL_RCM_UART5_CLK_DISABLE()  (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_UART5EN))
#define __DAL_RCM_CAN1_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_CAN1EN))
#define __DAL_RCM_CAN2_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_CAN2EN))
#define __DAL_RCM_TMR2_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR2EN))
#define __DAL_RCM_TMR3_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR3EN))
#define __DAL_RCM_TMR4_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_TMR4EN))
#define __DAL_RCM_SPI3_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_SPI3EN))
#define __DAL_RCM_I2C3_CLK_DISABLE()   (RCM->APB1CLKEN &= ~(RCM_APB1CLKEN_I2C3EN))
/**
  * @}
  */ 
  
/** @defgroup RCMEx_APB1_Peripheral_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_TMR12_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR12EN)) != RESET)
#define __DAL_RCM_TMR13_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR13EN)) != RESET)
#define __DAL_RCM_TMR14_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR14EN)) != RESET)
#define __DAL_RCM_USART3_IS_CLK_ENABLED() ((RCM->APB1CLKEN & (RCM_APB1CLKEN_USART3EN)) != RESET)
#define __DAL_RCM_UART4_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART4EN)) != RESET)
#define __DAL_RCM_UART5_IS_CLK_ENABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART5EN)) != RESET)
#define __DAL_RCM_CAN1_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN1EN)) != RESET)
#define __DAL_RCM_CAN2_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN2EN)) != RESET)
#define __DAL_RCM_TMR2_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR2EN)) != RESET) 
#define __DAL_RCM_TMR3_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR3EN)) != RESET) 
#define __DAL_RCM_TMR4_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR4EN)) != RESET) 
#define __DAL_RCM_SPI3_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_SPI3EN)) != RESET) 
#define __DAL_RCM_I2C3_IS_CLK_ENABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C3EN)) != RESET)

#define __DAL_RCM_TMR12_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR12EN)) == RESET)
#define __DAL_RCM_TMR13_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR13EN)) == RESET)
#define __DAL_RCM_TMR14_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR14EN)) == RESET)
#define __DAL_RCM_USART3_IS_CLK_DISABLED() ((RCM->APB1CLKEN & (RCM_APB1CLKEN_USART3EN)) == RESET)
#define __DAL_RCM_UART4_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART4EN)) == RESET)
#define __DAL_RCM_UART5_IS_CLK_DISABLED()  ((RCM->APB1CLKEN & (RCM_APB1CLKEN_UART5EN)) == RESET)
#define __DAL_RCM_CAN1_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN1EN)) == RESET)
#define __DAL_RCM_CAN2_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_CAN2EN)) == RESET)
#define __DAL_RCM_TMR2_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR2EN)) == RESET) 
#define __DAL_RCM_TMR3_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR3EN)) == RESET) 
#define __DAL_RCM_TMR4_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_TMR4EN)) == RESET) 
#define __DAL_RCM_SPI3_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_SPI3EN)) == RESET) 
#define __DAL_RCM_I2C3_IS_CLK_DISABLED()   ((RCM->APB1CLKEN & (RCM_APB1CLKEN_I2C3EN)) == RESET) 
/**
  * @}
  */ 
  
/** @defgroup RCMEx_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @{
  */
#define __DAL_RCM_TMR8_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR8EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR8EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_ADC2_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC2EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC2EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_SPI5_CLK_ENABLE()   do { \
                                      __IO uint32_t tmpreg = 0x00U; \
                                      SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SPI5EN);\
                                      /* Delay after an RCM peripheral clock enabling */ \
                                      tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SPI5EN);\
                                      UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_SDIO_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SDIOEN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SDIOEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_SPI4_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SPI4EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_SPI4EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_TMR10_CLK_ENABLE()    do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR10EN);\
                                        /* Delay after an RCM peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR10EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
#define __DAL_RCM_SDIO_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_SDIOEN))
#define __DAL_RCM_SPI4_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_SPI4EN))
#define __DAL_RCM_TMR10_CLK_DISABLE()  (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_TMR10EN))
#define __DAL_RCM_SPI5_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_SPI5EN))
#define __DAL_RCM_TMR8_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_TMR8EN))
#define __DAL_RCM_ADC2_CLK_DISABLE()   (RCM->APB2CLKEN &= ~(RCM_APB2CLKEN_ADC2EN))
/**
  * @}
  */
  
/** @defgroup RCMEx_APB2_Peripheral_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
#define __DAL_RCM_SDIO_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SDIOEN)) != RESET)  
#define __DAL_RCM_SPI4_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SPI4EN)) != RESET)   
#define __DAL_RCM_TMR10_IS_CLK_ENABLED()  ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR10EN)) != RESET)  
#define __DAL_RCM_SPI5_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SPI5EN)) != RESET) 
#define __DAL_RCM_TMR8_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR8EN)) != RESET)
#define __DAL_RCM_ADC2_IS_CLK_ENABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC2EN)) != RESET)

#define __DAL_RCM_SDIO_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SDIOEN)) == RESET)  
#define __DAL_RCM_SPI4_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SPI4EN)) == RESET)   
#define __DAL_RCM_TMR10_IS_CLK_DISABLED()  ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR10EN)) == RESET)  
#define __DAL_RCM_SPI5_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_SPI5EN)) == RESET)   
#define __DAL_RCM_TMR8_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_TMR8EN)) == RESET)
#define __DAL_RCM_ADC2_IS_CLK_DISABLED()   ((RCM->APB2CLKEN & (RCM_APB2CLKEN_ADC2EN)) == RESET)
/**
  * @}
  */  
  
/** @defgroup RCMEx_AHB1_Force_Release_Reset AHB1 Force Release Reset 
  * @brief  Force or release AHB1 peripheral reset.
  * @{
  */ 
#define __DAL_RCM_GPIOD_FORCE_RESET()   (RCM->AHB1RST |= (RCM_AHB1RST_PDRST))
#define __DAL_RCM_GPIOE_FORCE_RESET()   (RCM->AHB1RST |= (RCM_AHB1RST_PERST))
#define __DAL_RCM_CRC_FORCE_RESET()     (RCM->AHB1RST |= (RCM_AHB1RST_CRCRST))

#define __DAL_RCM_GPIOD_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_PDRST))
#define __DAL_RCM_GPIOE_RELEASE_RESET()  (RCM->AHB1RST &= ~(RCM_AHB1RST_PERST))
#define __DAL_RCM_CRC_RELEASE_RESET()    (RCM->AHB1RST &= ~(RCM_AHB1RST_CRCRST))
/**
  * @}
  */

/** @defgroup RCMEx_AHB2_Force_Release_Reset AHB2 Force Release Reset 
  * @brief  Force or release AHB2 peripheral reset.
  * @{
  */
#define __DAL_RCM_AHB2_FORCE_RESET()    (RCM->AHB2RST = 0xFFFFFFFFU) 
#define __DAL_RCM_USB_OTG_FS_FORCE_RESET()   (RCM->AHB2RST |= (RCM_AHB2RST_OTGFSRST))

#define __DAL_RCM_AHB2_RELEASE_RESET()  (RCM->AHB2RST = 0x00U)
#define __DAL_RCM_USB_OTG_FS_RELEASE_RESET() (RCM->AHB2RST &= ~(RCM_AHB2RST_OTGFSRST))

#define __DAL_RCM_RNG_FORCE_RESET()     (RCM->AHB2RST |= (RCM_AHB2RST_RNGRST))
#define __DAL_RCM_RNG_RELEASE_RESET()   (RCM->AHB2RST &= ~(RCM_AHB2RST_RNGRST))

#define __DAL_RCM_SMC_FORCE_RESET()     (RCM->AHB2RST |= (RCM_AHB2RST_SMCRST))
#define __DAL_RCM_SMC_RELEASE_RESET()   (RCM->AHB2RST &= ~(RCM_AHB2RST_SMCRST))

#define __DAL_RCM_QSPI_FORCE_RESET()    (RCM->AHB2RST |= (RCM_AHB2RST_QSPIRST))
#define __DAL_RCM_QSPI_RELEASE_RESET()  (RCM->AHB2RST &= ~(RCM_AHB2RST_QSPIRST))
/**
  * @}
  */

/** @defgroup RCMEx_AHB3_Force_Release_Reset AHB3 Force Release Reset 
  * @brief  Force or release AHB3 peripheral reset.
  * @{
  */ 
#define __DAL_RCM_AHB3_FORCE_RESET() (RCM->AHB3RST = 0xFFFFFFFFU)
#define __DAL_RCM_AHB3_RELEASE_RESET() (RCM->AHB3RST = 0x00U) 
/**
  * @}
  */

/** @defgroup RCMEx_APB1_Force_Release_Reset APB1 Force Release Reset 
  * @brief  Force or release APB1 peripheral reset.
  * @{
  */
#define __DAL_RCM_TMR2_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR2RST))
#define __DAL_RCM_TMR3_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR3RST))
#define __DAL_RCM_TMR4_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_TMR4RST))
#define __DAL_RCM_SPI3_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_SPI3RST))
#define __DAL_RCM_I2C3_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_I2C3RST))
#define __DAL_RCM_TMR12_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_TMR12RST))
#define __DAL_RCM_TMR13_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_TMR13RST))
#define __DAL_RCM_TMR14_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_TMR14RST))
#define __DAL_RCM_USART3_FORCE_RESET()   (RCM->APB1RST |= (RCM_APB1RST_USART3RST))
#define __DAL_RCM_UART4_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_UART4RST))
#define __DAL_RCM_UART5_FORCE_RESET()    (RCM->APB1RST |= (RCM_APB1RST_UART5RST))
#define __DAL_RCM_CAN1_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_CAN1RST))
#define __DAL_RCM_CAN2_FORCE_RESET()     (RCM->APB1RST |= (RCM_APB1RST_CAN2RST))

#define __DAL_RCM_TMR2_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR2RST))
#define __DAL_RCM_TMR3_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR3RST))
#define __DAL_RCM_TMR4_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_TMR4RST))
#define __DAL_RCM_SPI3_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_SPI3RST))
#define __DAL_RCM_I2C3_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_I2C3RST))
#define __DAL_RCM_TMR12_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_TMR12RST))
#define __DAL_RCM_TMR13_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_TMR13RST))
#define __DAL_RCM_TMR14_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_TMR14RST))
#define __DAL_RCM_USART3_RELEASE_RESET() (RCM->APB1RST &= ~(RCM_APB1RST_USART3RST))
#define __DAL_RCM_UART4_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_UART4RST))
#define __DAL_RCM_UART5_RELEASE_RESET()  (RCM->APB1RST &= ~(RCM_APB1RST_UART5RST))
#define __DAL_RCM_CAN1_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_CAN1RST))
#define __DAL_RCM_CAN2_RELEASE_RESET()   (RCM->APB1RST &= ~(RCM_APB1RST_CAN2RST))
/**
  * @}
  */

/** @defgroup RCMEx_APB2_Force_Release_Reset APB2 Force Release Reset 
  * @brief  Force or release APB2 peripheral reset.
  * @{
  */
#define __DAL_RCM_SPI5_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_SPI5RST))
#define __DAL_RCM_SDIO_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_SDIORST))
#define __DAL_RCM_SPI4_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_SPI4RST))
#define __DAL_RCM_TMR10_FORCE_RESET()    (RCM->APB2RST |= (RCM_APB2RST_TMR10RST))
#define __DAL_RCM_TMR8_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_TMR8RST))
#define __DAL_RCM_ADC2_FORCE_RESET()     (RCM->APB2RST |= (RCM_APB2RST_ADC2RST))

#define __DAL_RCM_SDIO_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_SDIORST))
#define __DAL_RCM_SPI4_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_SPI4RST))
#define __DAL_RCM_TMR10_RELEASE_RESET()  (RCM->APB2RST &= ~(RCM_APB2RST_TMR10RST))
#define __DAL_RCM_SPI5_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_SPI5RST))
#define __DAL_RCM_TMR8_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_TMR8RST))
#define __DAL_RCM_ADC2_RELEASE_RESET()   (RCM->APB2RST &= ~(RCM_APB2RST_ADC2RST))
/**
  * @}
  */

/** @defgroup RCMEx_AHB1_LowPower_Enable_Disable AHB1 Peripheral Low Power Enable Disable 
  * @brief  Enable or disable the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_GPIOD_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PDLPEN))
#define __DAL_RCM_GPIOE_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_PELPEN))
#define __DAL_RCM_CRC_CLK_SLEEP_ENABLE()      (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_CRCLPEN))
#define __DAL_RCM_FLITF_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_FMCEN))
#define __DAL_RCM_SRAM1_CLK_SLEEP_ENABLE()    (RCM->LPAHB1CLKEN |= (RCM_LPAHB1CLKEN_SRAM1EN))
                                        
#define __DAL_RCM_GPIOD_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PDLPEN))                                        
#define __DAL_RCM_GPIOE_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_PELPEN))
#define __DAL_RCM_CRC_CLK_SLEEP_DISABLE()     (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_CRCLPEN))
#define __DAL_RCM_FLITF_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_FMCEN))
#define __DAL_RCM_SRAM1_CLK_SLEEP_DISABLE()   (RCM->LPAHB1CLKEN &= ~(RCM_LPAHB1CLKEN_SRAM1EN))
/**
  * @}
  */

/** @defgroup RCMEx_AHB2_LowPower_Enable_Disable AHB2 Peripheral Low Power Enable Disable
  * @brief  Enable or disable the AHB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wake-up from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
#define __DAL_RCM_USB_OTG_FS_CLK_SLEEP_ENABLE()  (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_OTGFSEN))
#define __DAL_RCM_USB_OTG_FS_CLK_SLEEP_DISABLE()   (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_OTGFSEN))

#define __DAL_RCM_RNG_CLK_SLEEP_ENABLE()    (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_RNGEN))
#define __DAL_RCM_RNG_CLK_SLEEP_DISABLE()   (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_RNGEN))

#define __DAL_RCM_SMC_CLK_SLEEP_ENABLE()    (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_SMCEN))
#define __DAL_RCM_SMC_CLK_SLEEP_DISABLE()   (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_SMCEN))

#define __DAL_RCM_QSPI_CLK_SLEEP_ENABLE()   (RCM->LPAHB2CLKEN |= (RCM_LPAHB2CLKEN_QSPIEN))
#define __DAL_RCM_QSPI_CLK_SLEEP_DISABLE()  (RCM->LPAHB2CLKEN &= ~(RCM_LPAHB2CLKEN_QSPIEN))
/**
  * @}
  */

/** @defgroup RCMEx_APB1_LowPower_Enable_Disable APB1 Peripheral Low Power Enable Disable 
  * @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @{
  */
#define __DAL_RCM_TMR2_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR2EN))
#define __DAL_RCM_TMR3_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR3EN))
#define __DAL_RCM_TMR4_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR4EN))
#define __DAL_RCM_SPI3_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_SPI3EN))
#define __DAL_RCM_I2C3_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_I2C3EN))
#define __DAL_RCM_TMR12_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR12EN))
#define __DAL_RCM_TMR13_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR13EN))
#define __DAL_RCM_TMR14_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_TMR14EN))
#define __DAL_RCM_USART3_CLK_SLEEP_ENABLE()  (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_USART3EN))
#define __DAL_RCM_UART4_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_UART4EN))
#define __DAL_RCM_UART5_CLK_SLEEP_ENABLE()   (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_UART5EN))
#define __DAL_RCM_CAN1_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_CAN1EN))
#define __DAL_RCM_CAN2_CLK_SLEEP_ENABLE()    (RCM->LPAPB1CLKEN |= (RCM_LPAPB1CLKEN_CAN2EN))

#define __DAL_RCM_TMR2_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR2EN))
#define __DAL_RCM_TMR3_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR3EN))
#define __DAL_RCM_TMR4_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR4EN))
#define __DAL_RCM_SPI3_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_SPI3EN))
#define __DAL_RCM_I2C3_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_I2C3EN))
#define __DAL_RCM_TMR12_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR12EN))
#define __DAL_RCM_TMR13_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR13EN))
#define __DAL_RCM_TMR14_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_TMR14EN))
#define __DAL_RCM_USART3_CLK_SLEEP_DISABLE() (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_USART3EN))
#define __DAL_RCM_UART4_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_UART4EN))
#define __DAL_RCM_UART5_CLK_SLEEP_DISABLE()  (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_UART5EN))
#define __DAL_RCM_CAN1_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_CAN1EN))
#define __DAL_RCM_CAN2_CLK_SLEEP_DISABLE()   (RCM->LPAPB1CLKEN &= ~(RCM_LPAPB1CLKEN_CAN2EN))
/**
  * @}
  */

/** @defgroup RCMEx_APB2_LowPower_Enable_Disable APB2 Peripheral Low Power Enable Disable 
  * @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @{
  */
#define __DAL_RCM_SPI5_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_SPI5LPEN))
#define __DAL_RCM_SDIO_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_SDIOEN))
#define __DAL_RCM_SPI4_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_SPI4EN))
#define __DAL_RCM_TMR10_CLK_SLEEP_ENABLE()   (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_TMR10EN))
#define __DAL_RCM_TMR8_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_TMR8EN))
#define __DAL_RCM_ADC2_CLK_SLEEP_ENABLE()    (RCM->LPAPB2CLKEN |= (RCM_LPAPB2CLKEN_ADC2EN))

#define __DAL_RCM_SDIO_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_SDIOEN))
#define __DAL_RCM_SPI4_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_SPI4EN))
#define __DAL_RCM_TMR10_CLK_SLEEP_DISABLE()  (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_TMR10EN))
#define __DAL_RCM_SPI5_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_SPI5LPEN))
#define __DAL_RCM_TMR8_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_TMR8EN))
#define __DAL_RCM_ADC2_CLK_SLEEP_DISABLE()   (RCM->LPAPB2CLKEN &= ~(RCM_LPAPB2CLKEN_ADC2EN))
/**
  * @}
  */
#endif /* APM32F411xx */
/*----------------------------------------------------------------------------*/

/*------------------------------- PLL Configuration --------------------------*/
/** @brief  Macro to configure the main PLL clock source, multiplication and division factors.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __RCM_PLLSource__ specifies the PLL entry clock source.
  *         This parameter can be one of the following values:
  *            @arg RCM_PLLSOURCE_HSI: HSI oscillator clock selected as PLL clock entry
  *            @arg RCM_PLLSOURCE_HSE: HSE oscillator clock selected as PLL clock entry
  * @note   This clock source (RCM_PLLSource) is common for the main PLL and PLLI2S.  
  * @param  __PLLB__ specifies the division factor for PLL VCO input clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 63.
  * @note   You have to set the PLLB parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLL jitter.
  * @param  __PLL1A__ specifies the multiplication factor for PLL VCO output clock
  *         This parameter must be a number between Min_Data = 50 and Max_Data = 432
  *         Except for APM32F411xx devices where Min_Data = 192.
  * @note   You have to set the PLL1A parameter correctly to ensure that the VCO
  *         output frequency is between 100 and 432 MHz, Except for APM32F411xx devices
  *         where frequency is between 192 and 432 MHz.
  * @param  __PLL1C__ specifies the division factor for main system clock (SYSCLK)
  *         This parameter must be a number in the range {2, 4, 6, or 8}.
  *           
  * @param  __PLLD__ specifies the division factor for OTG FS, SDIO and RNG clocks
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 15.
  * @note   If the USB OTG FS is used in your application, you have to set the
  *         PLLD parameter correctly to have 48 MHz clock for the USB. However,
  *         the SDIO and RNG need a frequency lower than or equal to 48 MHz to work
  *         correctly.
  *      
  */
#define __DAL_RCM_PLL_CONFIG(__RCM_PLLSource__, __PLLB__, __PLL1A__, __PLL1C__, __PLLD__)     \
                            (RCM->PLL1CFG = (0x20000000U | (__RCM_PLLSource__) | (__PLLB__)| \
                            ((__PLL1A__) << RCM_PLL1CFG_PLL1A_Pos)                | \
                            ((((__PLL1C__) >> 1U) -1U) << RCM_PLL1CFG_PLL1C_Pos)    | \
                            ((__PLLD__) << RCM_PLL1CFG_PLLD_Pos)))
/*----------------------------------------------------------------------------*/
                             
/*----------------------------PLLI2S Configuration ---------------------------*/
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx)

/** @brief Macros to enable or disable the PLLI2S. 
  * @note  The PLLI2S is disabled by hardware when entering STOP and STANDBY modes.
  */
#define __DAL_RCM_PLLI2S_ENABLE() (*(__IO uint32_t *) RCM_CTRL_PLL2EN_BB = ENABLE)
#define __DAL_RCM_PLLI2S_DISABLE() (*(__IO uint32_t *) RCM_CTRL_PLL2EN_BB = DISABLE)

#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

/** @brief  Macro to configure the PLLI2S clock multiplication and division factors .
  * @note   This macro must be used only when the PLLI2S is disabled.
  * @note   PLLI2S clock source is common with the main PLL (configured in 
  *         DAL_RCM_ClockConfig() API).
  * @param  __PLL2A__ specifies the multiplication factor for PLLI2S VCO output clock
  *         This parameter must be a number between Min_Data = 50 and Max_Data = 432.
  * @note   You have to set the PLL2A parameter correctly to ensure that the VCO 
  *         output frequency is between Min_Data = 100 and Max_Data = 432 MHz.
  *
  * @param  __PLL2C__ specifies the division factor for I2S clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 7.
  * @note   You have to set the PLL2C parameter correctly to not exceed 192 MHz
  *         on the I2S clock frequency.
  *
  */
#define __DAL_RCM_PLLI2S_CONFIG(__PLL2A__, __PLL2C__)                                                    \
                               (RCM->PLL2CFG = (((__PLL2A__) << RCM_PLL2CFG_PLL2A_Pos)  |\
                               ((__PLL2C__) << RCM_PLL2CFG_PLL2C_Pos)))

#if defined(APM32F411xx)
/** @brief  Macro to configure the PLLI2S clock multiplication and division factors .
  * @note   This macro must be used only when the PLLI2S is disabled.
  * @note   This macro must be used only when the PLLI2S is disabled.
  * @note   PLLI2S clock source is common with the main PLL (configured in 
  *         DAL_RCM_ClockConfig() API).
  * @param  __PLL2B__ specifies the division factor for PLLI2S VCO input clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 63.
  * @note   The PLL2B parameter is only used with APM32F411xx Devices
  * @note   You have to set the PLL2B parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLLI2S jitter.    
  * @param  __PLL2A__ specifies the multiplication factor for PLLI2S VCO output clock
  *         This parameter must be a number between Min_Data = 192 and Max_Data = 432.
  * @note   You have to set the PLL2A parameter correctly to ensure that the VCO 
  *         output frequency is between Min_Data = 192 and Max_Data = 432 MHz.
  * @param  __PLL2C__ specifies the division factor for I2S clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 7.
  * @note   You have to set the PLL2C parameter correctly to not exceed 192 MHz
  *         on the I2S clock frequency.
  */
#define __DAL_RCM_PLLI2S_I2SCLK_CONFIG(__PLL2B__, __PLL2A__, __PLL2C__) (RCM->PLL2CFG = ((__PLL2B__)                                                       |\
                                                                                                  ((__PLL2A__) << RCM_PLL2CFG_PLL2A_Pos)             |\
                                                                                                  ((__PLL2C__) << RCM_PLL2CFG_PLL2C_Pos)))
#endif /* APM32F411xx */

/*------------------------- Peripheral Clock selection -----------------------*/
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
/** @brief  Macro to configure the I2S clock source (I2SCLK).
  * @note   This function must be called before enabling the I2S APB clock.
  * @param  __SOURCE__ specifies the I2S clock source.
  *         This parameter can be one of the following values:
  *            @arg RCM_I2SCLKSOURCE_PLLI2S: PLLI2S clock used as I2S clock source.
  *            @arg RCM_I2SCLKSOURCE_EXT: External clock mapped on the I2S_CKIN pin
  *                                       used as I2S clock source.
  */
#define __DAL_RCM_I2S_CONFIG(__SOURCE__) (*(__IO uint32_t *) RCM_CFG_I2SSEL_BB = (__SOURCE__))


/** @brief  Macro to get the I2S clock source (I2SCLK).
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCM_I2SCLKSOURCE_PLLI2S: PLLI2S clock used as I2S clock source.
  *            @arg @ref RCM_I2SCLKSOURCE_EXT External clock mapped on the I2S_CKIN pin
  *                                        used as I2S clock source
  */
#define __DAL_RCM_GET_I2S_SOURCE() ((uint32_t)(READ_BIT(RCM->CFG, RCM_CFG_I2SSEL)))
#endif /* APM32F40xxx || APM32F41xxx || APM32F465xx */

#if defined(APM32F411xx)
/** @brief  Macro to configure the Timers clocks prescalers 
  * @note   This feature is only available with APM32F429x/439x Devices.  
  * @param  __PRESC__  specifies the Timers clocks prescalers selection
  *         This parameter can be one of the following values:
  *            @arg RCM_TMRPRES_DESACTIVATED: The Timers kernels clocks prescaler is 
  *                 equal to HPRE if PPREx is corresponding to division by 1 or 2, 
  *                 else it is equal to [(HPRE * PPREx) / 2] if PPREx is corresponding to 
  *                 division by 4 or more.       
  *            @arg RCM_TMRPRES_ACTIVATED: The Timers kernels clocks prescaler is 
  *                 equal to HPRE if PPREx is corresponding to division by 1, 2 or 4, 
  *                 else it is equal to [(HPRE * PPREx) / 4] if PPREx is corresponding 
  *                 to division by 8 or more.
  */     
#define __DAL_RCM_TMRCLKPRESCALER(__PRESC__) (*(__IO uint32_t *) RCM_CFGSEL_CLKPSEL_BB = (__PRESC__))

#endif /* APM32F411xx */

/*----------------------------------------------------------------------------*/

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCMEx_Exported_Functions
  *  @{
  */

/** @addtogroup RCMEx_Exported_Functions_Group1
  *  @{
  */
DAL_StatusTypeDef DAL_RCMEx_PeriphCLKConfig(RCM_PeriphCLKInitTypeDef  *PeriphClkInit);
void DAL_RCMEx_GetPeriphCLKConfig(RCM_PeriphCLKInitTypeDef  *PeriphClkInit);

uint32_t DAL_RCMEx_GetPeriphCLKFreq(uint32_t PeriphClk);

#if defined(APM32F411xx) && defined(RCM_BDCTRL_LSEMOD)
void DAL_RCMEx_SelectLSEMode(uint8_t Mode);
#endif /* APM32F411xx */
#if defined(RCM_PLLI2S_SUPPORT)
DAL_StatusTypeDef DAL_RCMEx_EnablePLLI2S(RCM_PLLI2SInitTypeDef  *PLLI2SInit);
DAL_StatusTypeDef DAL_RCMEx_DisablePLLI2S(void);
#endif /* RCM_PLLI2S_SUPPORT */
/**
  * @}
  */ 

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup RCMEx_Private_Constants RCMEx Private Constants
  * @{
  */

/** @defgroup RCMEx_BitAddress_AliasRegion RCM BitAddress AliasRegion
  * @brief RCM registers bit address in the alias region
  * @{
  */
/* --- CTRL Register ---*/

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx)
/* Alias word address of PLL2EN bit */
#define RCM_PLL2EN_BIT_NUMBER       0x1AU
#define RCM_CTRL_PLL2EN_BB          (PERIPH_BB_BASE + (RCM_CTRL_OFFSET * 32U) + (RCM_PLL2EN_BIT_NUMBER * 4U))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

/* --- DCKCFGR Register ---*/
#if defined(APM32F411xx)
/* Alias word address of TMRPRE bit */
#define RCM_CFGSEL_OFFSET           (RCM_OFFSET + 0x8CU)
#define RCM_CLKPSEL_BIT_NUMBER      0x18U
#define RCM_CFGSEL_CLKPSEL_BB       (PERIPH_BB_BASE + (RCM_CFGSEL_OFFSET * 32U) + (RCM_CLKPSEL_BIT_NUMBER * 4U))
#endif /* APM32F411xx */

/* --- CFG Register ---*/
#define RCM_CFG_OFFSET              (RCM_OFFSET + 0x08U)
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx)
/* Alias word address of I2SSRC bit */
#define RCM_I2SSEL_BIT_NUMBER       0x17U
#define RCM_CFG_I2SSEL_BB           (PERIPH_BB_BASE + (RCM_CFG_OFFSET * 32U) + (RCM_I2SSEL_BIT_NUMBER * 4U))
      
#define PLLI2S_TIMEOUT_VALUE       2U  /* Timeout value fixed to 2 ms  */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

#define PLL_TIMEOUT_VALUE          2U  /* 2 ms */
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RCMEx_Private_Macros RCMEx Private Macros
  * @{
  */
/** @defgroup RCMEx_IS_RCM_Definitions RCM Private macros to check input parameters
  * @{
  */
#if defined(APM32F407xx) || defined(APM32F417xx)
#define IS_RCM_SDRAM_DIV(DIV)           (((DIV) == RCM_SDRAM_DIV_1) ||\
                                         ((DIV) == RCM_SDRAM_DIV_2) ||\
                                         ((DIV) == RCM_SDRAM_DIV_4))
#endif /* APM32F407xx || APM32F417xx */

#define IS_RCM_PLL1A_VALUE(VALUE) ((50U <= (VALUE)) && ((VALUE) <= 432U))
#define IS_RCM_PLL2A_VALUE(VALUE) ((50U <= (VALUE)) && ((VALUE) <= 432U))

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx) 
#define IS_RCM_PERIPHCLOCK(SELECTION) ((1U <= (SELECTION)) && ((SELECTION) <= 0x0000000FU))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

#if defined(APM32F411xx) 
#define IS_RCM_PERIPHCLOCK(SELECTION) ((1U <= (SELECTION)) && ((SELECTION) <= 0x0000000FU))
#endif /* APM32F411xx */

#define IS_RCM_PLL2C_VALUE(VALUE) ((2U <= (VALUE)) && ((VALUE) <= 7U))

#if defined(APM32F411xx)
#define IS_RCM_PLL2B_VALUE(VALUE)   ((2U <= (VALUE)) && ((VALUE) <= 63U))
#if defined(RCM_BDCTRL_LSEMOD)
#define IS_RCM_LSE_MODE(MODE)           (((MODE) == RCM_LSE_LOWPOWER_MODE) ||\
                                         ((MODE) == RCM_LSE_HIGHDRIVE_MODE))
#endif /* RCM_BDCTRL_LSEMOD */
#endif /* APM32F411xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx)
      
#define IS_RCM_MCO2SOURCE(SOURCE) (((SOURCE) == RCM_MCO2SOURCE_SYSCLK) || ((SOURCE) == RCM_MCO2SOURCE_PLLI2SCLK)|| \
                                   ((SOURCE) == RCM_MCO2SOURCE_HSE)    || ((SOURCE) == RCM_MCO2SOURCE_PLLCLK))

#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

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

#endif /* APM32F4xx_DAL_RCM_EX_H */

