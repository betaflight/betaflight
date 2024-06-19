/**
  *
  * @file    apm32f4xx_ddl_bus.h
  * @brief   Header file of BUS DDL module.

  @verbatim
                      ##### RCM Limitations #####
  ==============================================================================
    [..]
      A delay between an RCM peripheral clock enable and the effective peripheral
      enabling should be taken into account in order to manage the peripheral read/write
      from/to registers.
      (+) This delay depends on the peripheral mapping.
        (++) AHB & APB peripherals, 1 dummy read is necessary

    [..]
      Workarounds:
      (#) For AHB & APB peripherals, a dummy read to the peripheral register has been
          inserted in each DDL_{BUS}_GRP{x}_EnableClock() function.

  @endverbatim
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
#ifndef APM32F4xx_DDL_BUS_H
#define APM32F4xx_DDL_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(RCM)

/** @defgroup BUS_DDL BUS
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup BUS_DDL_Exported_Constants BUS Exported Constants
  * @{
  */

/** @defgroup BUS_DDL_EC_AHB1_GRP1_PERIPH  AHB1 GRP1 PERIPH
  * @{
  */
#define DDL_AHB1_GRP1_PERIPH_ALL             0xFFFFFFFFU
#define DDL_AHB1_GRP1_PERIPH_GPIOA           RCM_AHB1CLKEN_PAEN
#define DDL_AHB1_GRP1_PERIPH_GPIOB           RCM_AHB1CLKEN_PBEN
#define DDL_AHB1_GRP1_PERIPH_GPIOC           RCM_AHB1CLKEN_PCEN
#if defined(GPIOD)
#define DDL_AHB1_GRP1_PERIPH_GPIOD           RCM_AHB1CLKEN_PDEN
#endif /* GPIOD */
#if defined(GPIOE)
#define DDL_AHB1_GRP1_PERIPH_GPIOE           RCM_AHB1CLKEN_PEEN
#endif /* GPIOE */
#if defined(GPIOF)
#define DDL_AHB1_GRP1_PERIPH_GPIOF           RCM_AHB1CLKEN_PFEN
#endif /* GPIOF */
#if defined(GPIOG)
#define DDL_AHB1_GRP1_PERIPH_GPIOG           RCM_AHB1CLKEN_PGEN
#endif /* GPIOG */
#if defined(GPIOH)
#define DDL_AHB1_GRP1_PERIPH_GPIOH           RCM_AHB1CLKEN_PHEN
#endif /* GPIOH */
#if defined(GPIOI)
#define DDL_AHB1_GRP1_PERIPH_GPIOI           RCM_AHB1CLKEN_PIEN
#endif /* GPIOI */
#if defined(GPIOJ)
#define DDL_AHB1_GRP1_PERIPH_GPIOJ           RCM_AHB1CLKEN_GPIOJEN
#endif /* GPIOJ */
#if defined(GPIOK)
#define DDL_AHB1_GRP1_PERIPH_GPIOK           RCM_AHB1CLKEN_GPIOKEN
#endif /* GPIOK */
#define DDL_AHB1_GRP1_PERIPH_CRC             RCM_AHB1CLKEN_CRCEN
#if defined(RCM_AHB1CLKEN_BKPSRAMEN)
#define DDL_AHB1_GRP1_PERIPH_BKPSRAM         RCM_AHB1CLKEN_BKPSRAMEN
#endif /* RCM_AHB1CLKEN_BKPSRAMEN */
#if defined(RCM_AHB1CLKEN_DRAMEN)
#define DDL_AHB1_GRP1_PERIPH_CCMDATARAM      RCM_AHB1CLKEN_DRAMEN
#endif /* RCM_AHB1CLKEN_DRAMEN */
#define DDL_AHB1_GRP1_PERIPH_DMA1            RCM_AHB1CLKEN_DMA1EN
#define DDL_AHB1_GRP1_PERIPH_DMA2            RCM_AHB1CLKEN_DMA2EN
#if defined(RCM_AHB1CLKEN_RNGEN)
#define DDL_AHB1_GRP1_PERIPH_RNG             RCM_AHB1CLKEN_RNGEN
#endif /* RCM_AHB1CLKEN_RNGEN */
#if defined(ETH)
#define DDL_AHB1_GRP1_PERIPH_ETHMAC          RCM_AHB1CLKEN_ETHEN
#define DDL_AHB1_GRP1_PERIPH_ETHMACTX        RCM_AHB1CLKEN_ETHTXEN
#define DDL_AHB1_GRP1_PERIPH_ETHMACRX        RCM_AHB1CLKEN_ETHRXEN
#define DDL_AHB1_GRP1_PERIPH_ETHMACPTP       RCM_AHB1CLKEN_ETHPTPEN
#endif /* ETH */
#if defined(USB_OTG_HS)
#define DDL_AHB1_GRP1_PERIPH_OTGHS           RCM_AHB1CLKEN_OTGHS1EN
#define DDL_AHB1_GRP1_PERIPH_OTGHSULPI       RCM_AHB1CLKEN_OTGHSULPIEN
#endif /* USB_OTG_HS */
#define DDL_AHB1_GRP1_PERIPH_FLITF           RCM_LPAHB1CLKEN_FMCEN
#define DDL_AHB1_GRP1_PERIPH_SRAM1           RCM_LPAHB1CLKEN_SRAM1EN
#if defined(RCM_LPAHB1CLKEN_SRAM2EN)
#define DDL_AHB1_GRP1_PERIPH_SRAM2           RCM_LPAHB1CLKEN_SRAM2EN
#endif /* RCM_LPAHB1CLKEN_SRAM2EN */
#if defined(RCM_LPAHB1CLKEN_SRAM3LPEN)
#define DDL_AHB1_GRP1_PERIPH_SRAM3           RCM_LPAHB1CLKEN_SRAM3LPEN
#endif /* RCM_LPAHB1CLKEN_SRAM3LPEN */
/**
  * @}
  */

#if defined(RCM_AHB2_SUPPORT)
/** @defgroup BUS_DDL_EC_AHB2_GRP1_PERIPH  AHB2 GRP1 PERIPH
  * @{
  */
#define DDL_AHB2_GRP1_PERIPH_ALL            0xFFFFFFFFU
#if defined(DCI)
#define DDL_AHB2_GRP1_PERIPH_DCI            RCM_AHB2CLKEN_DCIEN
#endif /* DCI */

#define DDL_AHB2_GRP1_PERIPH_FPU            RCM_AHB2CLKEN_FPUEN
#if defined(BN)
#define DDL_AHB2_GRP1_PERIPH_BN             RCM_AHB2CLKEN_BNEN
#endif /* BN */

#if defined(SM3) || defined(SM4)
#define DDL_AHB2_GRP1_PERIPH_SM             RCM_AHB2CLKEN_SMEN
#endif /* SM3 || SM4 */

#if defined(CRYP)
#define DDL_AHB2_GRP1_PERIPH_CRYP           RCM_AHB2CLKEN_CRYPEN
#endif /* CRYP */
#if defined(HASH)
#define DDL_AHB2_GRP1_PERIPH_HASH           RCM_AHB2CLKEN_HASHEN
#endif /* HASH */
#if defined(RCM_AHB2CLKEN_RNGEN)
#define DDL_AHB2_GRP1_PERIPH_RNG            RCM_AHB2CLKEN_RNGEN
#endif /* RCM_AHB2CLKEN_RNGEN */
#if defined(USB_OTG_FS)
#define DDL_AHB2_GRP1_PERIPH_OTGFS          RCM_AHB2CLKEN_OTGFSEN
#endif /* USB_OTG_FS */
#if defined(QSPI)
#define DDL_AHB2_GRP1_PERIPH_QSPI           RCM_AHB2CLKEN_QSPIEN
#endif /* QSPI */
#if defined(SMC_Bank1)
#define DDL_AHB2_GRP1_PERIPH_SMC            RCM_AHB2CLKEN_SMCEN
#endif /* SMC_Bank1 */
/**
  * @}
  */
#endif /* RCM_AHB2_SUPPORT */

#if defined(RCM_AHB3_SUPPORT)
/** @defgroup BUS_DDL_EC_AHB3_GRP1_PERIPH  AHB3 GRP1 PERIPH
  * @{
  */
#define DDL_AHB3_GRP1_PERIPH_ALL            0xFFFFFFFFU
#if defined(SMC_Bank1)
#define DDL_AHB3_GRP1_PERIPH_EMMC           RCM_AHB3CLKEN_EMMCEN
#endif /* SMC_Bank1 */
/**
  * @}
  */
#endif /* RCM_AHB3_SUPPORT */

/** @defgroup BUS_DDL_EC_APB1_GRP1_PERIPH  APB1 GRP1 PERIPH
  * @{
  */
#define DDL_APB1_GRP1_PERIPH_ALL            0xFFFFFFFFU
#if defined(TMR2)
#define DDL_APB1_GRP1_PERIPH_TMR2           RCM_APB1CLKEN_TMR2EN
#endif /* TMR2 */
#if defined(TMR3)
#define DDL_APB1_GRP1_PERIPH_TMR3           RCM_APB1CLKEN_TMR3EN
#endif /* TMR3 */
#if defined(TMR4)
#define DDL_APB1_GRP1_PERIPH_TMR4           RCM_APB1CLKEN_TMR4EN
#endif /* TMR4 */
#define DDL_APB1_GRP1_PERIPH_TMR5           RCM_APB1CLKEN_TMR5EN
#if defined(TMR6)
#define DDL_APB1_GRP1_PERIPH_TMR6           RCM_APB1CLKEN_TMR6EN
#endif /* TMR6 */
#if defined(TMR7)
#define DDL_APB1_GRP1_PERIPH_TMR7           RCM_APB1CLKEN_TMR7EN
#endif /* TMR7 */
#if defined(TMR12)
#define DDL_APB1_GRP1_PERIPH_TMR12          RCM_APB1CLKEN_TMR12EN
#endif /* TMR12 */
#if defined(TMR13)
#define DDL_APB1_GRP1_PERIPH_TMR13          RCM_APB1CLKEN_TMR13EN
#endif /* TMR13 */
#if defined(TMR14)
#define DDL_APB1_GRP1_PERIPH_TMR14          RCM_APB1CLKEN_TMR14EN
#endif /* TMR14 */
#if defined(RCM_APB1CLKEN_RTCAPBEN)
#define DDL_APB1_GRP1_PERIPH_RTCAPB         RCM_APB1CLKEN_RTCAPBEN
#endif /* RCM_APB1CLKEN_RTCAPBEN */
#define DDL_APB1_GRP1_PERIPH_WWDT           RCM_APB1CLKEN_WWDTEN
#if defined(SPI2)
#define DDL_APB1_GRP1_PERIPH_SPI2           RCM_APB1CLKEN_SPI2EN
#endif /* SPI2 */
#if defined(SPI3)
#define DDL_APB1_GRP1_PERIPH_SPI3           RCM_APB1CLKEN_SPI3EN
#endif /* SPI3 */
#define DDL_APB1_GRP1_PERIPH_USART2         RCM_APB1CLKEN_USART2EN
#if defined(USART3)
#define DDL_APB1_GRP1_PERIPH_USART3         RCM_APB1CLKEN_USART3EN
#endif /* USART3 */
#if defined(UART4)
#define DDL_APB1_GRP1_PERIPH_UART4          RCM_APB1CLKEN_UART4EN
#endif /* UART4 */
#if defined(UART5)
#define DDL_APB1_GRP1_PERIPH_UART5          RCM_APB1CLKEN_UART5EN
#endif /* UART5 */
#define DDL_APB1_GRP1_PERIPH_I2C1           RCM_APB1CLKEN_I2C1EN
#define DDL_APB1_GRP1_PERIPH_I2C2           RCM_APB1CLKEN_I2C2EN
#if defined(I2C3)
#define DDL_APB1_GRP1_PERIPH_I2C3           RCM_APB1CLKEN_I2C3EN
#endif /* I2C3 */
#if defined(CAN1)
#define DDL_APB1_GRP1_PERIPH_CAN1           RCM_APB1CLKEN_CAN1EN
#endif /* CAN1 */
#if defined(CAN2)
#define DDL_APB1_GRP1_PERIPH_CAN2           RCM_APB1CLKEN_CAN2EN
#endif /* CAN2 */
#if defined(CAN3)
#define DDL_APB1_GRP1_PERIPH_CAN3           RCM_APB1CLKEN_CAN3EN
#endif /* CAN3 */
#define DDL_APB1_GRP1_PERIPH_PMU            RCM_APB1CLKEN_PMUEN
#if defined(DAC1)
#define DDL_APB1_GRP1_PERIPH_DAC1           RCM_APB1CLKEN_DACEN
#endif /* DAC1 */
#if defined(UART7)
#define DDL_APB1_GRP1_PERIPH_UART7          RCM_APB1CLKEN_UART7EN
#endif /* UART7 */
#if defined(UART8)
#define DDL_APB1_GRP1_PERIPH_UART8          RCM_APB1CLKEN_UART8EN
#endif /* UART8 */
/**
  * @}
  */

/** @defgroup BUS_DDL_EC_APB2_GRP1_PERIPH  APB2 GRP1 PERIPH
  * @{
  */
#define DDL_APB2_GRP1_PERIPH_ALL          0xFFFFFFFFU
#define DDL_APB2_GRP1_PERIPH_TMR1         RCM_APB2CLKEN_TMR1EN
#if defined(TMR8)
#define DDL_APB2_GRP1_PERIPH_TMR8         RCM_APB2CLKEN_TMR8EN
#endif /* TMR8 */
#define DDL_APB2_GRP1_PERIPH_USART1       RCM_APB2CLKEN_USART1EN
#if defined(USART6)
#define DDL_APB2_GRP1_PERIPH_USART6       RCM_APB2CLKEN_USART6EN
#endif /* USART6 */
#if defined(UART9)
#define DDL_APB2_GRP1_PERIPH_UART9        RCM_APB2CLKEN_UART9EN
#endif /* UART9 */
#if defined(UART10)
#define DDL_APB2_GRP1_PERIPH_UART10       RCM_APB2CLKEN_UART10EN
#endif /* UART10 */
#define DDL_APB2_GRP1_PERIPH_ADC1         RCM_APB2CLKEN_ADC1EN
#if defined(ADC2)
#define DDL_APB2_GRP1_PERIPH_ADC2         RCM_APB2CLKEN_ADC2EN
#endif /* ADC2 */
#if defined(ADC3)
#define DDL_APB2_GRP1_PERIPH_ADC3         RCM_APB2CLKEN_ADC3EN
#endif /* ADC3 */
#if defined(SDIO)
#define DDL_APB2_GRP1_PERIPH_SDIO         RCM_APB2CLKEN_SDIOEN
#endif /* SDIO */
#define DDL_APB2_GRP1_PERIPH_SPI1         RCM_APB2CLKEN_SPI1EN
#if defined(SPI4)
#define DDL_APB2_GRP1_PERIPH_SPI4         RCM_APB2CLKEN_SPI4EN
#endif /* SPI4 */
#define DDL_APB2_GRP1_PERIPH_SYSCFG       RCM_APB2CLKEN_SYSCFGEN
#if defined(RCM_APB2CLKEN_EINTEN)
#define DDL_APB2_GRP1_PERIPH_EINT         RCM_APB2CLKEN_EINTEN
#endif /* RCM_APB2CLKEN_EINTEN */
#define DDL_APB2_GRP1_PERIPH_TMR9         RCM_APB2CLKEN_TMR9EN
#if defined(TMR10)
#define DDL_APB2_GRP1_PERIPH_TMR10        RCM_APB2CLKEN_TMR10EN
#endif /* TMR10 */
#define DDL_APB2_GRP1_PERIPH_TMR11        RCM_APB2CLKEN_TMR11EN
#if defined(SPI5)
#define DDL_APB2_GRP1_PERIPH_SPI5         RCM_APB2CLKEN_SPI5EN
#endif /* SPI5 */
#if defined(SPI6)
#define DDL_APB2_GRP1_PERIPH_SPI6         RCM_APB2CLKEN_SPI6EN
#endif /* SPI6 */
#if defined(COMP1) || defined(COMP2)
#define DDL_APB2_GRP1_PERIPH_COMP         RCM_APB2CLKEN_SYSCFGEN
#endif /* COMP1 || COMP2 */
#define DDL_APB2_GRP1_PERIPH_ADC          RCM_APB2RST_ADCRST
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup BUS_DDL_Exported_Functions BUS Exported Functions
  * @{
  */

/** @defgroup BUS_DDL_EF_AHB1 AHB1
  * @{
  */

/**
  * @brief  Enable AHB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOD (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOH (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOI (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_BKPSRAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CCMDATARAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACPTP (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHS (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHSULPI (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB1_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->AHB1CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->AHB1CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB1 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOD (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOH (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOI (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_BKPSRAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CCMDATARAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACPTP (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHS (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHSULPI (*)
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t DDL_AHB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCM->AHB1CLKEN, Periphs) == Periphs);
}

/**
  * @brief  Disable AHB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOD (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOH (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOI (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_BKPSRAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CCMDATARAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACPTP (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHS (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHSULPI (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB1_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCM->AHB1CLKEN, Periphs);
}

/**
  * @brief  Force AHB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOD (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOH (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOI (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHS (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB1_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCM->AHB1RST, Periphs);
}

/**
  * @brief  Release AHB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOD (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOH (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOI (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHS (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCM->AHB1RST, Periphs);
}

/**
  * @brief  Enable AHB1 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOD (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOH (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOI (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_BKPSRAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_FLITF
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_SRAM1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_SRAM2 (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_SRAM3 (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACPTP (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHS (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHSULPI (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB1_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->LPAHB1CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->LPAHB1CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Disable AHB1 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOD (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOH (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOI (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_BKPSRAM (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_FLITF
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_SRAM1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_SRAM2 (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_SRAM3 (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_ETHMACPTP (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHS (*)
  *         @arg @ref DDL_AHB1_GRP1_PERIPH_OTGHSULPI (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB1_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  CLEAR_BIT(RCM->LPAHB1CLKEN, Periphs);
}

/**
  * @}
  */

#if defined(RCM_AHB2_SUPPORT)
/** @defgroup BUS_DDL_EF_AHB2 AHB2
  * @{
  */

/**
  * @brief  Enable AHB2 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_DCI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_FPU (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_CRYP (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_HASH (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_QSPI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_SMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   The SMC peripheral is available only in APM32F411xx devices.
*/
__STATIC_INLINE void DDL_AHB2_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->AHB2CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->AHB2CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB2 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_DCI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_FPU (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_CRYP (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_HASH (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_QSPI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_SMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
  * @note   The AHB2 SMC peripheral is available only in APM32F411xx devices.
*/
__STATIC_INLINE uint32_t DDL_AHB2_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCM->AHB2CLKEN, Periphs) == Periphs);
}

/**
  * @brief  Disable AHB2 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_DCI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_FPU (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_CRYP (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_HASH (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_QSPI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_SMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   The AHB2 SMC peripheral is available only in APM32F411xx devices.
*/
__STATIC_INLINE void DDL_AHB2_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCM->AHB2CLKEN, Periphs);
}

/**
  * @brief  Force AHB2 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_DCI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_FPU (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_CRYP (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_HASH (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_QSPI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_SMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   The AHB2 SMC peripheral is available only in APM32F411xx devices.
*/
__STATIC_INLINE void DDL_AHB2_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCM->AHB2RST, Periphs);
}

/**
  * @brief  Release AHB2 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_DCI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_FPU (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_CRYP (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_HASH (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_QSPI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_SMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   The AHB2 SMC peripheral is available only in APM32F411xx devices.
*/
__STATIC_INLINE void DDL_AHB2_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCM->AHB2RST, Periphs);
}

/**
  * @brief  Enable AHB2 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_DCI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_FPU (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_CRYP (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_HASH (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_QSPI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_SMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   The AHB2 SMC peripheral is available only in APM32F411xx devices.
*/
__STATIC_INLINE void DDL_AHB2_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->LPAHB2CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->LPAHB2CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Disable AHB2 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_DCI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_FPU (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_CRYP (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_HASH (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_RNG (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_QSPI (*)
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_SMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   The AHB2 SMC peripheral is available only in APM32F411xx devices.
*/
__STATIC_INLINE void DDL_AHB2_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  CLEAR_BIT(RCM->LPAHB2CLKEN, Periphs);
}

/**
  * @}
  */
#endif /* RCM_AHB2_SUPPORT */

#if defined(RCM_AHB3_SUPPORT)
/** @defgroup BUS_DDL_EF_AHB3 AHB3
  * @{
  */

/**
  * @brief  Enable AHB3 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_EMMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB3_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->AHB3CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->AHB3CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB3 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_EMMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t DDL_AHB3_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCM->AHB3CLKEN, Periphs) == Periphs);
}

/**
  * @brief  Disable AHB3 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_EMMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB3_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCM->AHB3CLKEN, Periphs);
}

/**
  * @brief  Force AHB3 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_ALL
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_EMMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB3_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCM->AHB3RST, Periphs);
}

/**
  * @brief  Release AHB3 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB2_GRP1_PERIPH_ALL
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_EMMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB3_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCM->AHB3RST, Periphs);
}

/**
  * @brief  Enable AHB3 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_EMMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB3_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->LPAHB3CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->LPAHB3CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Disable AHB3 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_AHB3_GRP1_PERIPH_EMMC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_AHB3_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  CLEAR_BIT(RCM->LPAHB3CLKEN, Periphs);
}

/**
  * @}
  */
#endif /* RCM_AHB3_SUPPORT */

/** @defgroup BUS_DDL_EF_APB1 APB1
  * @{
  */

/**
  * @brief  Enable APB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR5
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR6 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR12 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR13 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR14 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_PMU
  *         @arg @ref DDL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART8 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_RTCAPB (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB1_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->APB1CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->APB1CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR5
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR6 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR12 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR13 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR14 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_PMU
  *         @arg @ref DDL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART8 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_RTCAPB (*)
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t DDL_APB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCM->APB1CLKEN, Periphs) == Periphs);
}

/**
  * @brief  Disable APB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR5
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR6 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR12 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR13 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR14 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_PMU
  *         @arg @ref DDL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART8 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_RTCAPB (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB1_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCM->APB1CLKEN, Periphs);
}

/**
  * @brief  Force APB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR5
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR6 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR12 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR13 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR14 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_PMU
  *         @arg @ref DDL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART8 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB1_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCM->APB1RST, Periphs);
}

/**
  * @brief  Release APB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR5
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR6 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR12 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR13 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR14 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_PMU
  *         @arg @ref DDL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART8 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCM->APB1RST, Periphs);
}

/**
  * @brief  Enable APB1 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR5
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR6 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR12 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR13 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR14 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_PMU
  *         @arg @ref DDL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART8 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_RTCAPB (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB1_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->LPAPB1CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->LPAPB1CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Disable APB1 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR5
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR6 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR12 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR13 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_TMR14 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_WWDT
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref DDL_APB1_GRP1_PERIPH_I2C3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_PMU
  *         @arg @ref DDL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART7 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_UART8 (*)
  *         @arg @ref DDL_APB1_GRP1_PERIPH_RTCAPB (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB1_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  CLEAR_BIT(RCM->LPAPB1CLKEN, Periphs);
}

/**
  * @}
  */

/** @defgroup BUS_DDL_EF_APB2 APB2
  * @{
  */

/**
  * @brief  Enable APB2 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR8 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART6 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART9 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SDIO (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI4 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref DDL_APB2_GRP1_PERIPH_COMP (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_EINT (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR9
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR11
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI5 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI6 (*)

  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB2_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->APB2CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->APB2CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB2 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR8 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART6 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART9 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SDIO (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI4 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref DDL_APB2_GRP1_PERIPH_COMP (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_EINT (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR9
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR11
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI5 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI6 (*)
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t DDL_APB2_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCM->APB2CLKEN, Periphs) == Periphs);
}

/**
  * @brief  Disable APB2 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR8 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART6 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART9 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SDIO (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI4 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref DDL_APB2_GRP1_PERIPH_COMP (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_EINT (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR9
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR11
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI5 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI6 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB2_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCM->APB2CLKEN, Periphs);
}

/**
  * @brief  Force APB2 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ALL
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR8 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART6 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART9 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC1 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SDIO (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI4 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR9
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR11
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI5 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI6 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   ADC1 and ADC2 are not available on all devices.
*/
__STATIC_INLINE void DDL_APB2_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCM->APB2RST, Periphs);
}

/**
  * @brief  Release APB2 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ALL
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR8 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART6 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART9 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC1 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SDIO (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI4 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref DDL_APB2_GRP1_PERIPH_EINT (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR9
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR11
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI5 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI6 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  * @note   ADC1 and ADC2 are not available on all devices.
*/
__STATIC_INLINE void DDL_APB2_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCM->APB2RST, Periphs);
}

/**
  * @brief  Enable APB2 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR8 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART6 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART9 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SDIO (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI4 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref DDL_APB2_GRP1_PERIPH_EINT (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR9
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR11
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI5 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI6 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB2_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCM->LPAPB2CLKEN, Periphs);
  /* Delay after an RCM peripheral clock enabling */
  tmpreg = READ_BIT(RCM->LPAPB2CLKEN, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Disable APB2 peripheral clocks in low-power mode
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR8 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_USART6 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART9 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_UART10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SDIO (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI4 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref DDL_APB2_GRP1_PERIPH_EINT (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR9
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR10 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_TMR11
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI5 (*)
  *         @arg @ref DDL_APB2_GRP1_PERIPH_SPI6 (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void DDL_APB2_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  CLEAR_BIT(RCM->LPAPB2CLKEN, Periphs);
}

/**
  * @}
  */

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

#endif /* APM32F4xx_DDL_BUS_H */

