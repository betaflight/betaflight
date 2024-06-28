/**
  *
  * @file    apm32f4xx_ddl_system.h
  * @brief   Header file of SYSTEM DDL module.
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
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The DDL SYSTEM driver contains a set of generic APIs that can be
    used by user:
      (+) Some of the FLASH features need to be handled in the SYSTEM file.
      (+) Access to DBGCMU registers
      (+) Access to SYSCFG registers

  @endverbatim
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DDL_SYSTEM_H
#define APM32F4xx_DDL_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (FLASH) || defined (SYSCFG) || defined (DBGMCU)

/** @defgroup SYSTEM_DDL SYSTEM
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup SYSTEM_DDL_Private_Constants SYSTEM Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup SYSTEM_DDL_Exported_Constants SYSTEM Exported Constants
  * @{
  */

/** @defgroup SYSTEM_DDL_EC_REMAP SYSCFG REMAP
* @{
*/
#define DDL_SYSCFG_REMAP_FLASH              (uint32_t)0x00000000                            /*!< Main Flash memory mapped at 0x00000000              */
#define DDL_SYSCFG_REMAP_SYSTEMFLASH        SYSCFG_MMSEL_MMSEL_0                            /*!< System Flash memory mapped at 0x00000000            */
#if defined(SMC_Bank1)
#define DDL_SYSCFG_REMAP_SMC               SYSCFG_MMSEL_MMSEL_1                             /*!< SMC(NOR/PSRAM 1 and 2) mapped at 0x00000000        */
#endif /* SMC_Bank1 */

#define DDL_SYSCFG_REMAP_SRAM               (SYSCFG_MMSEL_MMSEL_1 | SYSCFG_MMSEL_MMSEL_0) /*!< SRAM1 mapped at 0x00000000                          */

/**
  * @}
  */

#if defined(SYSCFG_PMCFG_ENETSEL)
 /** @defgroup SYSTEM_DDL_EC_PMC SYSCFG PMC
* @{
*/
#define DDL_SYSCFG_PMCFG_ETHMII               (uint32_t)0x00000000                                /*!< ETH Media MII interface */
#define DDL_SYSCFG_PMCFG_ETHRMII              (uint32_t)SYSCFG_PMCFG_ENETSEL                   /*!< ETH Media RMII interface */

/**
  * @}
  */
#endif /* SYSCFG_PMCFG_ENETSEL */



#if defined(SYSCFG_MMSEL_UFB_MODE)
/** @defgroup SYSTEM_DDL_EC_BANKMODE SYSCFG BANK MODE
  * @{
  */
#define DDL_SYSCFG_BANKMODE_BANK1          (uint32_t)0x00000000       /*!< Flash Bank 1 base address mapped at 0x0800 0000 (AXI) and 0x0020 0000 (TCM)
                                                                      and Flash Bank 2 base address mapped at 0x0810 0000 (AXI) and 0x0030 0000 (TCM)*/
#define DDL_SYSCFG_BANKMODE_BANK2          SYSCFG_MMSEL_UFB_MODE     /*!< Flash Bank 2 base address mapped at 0x0800 0000 (AXI) and 0x0020 0000(TCM)
                                                                      and Flash Bank 1 base address mapped at 0x0810 0000 (AXI) and 0x0030 0000(TCM) */
/**
  * @}
  */
#endif /* SYSCFG_MMSEL_UFB_MODE */
/** @defgroup SYSTEM_DDL_EC_I2C_FASTMODEPLUS SYSCFG I2C FASTMODEPLUS
  * @{
  */ 
#if defined(SYSCFG_CFGR_FMPI2C1_SCL)
#define DDL_SYSCFG_I2C_FASTMODEPLUS_SCL         SYSCFG_CFGR_FMPI2C1_SCL   /*!< Enable Fast Mode Plus on FMPI2C_SCL pin */
#define DDL_SYSCFG_I2C_FASTMODEPLUS_SDA         SYSCFG_CFGR_FMPI2C1_SDA   /*!< Enable Fast Mode Plus on FMPI2C_SDA pin*/
#endif /* SYSCFG_CFGR_FMPI2C1_SCL */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_EINT_PORT SYSCFG EINT PORT
  * @{
  */
#define DDL_SYSCFG_EINT_PORTA               (uint32_t)0               /*!< EINT PORT A                        */
#define DDL_SYSCFG_EINT_PORTB               (uint32_t)1               /*!< EINT PORT B                        */
#define DDL_SYSCFG_EINT_PORTC               (uint32_t)2               /*!< EINT PORT C                        */
#define DDL_SYSCFG_EINT_PORTD               (uint32_t)3               /*!< EINT PORT D                        */
#define DDL_SYSCFG_EINT_PORTE               (uint32_t)4               /*!< EINT PORT E                        */
#if defined(GPIOF)
#define DDL_SYSCFG_EINT_PORTF               (uint32_t)5               /*!< EINT PORT F                        */
#endif /* GPIOF */
#if defined(GPIOG)
#define DDL_SYSCFG_EINT_PORTG               (uint32_t)6               /*!< EINT PORT G                        */
#endif /* GPIOG */
#define DDL_SYSCFG_EINT_PORTH               (uint32_t)7               /*!< EINT PORT H                        */
#if defined(GPIOI)
#define DDL_SYSCFG_EINT_PORTI               (uint32_t)8               /*!< EINT PORT I                        */
#endif /* GPIOI */
#if defined(GPIOJ)
#define DDL_SYSCFG_EINT_PORTJ               (uint32_t)9               /*!< EINT PORT J                        */
#endif /* GPIOJ */
#if defined(GPIOK)
#define DDL_SYSCFG_EINT_PORTK               (uint32_t)10              /*!< EINT PORT k                        */
#endif /* GPIOK */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_EINT_LINE SYSCFG EINT LINE
  * @{
  */
#define DDL_SYSCFG_EINT_LINE0               (uint32_t)(0x000FU << 16 | 0)  /*!< EINT_POSITION_0  | EINTCFG[0] */
#define DDL_SYSCFG_EINT_LINE1               (uint32_t)(0x00F0U << 16 | 0)  /*!< EINT_POSITION_4  | EINTCFG[0] */
#define DDL_SYSCFG_EINT_LINE2               (uint32_t)(0x0F00U << 16 | 0)  /*!< EINT_POSITION_8  | EINTCFG[0] */
#define DDL_SYSCFG_EINT_LINE3               (uint32_t)(0xF000U << 16 | 0)  /*!< EINT_POSITION_12 | EINTCFG[0] */
#define DDL_SYSCFG_EINT_LINE4               (uint32_t)(0x000FU << 16 | 1)  /*!< EINT_POSITION_0  | EINTCFG[1] */
#define DDL_SYSCFG_EINT_LINE5               (uint32_t)(0x00F0U << 16 | 1)  /*!< EINT_POSITION_4  | EINTCFG[1] */
#define DDL_SYSCFG_EINT_LINE6               (uint32_t)(0x0F00U << 16 | 1)  /*!< EINT_POSITION_8  | EINTCFG[1] */
#define DDL_SYSCFG_EINT_LINE7               (uint32_t)(0xF000U << 16 | 1)  /*!< EINT_POSITION_12 | EINTCFG[1] */
#define DDL_SYSCFG_EINT_LINE8               (uint32_t)(0x000FU << 16 | 2)  /*!< EINT_POSITION_0  | EINTCFG[2] */
#define DDL_SYSCFG_EINT_LINE9               (uint32_t)(0x00F0U << 16 | 2)  /*!< EINT_POSITION_4  | EINTCFG[2] */
#define DDL_SYSCFG_EINT_LINE10              (uint32_t)(0x0F00U << 16 | 2)  /*!< EINT_POSITION_8  | EINTCFG[2] */
#define DDL_SYSCFG_EINT_LINE11              (uint32_t)(0xF000U << 16 | 2)  /*!< EINT_POSITION_12 | EINTCFG[2] */
#define DDL_SYSCFG_EINT_LINE12              (uint32_t)(0x000FU << 16 | 3)  /*!< EINT_POSITION_0  | EINTCFG[3] */
#define DDL_SYSCFG_EINT_LINE13              (uint32_t)(0x00F0U << 16 | 3)  /*!< EINT_POSITION_4  | EINTCFG[3] */
#define DDL_SYSCFG_EINT_LINE14              (uint32_t)(0x0F00U << 16 | 3)  /*!< EINT_POSITION_8  | EINTCFG[3] */
#define DDL_SYSCFG_EINT_LINE15              (uint32_t)(0xF000U << 16 | 3)  /*!< EINT_POSITION_12 | EINTCFG[3] */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_TRACE DBGMCU TRACE Pin Assignment
  * @{
  */
#define DDL_DBGMCU_TRACE_NONE               0x00000000U                                     /*!< TRACE pins not assigned (default state) */
#define DDL_DBGMCU_TRACE_ASYNCH             DBGMCU_CFG_TRACE_IOEN                            /*!< TRACE pin assignment for Asynchronous Mode */
#define DDL_DBGMCU_TRACE_SYNCH_SIZE1        (DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE_0) /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 1 */
#define DDL_DBGMCU_TRACE_SYNCH_SIZE2        (DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE_1) /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 2 */
#define DDL_DBGMCU_TRACE_SYNCH_SIZE4        (DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE)   /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 4 */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_APB1_GRP1_STOP_IP DBGMCU APB1 GRP1 STOP IP
  * @{
  */
#if defined(DBGMCU_APB1F_TMR2_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR2_STOP      DBGMCU_APB1F_TMR2_STS          /*!< TMR2 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR2_STS */
#if defined(DBGMCU_APB1F_TMR3_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR3_STOP      DBGMCU_APB1F_TMR3_STS          /*!< TMR3 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR3_STS */
#if defined(DBGMCU_APB1F_TMR4_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR4_STOP      DBGMCU_APB1F_TMR4_STS          /*!< TMR4 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR4_STS */
#define DDL_DBGMCU_APB1_GRP1_TMR5_STOP      DBGMCU_APB1F_TMR5_STS          /*!< TMR5 counter stopped when core is halted */
#if defined(DBGMCU_APB1F_TMR6_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR6_STOP      DBGMCU_APB1F_TMR6_STS          /*!< TMR6 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR6_STS */
#if defined(DBGMCU_APB1F_TMR7_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR7_STOP      DBGMCU_APB1F_TMR7_STS          /*!< TMR7 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR7_STOP */
#if defined(DBGMCU_APB1F_TMR12_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR12_STOP     DBGMCU_APB1F_TMR12_STS         /*!< TMR12 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR12_STS */
#if defined(DBGMCU_APB1F_TMR13_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR13_STOP     DBGMCU_APB1F_TMR13_STS         /*!< TMR13 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR13_STS */
#if defined(DBGMCU_APB1F_TMR14_STS)
#define DDL_DBGMCU_APB1_GRP1_TMR14_STOP     DBGMCU_APB1F_TMR14_STS         /*!< TMR14 counter stopped when core is halted */
#endif /* DBGMCU_APB1F_TMR14_STS */
#if defined(DBGMCU_APB1F_LPTIM_STOP)
#define DDL_DBGMCU_APB1_GRP1_LPTIM_STOP     DBGMCU_APB1F_LPTIM_STOP         /*!< LPTIM counter stopped when core is halted */
#endif /* DBGMCU_APB1F_LPTIM_STOP */
#define DDL_DBGMCU_APB1_GRP1_RTC_STOP       DBGMCU_APB1F_RTC_STS           /*!< RTC counter stopped when core is halted */
#define DDL_DBGMCU_APB1_GRP1_WWDT_STOP      DBGMCU_APB1F_WWDT_STS          /*!< Debug Window Watchdog stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_IWDT_STOP      DBGMCU_APB1F_IWDT_STS          /*!< Debug Independent Watchdog stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_I2C1_STOP      DBGMCU_APB1F_I2C1_SMBUS_TIMEOUT_STS /*!< I2C1 SMBUS timeout mode stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_I2C2_STOP      DBGMCU_APB1F_I2C2_SMBUS_TIMEOUT_STS /*!< I2C2 SMBUS timeout mode stopped when Core is halted */
#if defined(DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS)
#define DDL_DBGMCU_APB1_GRP1_I2C3_STOP      DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS /*!< I2C3 SMBUS timeout mode stopped when Core is halted */
#endif /* DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS */
#if defined(DBGMCU_APB1F_I2C4_SMBUS_TIMEOUT)
#define DDL_DBGMCU_APB1_GRP1_I2C4_STOP      DBGMCU_APB1F_I2C4_SMBUS_TIMEOUT /*!< I2C4 SMBUS timeout mode stopped when Core is halted */
#endif /* DBGMCU_APB1F_I2C4_SMBUS_TIMEOUT */
#if defined(DBGMCU_APB1F_CAN1_STS)
#define DDL_DBGMCU_APB1_GRP1_CAN1_STOP      DBGMCU_APB1F_CAN1_STS          /*!< CAN1 debug stopped when Core is halted  */
#endif /* DBGMCU_APB1F_CAN1_STS */
#if defined(DBGMCU_APB1F_CAN2_STS)
#define DDL_DBGMCU_APB1_GRP1_CAN2_STOP      DBGMCU_APB1F_CAN2_STS          /*!< CAN2 debug stopped when Core is halted  */
#endif /* DBGMCU_APB1F_CAN2_STS */
#if defined(DBGMCU_APB1F_CAN3_STOP)
#define DDL_DBGMCU_APB1_GRP1_CAN3_STOP      DBGMCU_APB1F_CAN3_STOP          /*!< CAN3 debug stopped when Core is halted  */
#endif /* DBGMCU_APB1F_CAN3_STOP */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_APB2_GRP1_STOP_IP DBGMCU APB2 GRP1 STOP IP
  * @{
  */
#define DDL_DBGMCU_APB2_GRP1_TMR1_STOP      DBGMCU_APB2F_TMR1_STS   /*!< TMR1 counter stopped when core is halted */
#if defined(DBGMCU_APB2F_TMR8_STS)
#define DDL_DBGMCU_APB2_GRP1_TMR8_STOP      DBGMCU_APB2F_TMR8_STS   /*!< TMR8 counter stopped when core is halted */
#endif /* DBGMCU_APB2F_TMR8_STS */
#define DDL_DBGMCU_APB2_GRP1_TMR9_STOP      DBGMCU_APB2F_TMR9_STS   /*!< TMR9 counter stopped when core is halted */
#if defined(DBGMCU_APB2F_TMR10_STS)
#define DDL_DBGMCU_APB2_GRP1_TMR10_STOP     DBGMCU_APB2F_TMR10_STS   /*!< TMR10 counter stopped when core is halted */
#endif /* DBGMCU_APB2F_TMR10_STS */
#define DDL_DBGMCU_APB2_GRP1_TMR11_STOP     DBGMCU_APB2F_TMR11_STS   /*!< TMR11 counter stopped when core is halted */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_LATENCY FLASH LATENCY
  * @{
  */
#define DDL_FLASH_LATENCY_0                 FLASH_ACCTRL_WAITP_0WS   /*!< FLASH Zero wait state */
#define DDL_FLASH_LATENCY_1                 FLASH_ACCTRL_WAITP_1WS   /*!< FLASH One wait state */
#define DDL_FLASH_LATENCY_2                 FLASH_ACCTRL_WAITP_2WS   /*!< FLASH Two wait states */
#define DDL_FLASH_LATENCY_3                 FLASH_ACCTRL_WAITP_3WS   /*!< FLASH Three wait states */
#define DDL_FLASH_LATENCY_4                 FLASH_ACCTRL_WAITP_4WS   /*!< FLASH Four wait states */
#define DDL_FLASH_LATENCY_5                 FLASH_ACCTRL_WAITP_5WS   /*!< FLASH five wait state */
#define DDL_FLASH_LATENCY_6                 FLASH_ACCTRL_WAITP_6WS   /*!< FLASH six wait state */
#define DDL_FLASH_LATENCY_7                 FLASH_ACCTRL_WAITP_7WS   /*!< FLASH seven wait states */
#define DDL_FLASH_LATENCY_8                 FLASH_ACCTRL_WAITP_8WS   /*!< FLASH eight wait states */
#define DDL_FLASH_LATENCY_9                 FLASH_ACCTRL_WAITP_9WS   /*!< FLASH nine wait states */
#define DDL_FLASH_LATENCY_10                FLASH_ACCTRL_WAITP_10WS   /*!< FLASH ten wait states */
#define DDL_FLASH_LATENCY_11                FLASH_ACCTRL_WAITP_11WS   /*!< FLASH eleven wait states */
#define DDL_FLASH_LATENCY_12                FLASH_ACCTRL_WAITP_12WS   /*!< FLASH twelve wait states */
#define DDL_FLASH_LATENCY_13                FLASH_ACCTRL_WAITP_13WS   /*!< FLASH thirteen wait states */
#define DDL_FLASH_LATENCY_14                FLASH_ACCTRL_WAITP_14WS   /*!< FLASH fourteen wait states */
#define DDL_FLASH_LATENCY_15                FLASH_ACCTRL_WAITP_15WS   /*!< FLASH fifteen wait states */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup SYSTEM_DDL_Exported_Functions SYSTEM Exported Functions
  * @{
  */

/** @defgroup SYSTEM_DDL_EF_SYSCFG SYSCFG
  * @{
  */
/**
  * @brief  Set memory mapping at address 0x00000000
  * @param  Memory This parameter can be one of the following values:
  *         @arg @ref DDL_SYSCFG_REMAP_FLASH
  *         @arg @ref DDL_SYSCFG_REMAP_SYSTEMFLASH
  *         @arg @ref DDL_SYSCFG_REMAP_SRAM
  *         @arg @ref DDL_SYSCFG_REMAP_SMC (*)
  *         @arg @ref DDL_SYSCFG_REMAP_FMC (*)
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_SetRemapMemory(uint32_t Memory)
{
  MODIFY_REG(SYSCFG->MMSEL, SYSCFG_MMSEL_MMSEL, Memory);
}

/**
  * @brief  Get memory mapping at address 0x00000000
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SYSCFG_REMAP_FLASH
  *         @arg @ref DDL_SYSCFG_REMAP_SYSTEMFLASH
  *         @arg @ref DDL_SYSCFG_REMAP_SRAM
  *         @arg @ref DDL_SYSCFG_REMAP_SMC (*)
  *         @arg @ref DDL_SYSCFG_REMAP_FMC (*)
  */
__STATIC_INLINE uint32_t DDL_SYSCFG_GetRemapMemory(void)
{
  return (uint32_t)(READ_BIT(SYSCFG->MMSEL, SYSCFG_MMSEL_MMSEL));
}

/**
  * @brief  Enables the Compensation cell Power Down
  * @note   The I/O compensation cell can be used only when the device supply
  *         voltage ranges from 2.4 to 3.6 V
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_EnableCompensationCell(void)
{
  SET_BIT(SYSCFG->CCCTRL, SYSCFG_CCCTRL_CCPD);
}

/**
  * @brief  Disables the Compensation cell Power Down
  * @note   The I/O compensation cell can be used only when the device supply
  *         voltage ranges from 2.4 to 3.6 V
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_DisableCompensationCell(void)
{
  CLEAR_BIT(SYSCFG->CCCTRL, SYSCFG_CCCTRL_CCPD);
}

/**
  * @brief  Get Compensation Cell ready Flag
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SYSCFG_IsActiveFlag_CMPCR(void)
{
  return (READ_BIT(SYSCFG->CCCTRL, SYSCFG_CCCTRL_EDYFLG) == (SYSCFG_CCCTRL_EDYFLG));
}

#if defined(SYSCFG_PMCFG_ENETSEL)
/**
  * @brief  Select Ethernet PHY interface 
  * @param  Interface This parameter can be one of the following values:
  *         @arg @ref DDL_SYSCFG_PMCFG_ETHMII
  *         @arg @ref DDL_SYSCFG_PMCFG_ETHRMII
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_SetPHYInterface(uint32_t Interface)
{
  MODIFY_REG(SYSCFG->PMCFG, SYSCFG_PMCFG_ENETSEL, Interface);
}

/**
  * @brief  Get Ethernet PHY interface 
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SYSCFG_PMCFG_ETHMII
  *         @arg @ref DDL_SYSCFG_PMCFG_ETHRMII
  * @retval None
  */
__STATIC_INLINE uint32_t DDL_SYSCFG_GetPHYInterface(void)
{
  return (uint32_t)(READ_BIT(SYSCFG->PMCFG, SYSCFG_PMCFG_ENETSEL));
}
#endif /* SYSCFG_PMCFG_ENETSEL */
 


#if defined(SYSCFG_MMSEL_UFB_MODE)
/**
  * @brief  Select Flash bank mode (Bank flashed at 0x08000000)
  * @param  Bank This parameter can be one of the following values:
  *         @arg @ref DDL_SYSCFG_BANKMODE_BANK1
  *         @arg @ref DDL_SYSCFG_BANKMODE_BANK2
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_SetFlashBankMode(uint32_t Bank)
{ 
  MODIFY_REG(SYSCFG->MMSEL, SYSCFG_MMSEL_UFB_MODE, Bank);
}

/**
  * @brief  Get Flash bank mode (Bank flashed at 0x08000000)
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SYSCFG_BANKMODE_BANK1
  *         @arg @ref DDL_SYSCFG_BANKMODE_BANK2
  */
__STATIC_INLINE uint32_t DDL_SYSCFG_GetFlashBankMode(void)
{
  return (uint32_t)(READ_BIT(SYSCFG->MMSEL, SYSCFG_MMSEL_UFB_MODE));
}
#endif /* SYSCFG_MMSEL_UFB_MODE */

#if defined(SYSCFG_CFGR_FMPI2C1_SCL)
/**
  * @brief  Enable the I2C fast mode plus driving capability.
  * @param  ConfigFastModePlus This parameter can be a combination of the following values:
  *         @arg @ref DDL_SYSCFG_I2C_FASTMODEPLUS_SCL
  *         @arg @ref DDL_SYSCFG_I2C_FASTMODEPLUS_SDA
  *         (*) value not defined in all devices
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_EnableFastModePlus(uint32_t ConfigFastModePlus)
{
  SET_BIT(SYSCFG->CFGR, ConfigFastModePlus);
}

/**
  * @brief  Disable the I2C fast mode plus driving capability.
  * @param  ConfigFastModePlus This parameter can be a combination of the following values:
  *         @arg @ref DDL_SYSCFG_I2C_FASTMODEPLUS_SCL
  *         @arg @ref DDL_SYSCFG_I2C_FASTMODEPLUS_SDA
  *         (*) value not defined in all devices
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_DisableFastModePlus(uint32_t ConfigFastModePlus)
{
  CLEAR_BIT(SYSCFG->CFGR, ConfigFastModePlus);
}
#endif /* SYSCFG_CFGR_FMPI2C1_SCL */

/**
  * @brief  Configure source input for the EINT external interrupt.
  * @param  Port This parameter can be one of the following values:
  *         @arg @ref DDL_SYSCFG_EINT_PORTA
  *         @arg @ref DDL_SYSCFG_EINT_PORTB
  *         @arg @ref DDL_SYSCFG_EINT_PORTC
  *         @arg @ref DDL_SYSCFG_EINT_PORTD
  *         @arg @ref DDL_SYSCFG_EINT_PORTE
  *         @arg @ref DDL_SYSCFG_EINT_PORTF (*)
  *         @arg @ref DDL_SYSCFG_EINT_PORTG (*)
  *         @arg @ref DDL_SYSCFG_EINT_PORTH
  *
  *         (*) value not defined in all devices
  * @param  Line This parameter can be one of the following values:
  *         @arg @ref DDL_SYSCFG_EINT_LINE0
  *         @arg @ref DDL_SYSCFG_EINT_LINE1
  *         @arg @ref DDL_SYSCFG_EINT_LINE2
  *         @arg @ref DDL_SYSCFG_EINT_LINE3
  *         @arg @ref DDL_SYSCFG_EINT_LINE4
  *         @arg @ref DDL_SYSCFG_EINT_LINE5
  *         @arg @ref DDL_SYSCFG_EINT_LINE6
  *         @arg @ref DDL_SYSCFG_EINT_LINE7
  *         @arg @ref DDL_SYSCFG_EINT_LINE8
  *         @arg @ref DDL_SYSCFG_EINT_LINE9
  *         @arg @ref DDL_SYSCFG_EINT_LINE10
  *         @arg @ref DDL_SYSCFG_EINT_LINE11
  *         @arg @ref DDL_SYSCFG_EINT_LINE12
  *         @arg @ref DDL_SYSCFG_EINT_LINE13
  *         @arg @ref DDL_SYSCFG_EINT_LINE14
  *         @arg @ref DDL_SYSCFG_EINT_LINE15
  * @retval None
  */
__STATIC_INLINE void DDL_SYSCFG_SetEINTSource(uint32_t Port, uint32_t Line)
{
  MODIFY_REG(SYSCFG->EINTCFG[Line & 0xFF], (Line >> 16), Port << POSITION_VAL((Line >> 16)));
}

/**
  * @brief  Get the configured defined for specific EINT Line
  * @param  Line This parameter can be one of the following values:
  *         @arg @ref DDL_SYSCFG_EINT_LINE0
  *         @arg @ref DDL_SYSCFG_EINT_LINE1
  *         @arg @ref DDL_SYSCFG_EINT_LINE2
  *         @arg @ref DDL_SYSCFG_EINT_LINE3
  *         @arg @ref DDL_SYSCFG_EINT_LINE4
  *         @arg @ref DDL_SYSCFG_EINT_LINE5
  *         @arg @ref DDL_SYSCFG_EINT_LINE6
  *         @arg @ref DDL_SYSCFG_EINT_LINE7
  *         @arg @ref DDL_SYSCFG_EINT_LINE8
  *         @arg @ref DDL_SYSCFG_EINT_LINE9
  *         @arg @ref DDL_SYSCFG_EINT_LINE10
  *         @arg @ref DDL_SYSCFG_EINT_LINE11
  *         @arg @ref DDL_SYSCFG_EINT_LINE12
  *         @arg @ref DDL_SYSCFG_EINT_LINE13
  *         @arg @ref DDL_SYSCFG_EINT_LINE14
  *         @arg @ref DDL_SYSCFG_EINT_LINE15
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SYSCFG_EINT_PORTA
  *         @arg @ref DDL_SYSCFG_EINT_PORTB
  *         @arg @ref DDL_SYSCFG_EINT_PORTC
  *         @arg @ref DDL_SYSCFG_EINT_PORTD
  *         @arg @ref DDL_SYSCFG_EINT_PORTE
  *         @arg @ref DDL_SYSCFG_EINT_PORTF (*)
  *         @arg @ref DDL_SYSCFG_EINT_PORTG (*)
  *         @arg @ref DDL_SYSCFG_EINT_PORTH
  *         (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t DDL_SYSCFG_GetEINTSource(uint32_t Line)
{
  return (uint32_t)(READ_BIT(SYSCFG->EINTCFG[Line & 0xFF], (Line >> 16)) >> POSITION_VAL(Line >> 16));
}

/**
  * @}
  */


/** @defgroup SYSTEM_DDL_EF_DBGMCU DBGMCU
  * @{
  */

/**
  * @brief  Return the device identifier
  * @note For APM32F405/407xx and APM32F417xx devices, the device ID is 0x413
  * @note For APM32F411xx devices, the device ID is 0x431
  * @retval Values between Min_Data=0x00 and Max_Data=0xFFF
  */
__STATIC_INLINE uint32_t DDL_DBGMCU_GetDeviceID(void)
{
  return (uint32_t)(READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_EQR));
}

/**
  * @brief  Return the device revision identifier
  * @note This field indicates the revision of the device.
          For example, it is read as RevA -> 0x1000, Cat 2 revZ -> 0x1001, rev1 -> 0x1003, rev2 ->0x1007, revY -> 0x100F for APM32F405/407xx and APM32F417xx devices
          For example, it is read as RevA -> 0x0015 for APM32F411xx devices
  * @retval Values between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t DDL_DBGMCU_GetRevisionID(void)
{
  return (uint32_t)(READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_WVR) >> DBGMCU_IDCODE_WVR_Pos);
}

/**
  * @brief  Enable the Debug Module during SLEEP mode
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_EnableDBGSleepMode(void)
{
  SET_BIT(DBGMCU->CFG, DBGMCU_CFG_SLEEP_CLK_STS);
}

/**
  * @brief  Disable the Debug Module during SLEEP mode
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_DisableDBGSleepMode(void)
{
  CLEAR_BIT(DBGMCU->CFG, DBGMCU_CFG_SLEEP_CLK_STS);
}

/**
  * @brief  Enable the Debug Module during STOP mode
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_EnableDBGStopMode(void)
{
  SET_BIT(DBGMCU->CFG, DBGMCU_CFG_STOP_CLK_STS);
}

/**
  * @brief  Disable the Debug Module during STOP mode
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_DisableDBGStopMode(void)
{
  CLEAR_BIT(DBGMCU->CFG, DBGMCU_CFG_STOP_CLK_STS);
}

/**
  * @brief  Enable the Debug Module during STANDBY mode
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_EnableDBGStandbyMode(void)
{
  SET_BIT(DBGMCU->CFG, DBGMCU_CFG_STANDBY_CLK_STS);
}

/**
  * @brief  Disable the Debug Module during STANDBY mode
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_DisableDBGStandbyMode(void)
{
  CLEAR_BIT(DBGMCU->CFG, DBGMCU_CFG_STANDBY_CLK_STS);
}

/**
  * @brief  Set Trace pin assignment control
  * @param  PinAssignment This parameter can be one of the following values:
  *         @arg @ref DDL_DBGMCU_TRACE_NONE
  *         @arg @ref DDL_DBGMCU_TRACE_ASYNCH
  *         @arg @ref DDL_DBGMCU_TRACE_SYNCH_SIZE1
  *         @arg @ref DDL_DBGMCU_TRACE_SYNCH_SIZE2
  *         @arg @ref DDL_DBGMCU_TRACE_SYNCH_SIZE4
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_SetTracePinAssignment(uint32_t PinAssignment)
{
  MODIFY_REG(DBGMCU->CFG, DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE, PinAssignment);
}

/**
  * @brief  Get Trace pin assignment control
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DBGMCU_TRACE_NONE
  *         @arg @ref DDL_DBGMCU_TRACE_ASYNCH
  *         @arg @ref DDL_DBGMCU_TRACE_SYNCH_SIZE1
  *         @arg @ref DDL_DBGMCU_TRACE_SYNCH_SIZE2
  *         @arg @ref DDL_DBGMCU_TRACE_SYNCH_SIZE4
  */
__STATIC_INLINE uint32_t DDL_DBGMCU_GetTracePinAssignment(void)
{
  return (uint32_t)(READ_BIT(DBGMCU->CFG, DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE));
}

/**
  * @brief  Freeze APB1 peripherals (group1 peripherals)
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR2_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR3_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR4_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR5_STOP 
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR6_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR7_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR12_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR13_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR14_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_LPTIM_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_RTC_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_WWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_IWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C2_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C3_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C4_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN1_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN2_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN3_STOP (*)
  *         
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_APB1_GRP1_FreezePeriph(uint32_t Periphs)
{
  SET_BIT(DBGMCU->APB1F, Periphs);
}

/**
  * @brief  Unfreeze APB1 peripherals (group1 peripherals)
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR2_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR3_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR4_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR5_STOP 
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR6_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR7_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR12_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR13_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR14_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_LPTIM_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_RTC_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_WWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_IWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C2_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C3_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C4_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN1_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN2_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN3_STOP (*)
  *         
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_APB1_GRP1_UnFreezePeriph(uint32_t Periphs)
{
  CLEAR_BIT(DBGMCU->APB1F, Periphs);
}

/**
  * @brief  Freeze APB2 peripherals
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR1_STOP
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR8_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR9_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR10_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR11_STOP (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_APB2_GRP1_FreezePeriph(uint32_t Periphs)
{
  SET_BIT(DBGMCU->APB2F, Periphs);
}

/**
  * @brief  Unfreeze APB2 peripherals
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR1_STOP
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR8_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR9_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR10_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB2_GRP1_TMR11_STOP (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_APB2_GRP1_UnFreezePeriph(uint32_t Periphs)
{
  CLEAR_BIT(DBGMCU->APB2F, Periphs);
}
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EF_FLASH FLASH
  * @{
  */

/**
  * @brief  Set FLASH Latency
  * @param  Latency This parameter can be one of the following values:
  *         @arg @ref DDL_FLASH_LATENCY_0
  *         @arg @ref DDL_FLASH_LATENCY_1
  *         @arg @ref DDL_FLASH_LATENCY_2
  *         @arg @ref DDL_FLASH_LATENCY_3
  *         @arg @ref DDL_FLASH_LATENCY_4
  *         @arg @ref DDL_FLASH_LATENCY_5
  *         @arg @ref DDL_FLASH_LATENCY_6
  *         @arg @ref DDL_FLASH_LATENCY_7
  *         @arg @ref DDL_FLASH_LATENCY_8
  *         @arg @ref DDL_FLASH_LATENCY_9
  *         @arg @ref DDL_FLASH_LATENCY_10
  *         @arg @ref DDL_FLASH_LATENCY_11
  *         @arg @ref DDL_FLASH_LATENCY_12
  *         @arg @ref DDL_FLASH_LATENCY_13
  *         @arg @ref DDL_FLASH_LATENCY_14
  *         @arg @ref DDL_FLASH_LATENCY_15
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_SetLatency(uint32_t Latency)
{
  MODIFY_REG(FLASH->ACCTRL, FLASH_ACCTRL_WAITP, Latency);
}

/**
  * @brief  Get FLASH Latency
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_FLASH_LATENCY_0
  *         @arg @ref DDL_FLASH_LATENCY_1
  *         @arg @ref DDL_FLASH_LATENCY_2
  *         @arg @ref DDL_FLASH_LATENCY_3
  *         @arg @ref DDL_FLASH_LATENCY_4
  *         @arg @ref DDL_FLASH_LATENCY_5
  *         @arg @ref DDL_FLASH_LATENCY_6
  *         @arg @ref DDL_FLASH_LATENCY_7
  *         @arg @ref DDL_FLASH_LATENCY_8
  *         @arg @ref DDL_FLASH_LATENCY_9
  *         @arg @ref DDL_FLASH_LATENCY_10
  *         @arg @ref DDL_FLASH_LATENCY_11
  *         @arg @ref DDL_FLASH_LATENCY_12
  *         @arg @ref DDL_FLASH_LATENCY_13
  *         @arg @ref DDL_FLASH_LATENCY_14
  *         @arg @ref DDL_FLASH_LATENCY_15
  */
__STATIC_INLINE uint32_t DDL_FLASH_GetLatency(void)
{
  return (uint32_t)(READ_BIT(FLASH->ACCTRL, FLASH_ACCTRL_WAITP));
}

/**
  * @brief  Enable Prefetch
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnablePrefetch(void)
{
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREFEN);
}

/**
  * @brief  Disable Prefetch
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisablePrefetch(void)
{
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREFEN);
}

/**
  * @brief  Check if Prefetch buffer is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_FLASH_IsPrefetchEnabled(void)
{
  return (READ_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREFEN) == (FLASH_ACCTRL_PREFEN));
}

/**
  * @brief  Enable Instruction cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableInstCache(void)
{
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHEEN);
}

/**
  * @brief  Disable Instruction cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableInstCache(void)
{
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHEEN);
}

/**
  * @brief  Enable Data cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableDataCache(void)
{
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHEEN);
}

/**
  * @brief  Disable Data cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableDataCache(void)
{
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHEEN);
}

/**
  * @brief  Enable Instruction cache reset
  * @note  bit can be written only when the instruction cache is disabled
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableInstCacheReset(void)
{
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHERST);
}

/**
  * @brief  Disable Instruction cache reset
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableInstCacheReset(void)
{
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHERST);
}

/**
  * @brief  Enable Data cache reset
  * @note bit can be written only when the data cache is disabled
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableDataCacheReset(void)
{
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHERST);
}

/**
  * @brief  Disable Data cache reset
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableDataCacheReset(void)
{
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHERST);
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

#endif /* defined (FLASH) || defined (SYSCFG) || defined (DBGMCU) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_SYSTEM_H */


