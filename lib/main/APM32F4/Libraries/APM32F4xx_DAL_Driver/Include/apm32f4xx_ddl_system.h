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
  * The original code has been modified by Geehy Semiconductor.
  * Copyright (c) 2017 STMicroelectronics. Copyright (C) 2023-2025 Geehy Semiconductor.
  * All rights reserved.
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
#if defined(SYSCFG)
/** @defgroup SYSTEM_DDL_EC_REMAP SYSCFG REMAP
* @{
*/
#define DDL_SYSCFG_REMAP_FLASH              (uint32_t)0x00000000                            /*!< Main Flash memory mapped at 0x00000000              */
#define DDL_SYSCFG_REMAP_SYSTEMFLASH        SYSCFG_MMSEL_MMSEL_0                            /*!< System Flash memory mapped at 0x00000000            */
#if defined(SMC_Bank1)
#define DDL_SYSCFG_REMAP_SMC                SYSCFG_MMSEL_MMSEL_1                            /*!< SMC(NOR/PSRAM 1 and 2) mapped at 0x00000000        */
#endif /* SMC_Bank1 */
#if defined(DMC)
#define DDL_SYSCFG_REMAP_DMC                SYSCFG_MMSEL_MMSEL_1                            /*!< DMC(SDRAM bank 1) mapped at 0x00000000        */
#endif /* DMC */

#define DDL_SYSCFG_REMAP_SRAM               (SYSCFG_MMSEL_MMSEL_1 | SYSCFG_MMSEL_MMSEL_0)   /*!< SRAM1 mapped at 0x00000000                          */

/**
  * @}
  */

#if defined(SYSCFG_PMCFG_ENETSEL)
 /** @defgroup SYSTEM_DDL_EC_PMC SYSCFG PMC
* @{
*/
#define DDL_SYSCFG_PMCFG_ETHMII               (uint32_t)0x00000000                             /*!< ETH Media MII interface */
#define DDL_SYSCFG_PMCFG_ETHRMII              (uint32_t)SYSCFG_PMCFG_ENETSEL                   /*!< ETH Media RMII interface */

/**
  * @}
  */
#endif /* SYSCFG_PMCFG_ENETSEL */

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
#endif /* SYSCFG */

/** @defgroup SYSTEM_DDL_EC_TRACE DBGMCU TRACE Pin Assignment
  * @{
  */
#define DDL_DBGMCU_TRACE_NONE               0x00000000U                                       /*!< TRACE pins not assigned (default state) */
#define DDL_DBGMCU_TRACE_ASYNCH             DBGMCU_CFG_TRACE_IOEN                             /*!< TRACE pin assignment for Asynchronous Mode */
#define DDL_DBGMCU_TRACE_SYNCH_SIZE1        (DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE_0) /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 1 */
#define DDL_DBGMCU_TRACE_SYNCH_SIZE2        (DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE_1) /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 2 */
#define DDL_DBGMCU_TRACE_SYNCH_SIZE4        (DBGMCU_CFG_TRACE_IOEN | DBGMCU_CFG_TRACE_MODE)   /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 4 */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_APB1_GRP1_STOP_IP DBGMCU APB1 GRP1 STOP IP
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define DDL_DBGMCU_APB1_GRP1_TMR2_STOP      DBGMCU_CFG_TMR2_STS            /*!< TMR2 counter stopped when core is halted */
#define DDL_DBGMCU_APB1_GRP1_TMR3_STOP      DBGMCU_CFG_TMR3_STS            /*!< TMR3 counter stopped when core is halted */
#define DDL_DBGMCU_APB1_GRP1_TMR4_STOP      DBGMCU_CFG_TMR4_STS            /*!< TMR4 counter stopped when core is halted */
#define DDL_DBGMCU_APB1_GRP1_TMR5_STOP      DBGMCU_CFG_TMR5_STS            /*!< TMR5 counter stopped when core is halted */
#define DDL_DBGMCU_APB1_GRP1_WWDT_STOP      DBGMCU_CFG_WWDT_STS            /*!< Debug Window Watchdog stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_IWDT_STOP      DBGMCU_CFG_IWDT_STS            /*!< Debug Independent Watchdog stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_I2C1_STOP      DBGMCU_CFG_I2C1_SMBUS_TIMEOUT_STS /*!< I2C1 SMBUS timeout mode stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_CAN1_STOP      DBGMCU_CFG_CAN1_STS            /*!< CAN1 debug stopped when Core is halted  */
#define DDL_DBGMCU_APB1_GRP1_CAN2_STOP      DBGMCU_CFG_CAN2_STS            /*!< CAN2 debug stopped when Core is halted  */
#else
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
#define DDL_DBGMCU_APB1_GRP1_RTC_STOP       DBGMCU_APB1F_RTC_STS           /*!< RTC counter stopped when core is halted */
#define DDL_DBGMCU_APB1_GRP1_WWDT_STOP      DBGMCU_APB1F_WWDT_STS          /*!< Debug Window Watchdog stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_IWDT_STOP      DBGMCU_APB1F_IWDT_STS          /*!< Debug Independent Watchdog stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_I2C1_STOP      DBGMCU_APB1F_I2C1_SMBUS_TIMEOUT_STS /*!< I2C1 SMBUS timeout mode stopped when Core is halted */
#define DDL_DBGMCU_APB1_GRP1_I2C2_STOP      DBGMCU_APB1F_I2C2_SMBUS_TIMEOUT_STS /*!< I2C2 SMBUS timeout mode stopped when Core is halted */
#if defined(DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS)
#define DDL_DBGMCU_APB1_GRP1_I2C3_STOP      DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS /*!< I2C3 SMBUS timeout mode stopped when Core is halted */
#endif /* DBGMCU_APB1F_I2C3_SMBUS_TIMEOUT_STS */
#if defined(DBGMCU_APB1F_CAN1_STS)
#define DDL_DBGMCU_APB1_GRP1_CAN1_STOP      DBGMCU_APB1F_CAN1_STS          /*!< CAN1 debug stopped when Core is halted  */
#endif /* DBGMCU_APB1F_CAN1_STS */
#if defined(DBGMCU_APB1F_CAN2_STS)
#define DDL_DBGMCU_APB1_GRP1_CAN2_STOP      DBGMCU_APB1F_CAN2_STS          /*!< CAN2 debug stopped when Core is halted  */
#endif /* DBGMCU_APB1F_CAN2_STS */
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_APB2_GRP1_STOP_IP DBGMCU APB2 GRP1 STOP IP
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define DDL_DBGMCU_APB2_GRP1_TMR1_STOP      DBGMCU_CFG_TMR1_STS     /*!< TMR1 counter stopped when core is halted */
#define DDL_DBGMCU_APB2_GRP1_TMR8_STOP      DBGMCU_CFG_TMR8_STS     /*!< TMR8 counter stopped when core is halted */
#else
#define DDL_DBGMCU_APB2_GRP1_TMR1_STOP      DBGMCU_APB2F_TMR1_STS   /*!< TMR1 counter stopped when core is halted */
#if defined(DBGMCU_APB2F_TMR8_STS)
#define DDL_DBGMCU_APB2_GRP1_TMR8_STOP      DBGMCU_APB2F_TMR8_STS   /*!< TMR8 counter stopped when core is halted */
#endif /* DBGMCU_APB2F_TMR8_STS */
#define DDL_DBGMCU_APB2_GRP1_TMR9_STOP      DBGMCU_APB2F_TMR9_STS   /*!< TMR9 counter stopped when core is halted */
#if defined(DBGMCU_APB2F_TMR10_STS)
#define DDL_DBGMCU_APB2_GRP1_TMR10_STOP     DBGMCU_APB2F_TMR10_STS  /*!< TMR10 counter stopped when core is halted */
#endif /* DBGMCU_APB2F_TMR10_STS */
#define DDL_DBGMCU_APB2_GRP1_TMR11_STOP     DBGMCU_APB2F_TMR11_STS  /*!< TMR11 counter stopped when core is halted */
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_LATENCY FLASH LATENCY
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define DDL_FLASH_LATENCY_0                 FLASH_CTRL_WS_0WS   /*!< FLASH Zero wait state */
#define DDL_FLASH_LATENCY_1                 FLASH_CTRL_WS_1WS   /*!< FLASH One wait state */
#define DDL_FLASH_LATENCY_2                 FLASH_CTRL_WS_2WS   /*!< FLASH Two wait states */
#define DDL_FLASH_LATENCY_3                 FLASH_CTRL_WS_3WS   /*!< FLASH Three wait states */
#define DDL_FLASH_LATENCY_4                 FLASH_CTRL_WS_4WS   /*!< FLASH Four wait states */
#define DDL_FLASH_LATENCY_5                 FLASH_CTRL_WS_5WS   /*!< FLASH five wait state */
#define DDL_FLASH_LATENCY_6                 FLASH_CTRL_WS_6WS   /*!< FLASH six wait state */
#define DDL_FLASH_LATENCY_7                 FLASH_CTRL_WS_7WS   /*!< FLASH seven wait states */
#define DDL_FLASH_LATENCY_8                 FLASH_CTRL_WS_8WS   /*!< FLASH eight wait states */
#define DDL_FLASH_LATENCY_9                 FLASH_CTRL_WS_9WS   /*!< FLASH nine wait states */
#define DDL_FLASH_LATENCY_10                FLASH_CTRL_WS_10WS  /*!< FLASH ten wait states */
#define DDL_FLASH_LATENCY_11                FLASH_CTRL_WS_11WS  /*!< FLASH eleven wait states */
#define DDL_FLASH_LATENCY_12                FLASH_CTRL_WS_12WS  /*!< FLASH twelve wait states */
#define DDL_FLASH_LATENCY_13                FLASH_CTRL_WS_13WS  /*!< FLASH thirteen wait states */
#define DDL_FLASH_LATENCY_14                FLASH_CTRL_WS_14WS  /*!< FLASH fourteen wait states */
#define DDL_FLASH_LATENCY_15                FLASH_CTRL_WS_15WS  /*!< FLASH fifteen wait states */
#define DDL_FLASH_LATENCY_16                FLASH_CTRL_WS_16WS  /*!< FLASH sixteen wait states */
#define DDL_FLASH_LATENCY_17                FLASH_CTRL_WS_17WS  /*!< FLASH seventeen wait states */
#define DDL_FLASH_LATENCY_18                FLASH_CTRL_WS_18WS  /*!< FLASH eighteen wait states */
#define DDL_FLASH_LATENCY_19                FLASH_CTRL_WS_19WS  /*!< FLASH nineteen wait states */
#define DDL_FLASH_LATENCY_20                FLASH_CTRL_WS_20WS  /*!< FLASH twenty wait states */
#define DDL_FLASH_LATENCY_21                FLASH_CTRL_WS_21WS  /*!< FLASH twenty one wait states */
#define DDL_FLASH_LATENCY_22                FLASH_CTRL_WS_22WS  /*!< FLASH twenty two wait states */
#define DDL_FLASH_LATENCY_23                FLASH_CTRL_WS_23WS  /*!< FLASH twenty three wait states */
#define DDL_FLASH_LATENCY_24                FLASH_CTRL_WS_24WS  /*!< FLASH twenty four wait states */
#define DDL_FLASH_LATENCY_25                FLASH_CTRL_WS_25WS  /*!< FLASH twenty five wait states */
#define DDL_FLASH_LATENCY_26                FLASH_CTRL_WS_26WS  /*!< FLASH twenty six wait states */
#define DDL_FLASH_LATENCY_27                FLASH_CTRL_WS_27WS  /*!< FLASH twenty seven wait states */
#define DDL_FLASH_LATENCY_28                FLASH_CTRL_WS_28WS  /*!< FLASH twenty eight wait states */
#define DDL_FLASH_LATENCY_29                FLASH_CTRL_WS_29WS  /*!< FLASH twenty nine wait states */
#define DDL_FLASH_LATENCY_30                FLASH_CTRL_WS_30WS  /*!< FLASH thirty wait states */
#define DDL_FLASH_LATENCY_31                FLASH_CTRL_WS_31WS  /*!< FLASH thirty one wait states */
#else
#define DDL_FLASH_LATENCY_0                 FLASH_ACCTRL_WAITP_0WS   /*!< FLASH Zero wait state */
#define DDL_FLASH_LATENCY_1                 FLASH_ACCTRL_WAITP_1WS   /*!< FLASH One wait state */
#define DDL_FLASH_LATENCY_2                 FLASH_ACCTRL_WAITP_2WS   /*!< FLASH Two wait states */
#define DDL_FLASH_LATENCY_3                 FLASH_ACCTRL_WAITP_3WS   /*!< FLASH Three wait states */
#define DDL_FLASH_LATENCY_4                 FLASH_ACCTRL_WAITP_4WS   /*!< FLASH Four wait states */
#define DDL_FLASH_LATENCY_5                 FLASH_ACCTRL_WAITP_5WS   /*!< FLASH five wait state */
#define DDL_FLASH_LATENCY_6                 FLASH_ACCTRL_WAITP_6WS   /*!< FLASH six wait state */
#define DDL_FLASH_LATENCY_7                 FLASH_ACCTRL_WAITP_7WS   /*!< FLASH seven wait states */
#if defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define DDL_FLASH_LATENCY_8                 FLASH_ACCTRL_WAITP_8WS
#define DDL_FLASH_LATENCY_9                 FLASH_ACCTRL_WAITP_9WS
#define DDL_FLASH_LATENCY_10                FLASH_ACCTRL_WAITP_10WS
#define DDL_FLASH_LATENCY_11                FLASH_ACCTRL_WAITP_11WS
#define DDL_FLASH_LATENCY_12                FLASH_ACCTRL_WAITP_12WS
#define DDL_FLASH_LATENCY_13                FLASH_ACCTRL_WAITP_13WS
#define DDL_FLASH_LATENCY_14                FLASH_ACCTRL_WAITP_14WS
#define DDL_FLASH_LATENCY_15                FLASH_ACCTRL_WAITP_15WS
#define DDL_FLASH_LATENCY_16                FLASH_ACCTRL_WAITP_16WS
#define DDL_FLASH_LATENCY_17                FLASH_ACCTRL_WAITP_17WS
#define DDL_FLASH_LATENCY_18                FLASH_ACCTRL_WAITP_18WS
#define DDL_FLASH_LATENCY_19                FLASH_ACCTRL_WAITP_19WS
#define DDL_FLASH_LATENCY_20                FLASH_ACCTRL_WAITP_20WS
#define DDL_FLASH_LATENCY_21                FLASH_ACCTRL_WAITP_21WS
#define DDL_FLASH_LATENCY_22                FLASH_ACCTRL_WAITP_22WS
#define DDL_FLASH_LATENCY_23                FLASH_ACCTRL_WAITP_23WS
#define DDL_FLASH_LATENCY_24                FLASH_ACCTRL_WAITP_24WS
#define DDL_FLASH_LATENCY_25                FLASH_ACCTRL_WAITP_25WS
#define DDL_FLASH_LATENCY_26                FLASH_ACCTRL_WAITP_26WS
#define DDL_FLASH_LATENCY_27                FLASH_ACCTRL_WAITP_27WS
#define DDL_FLASH_LATENCY_28                FLASH_ACCTRL_WAITP_28WS
#define DDL_FLASH_LATENCY_29                FLASH_ACCTRL_WAITP_29WS
#define DDL_FLASH_LATENCY_30                FLASH_ACCTRL_WAITP_30WS
#define DDL_FLASH_LATENCY_31                FLASH_ACCTRL_WAITP_31WS
#define DDL_FLASH_LATENCY_32                FLASH_ACCTRL_WAITP_32WS
#define DDL_FLASH_LATENCY_33                FLASH_ACCTRL_WAITP_33WS
#define DDL_FLASH_LATENCY_34                FLASH_ACCTRL_WAITP_34WS
#define DDL_FLASH_LATENCY_35                FLASH_ACCTRL_WAITP_35WS
#define DDL_FLASH_LATENCY_36                FLASH_ACCTRL_WAITP_36WS
#define DDL_FLASH_LATENCY_37                FLASH_ACCTRL_WAITP_37WS
#define DDL_FLASH_LATENCY_38                FLASH_ACCTRL_WAITP_38WS
#define DDL_FLASH_LATENCY_39                FLASH_ACCTRL_WAITP_39WS
#define DDL_FLASH_LATENCY_40                FLASH_ACCTRL_WAITP_40WS
#define DDL_FLASH_LATENCY_41                FLASH_ACCTRL_WAITP_41WS
#define DDL_FLASH_LATENCY_42                FLASH_ACCTRL_WAITP_42WS
#define DDL_FLASH_LATENCY_43                FLASH_ACCTRL_WAITP_43WS
#define DDL_FLASH_LATENCY_44                FLASH_ACCTRL_WAITP_44WS
#define DDL_FLASH_LATENCY_45                FLASH_ACCTRL_WAITP_45WS
#define DDL_FLASH_LATENCY_46                FLASH_ACCTRL_WAITP_46WS
#define DDL_FLASH_LATENCY_47                FLASH_ACCTRL_WAITP_47WS
#define DDL_FLASH_LATENCY_48                FLASH_ACCTRL_WAITP_48WS
#define DDL_FLASH_LATENCY_49                FLASH_ACCTRL_WAITP_49WS
#define DDL_FLASH_LATENCY_50                FLASH_ACCTRL_WAITP_50WS
#define DDL_FLASH_LATENCY_51                FLASH_ACCTRL_WAITP_51WS
#define DDL_FLASH_LATENCY_52                FLASH_ACCTRL_WAITP_52WS
#define DDL_FLASH_LATENCY_53                FLASH_ACCTRL_WAITP_53WS
#define DDL_FLASH_LATENCY_54                FLASH_ACCTRL_WAITP_54WS
#define DDL_FLASH_LATENCY_55                FLASH_ACCTRL_WAITP_55WS
#define DDL_FLASH_LATENCY_56                FLASH_ACCTRL_WAITP_56WS
#define DDL_FLASH_LATENCY_57                FLASH_ACCTRL_WAITP_57WS
#define DDL_FLASH_LATENCY_58                FLASH_ACCTRL_WAITP_58WS
#define DDL_FLASH_LATENCY_59                FLASH_ACCTRL_WAITP_59WS
#define DDL_FLASH_LATENCY_60                FLASH_ACCTRL_WAITP_60WS
#define DDL_FLASH_LATENCY_61                FLASH_ACCTRL_WAITP_61WS
#define DDL_FLASH_LATENCY_62                FLASH_ACCTRL_WAITP_62WS
#define DDL_FLASH_LATENCY_63                FLASH_ACCTRL_WAITP_63WS
#define DDL_FLASH_LATENCY_64                FLASH_ACCTRL_WAITP_64WS
#define DDL_FLASH_LATENCY_65                FLASH_ACCTRL_WAITP_65WS
#define DDL_FLASH_LATENCY_66                FLASH_ACCTRL_WAITP_66WS
#define DDL_FLASH_LATENCY_67                FLASH_ACCTRL_WAITP_67WS
#define DDL_FLASH_LATENCY_68                FLASH_ACCTRL_WAITP_68WS
#define DDL_FLASH_LATENCY_69                FLASH_ACCTRL_WAITP_69WS
#define DDL_FLASH_LATENCY_70                FLASH_ACCTRL_WAITP_70WS
#define DDL_FLASH_LATENCY_71                FLASH_ACCTRL_WAITP_71WS
#define DDL_FLASH_LATENCY_72                FLASH_ACCTRL_WAITP_72WS
#define DDL_FLASH_LATENCY_73                FLASH_ACCTRL_WAITP_73WS
#define DDL_FLASH_LATENCY_74                FLASH_ACCTRL_WAITP_74WS
#define DDL_FLASH_LATENCY_75                FLASH_ACCTRL_WAITP_75WS
#define DDL_FLASH_LATENCY_76                FLASH_ACCTRL_WAITP_76WS
#define DDL_FLASH_LATENCY_77                FLASH_ACCTRL_WAITP_77WS
#define DDL_FLASH_LATENCY_78                FLASH_ACCTRL_WAITP_78WS
#define DDL_FLASH_LATENCY_79                FLASH_ACCTRL_WAITP_79WS
#define DDL_FLASH_LATENCY_80                FLASH_ACCTRL_WAITP_80WS
#define DDL_FLASH_LATENCY_81                FLASH_ACCTRL_WAITP_81WS
#define DDL_FLASH_LATENCY_82                FLASH_ACCTRL_WAITP_82WS
#define DDL_FLASH_LATENCY_83                FLASH_ACCTRL_WAITP_83WS
#define DDL_FLASH_LATENCY_84                FLASH_ACCTRL_WAITP_84WS
#define DDL_FLASH_LATENCY_85                FLASH_ACCTRL_WAITP_85WS
#define DDL_FLASH_LATENCY_86                FLASH_ACCTRL_WAITP_86WS
#define DDL_FLASH_LATENCY_87                FLASH_ACCTRL_WAITP_87WS
#define DDL_FLASH_LATENCY_88                FLASH_ACCTRL_WAITP_88WS
#define DDL_FLASH_LATENCY_89                FLASH_ACCTRL_WAITP_89WS
#define DDL_FLASH_LATENCY_90                FLASH_ACCTRL_WAITP_90WS
#define DDL_FLASH_LATENCY_91                FLASH_ACCTRL_WAITP_91WS
#define DDL_FLASH_LATENCY_92                FLASH_ACCTRL_WAITP_92WS
#define DDL_FLASH_LATENCY_93                FLASH_ACCTRL_WAITP_93WS
#define DDL_FLASH_LATENCY_94                FLASH_ACCTRL_WAITP_94WS
#define DDL_FLASH_LATENCY_95                FLASH_ACCTRL_WAITP_95WS
#define DDL_FLASH_LATENCY_96                FLASH_ACCTRL_WAITP_96WS
#define DDL_FLASH_LATENCY_97                FLASH_ACCTRL_WAITP_97WS
#define DDL_FLASH_LATENCY_98                FLASH_ACCTRL_WAITP_98WS
#define DDL_FLASH_LATENCY_99                FLASH_ACCTRL_WAITP_99WS
#define DDL_FLASH_LATENCY_100               FLASH_ACCTRL_WAITP_100WS
#define DDL_FLASH_LATENCY_101               FLASH_ACCTRL_WAITP_101WS
#define DDL_FLASH_LATENCY_102               FLASH_ACCTRL_WAITP_102WS
#define DDL_FLASH_LATENCY_103               FLASH_ACCTRL_WAITP_103WS
#define DDL_FLASH_LATENCY_104               FLASH_ACCTRL_WAITP_104WS
#define DDL_FLASH_LATENCY_105               FLASH_ACCTRL_WAITP_105WS
#define DDL_FLASH_LATENCY_106               FLASH_ACCTRL_WAITP_106WS
#define DDL_FLASH_LATENCY_107               FLASH_ACCTRL_WAITP_107WS
#define DDL_FLASH_LATENCY_108               FLASH_ACCTRL_WAITP_108WS
#define DDL_FLASH_LATENCY_109               FLASH_ACCTRL_WAITP_109WS
#define DDL_FLASH_LATENCY_110               FLASH_ACCTRL_WAITP_110WS
#define DDL_FLASH_LATENCY_111               FLASH_ACCTRL_WAITP_111WS
#define DDL_FLASH_LATENCY_112               FLASH_ACCTRL_WAITP_112WS
#define DDL_FLASH_LATENCY_113               FLASH_ACCTRL_WAITP_113WS
#define DDL_FLASH_LATENCY_114               FLASH_ACCTRL_WAITP_114WS
#define DDL_FLASH_LATENCY_115               FLASH_ACCTRL_WAITP_115WS
#define DDL_FLASH_LATENCY_116               FLASH_ACCTRL_WAITP_116WS
#define DDL_FLASH_LATENCY_117               FLASH_ACCTRL_WAITP_117WS
#define DDL_FLASH_LATENCY_118               FLASH_ACCTRL_WAITP_118WS
#define DDL_FLASH_LATENCY_119               FLASH_ACCTRL_WAITP_119WS
#define DDL_FLASH_LATENCY_120               FLASH_ACCTRL_WAITP_120WS
#define DDL_FLASH_LATENCY_121               FLASH_ACCTRL_WAITP_121WS
#define DDL_FLASH_LATENCY_122               FLASH_ACCTRL_WAITP_122WS
#define DDL_FLASH_LATENCY_123               FLASH_ACCTRL_WAITP_123WS
#define DDL_FLASH_LATENCY_124               FLASH_ACCTRL_WAITP_124WS
#define DDL_FLASH_LATENCY_125               FLASH_ACCTRL_WAITP_125WS
#define DDL_FLASH_LATENCY_126               FLASH_ACCTRL_WAITP_126WS
#define DDL_FLASH_LATENCY_127               FLASH_ACCTRL_WAITP_127WS
#endif /* APM32F423xx || APM32F425xx || APM32F427xx */
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/** @defgroup SYSTEM_DDL_EC_READ_INT_TIME FLASH Read Interrupt Time
  * @{
  */
#if defined(FLASH_ACCTRL_RD_INT_TIMING)
#define DDL_FLASH_TRC_FAST              0x00000000U                 /*!< FLASH Read Interrupt Timing Fast */
#define DDL_FLASH_TRC_SLOW              FLASH_ACCTRL_RD_INT_TIMING  /*!< FLASH Read Interrupt Timing Slow */
#endif /* FLASH_ACCTRL_RD_INT_TIMING */
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
#if defined(SYSCFG)
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
  *         @arg @ref DDL_SYSCFG_REMAP_DMC (*)
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
  *         @arg @ref DDL_SYSCFG_REMAP_DMC (*)
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
  return (READ_BIT(SYSCFG->CCCTRL, SYSCFG_CCCTRL_RDYFLG) == (SYSCFG_CCCTRL_RDYFLG));
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
#endif /* SYSCFG */


/** @defgroup SYSTEM_DDL_EF_DBGMCU DBGMCU
  * @{
  */

/**
  * @brief  Return the device identifier
  * @note For APM32F405/407xx and APM32F415/417xx devices, the device ID is 0x413
  * @note For APM32F411xx devices, the device ID is 0x431
  * @note For APM32F423/425/427xx devices, the device ID is 0x419
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
          For example, it is read as RevA -> 0x1000 for APM32F423/425/427xx devices
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
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_RTC_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_WWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_IWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C2_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C3_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN1_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN2_STOP (*)
  *
  * @param  Periphs This parameter can be a combination of the following values(Only for APM32F402/403xx device):
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR2_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR3_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR4_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR5_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_WWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_IWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN2_STOP
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_APB1_GRP1_FreezePeriph(uint32_t Periphs)
{
#if defined(APM32F403xx) || defined(APM32F402xx)
  SET_BIT(DBGMCU->CFG, Periphs);
#else
  SET_BIT(DBGMCU->APB1F, Periphs);
#endif /* APM32F403xx || APM32F402xx */
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
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_RTC_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_WWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_IWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C2_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C3_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN1_STOP (*)
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN2_STOP (*)
  *
  * @param  Periphs This parameter can be a combination of the following values(Only for APM32F402/403xx device):
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR2_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR3_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR4_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_TMR5_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_WWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_IWDT_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN1_STOP
  *         @arg @ref DDL_DBGMCU_APB1_GRP1_CAN2_STOP
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void DDL_DBGMCU_APB1_GRP1_UnFreezePeriph(uint32_t Periphs)
{
#if defined(APM32F403xx) || defined(APM32F402xx)
  CLEAR_BIT(DBGMCU->CFG, Periphs);
#else
  CLEAR_BIT(DBGMCU->APB1F, Periphs);
#endif /* APM32F403xx || APM32F402xx */
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
#if defined(APM32F403xx) || defined(APM32F402xx)
  SET_BIT(DBGMCU->CFG, Periphs);
#else
  SET_BIT(DBGMCU->APB2F, Periphs);
#endif /* APM32F403xx || APM32F402xx */
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
#if defined(APM32F403xx) || defined(APM32F402xx)
  CLEAR_BIT(DBGMCU->CFG, Periphs);
#else
  CLEAR_BIT(DBGMCU->APB2F, Periphs);
#endif /* APM32F403xx || APM32F402xx */
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
  *         @arg @ref DDL_FLASH_LATENCY_7 - DDL_FLASH_LATENCY_127 (*)
  *
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_SetLatency(uint32_t Latency)
{
#if defined(FLASH_ACCTRL_WAITP)
  MODIFY_REG(FLASH->ACCTRL, FLASH_ACCTRL_WAITP, Latency);
#else
  MODIFY_REG(FLASH->CTRL1, FLASH_CTRL1_WS02 | FLASH_CTRL1_WS34, Latency);
#endif /* FLASH_ACCTRL_WAITP */
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
  *         @arg @ref DDL_FLASH_LATENCY_7 - DDL_FLASH_LATENCY_127 (*)
  */
__STATIC_INLINE uint32_t DDL_FLASH_GetLatency(void)
{
#if defined(FLASH_ACCTRL_WAITP)
  return (uint32_t)(READ_BIT(FLASH->ACCTRL, FLASH_ACCTRL_WAITP));
#else
  uint32_t temp = READ_BIT(FLASH->CTRL1, FLASH_CTRL1_WS34);
  return (uint32_t)(READ_BIT(FLASH->CTRL1, FLASH_CTRL1_WS02) | temp);
#endif /* FLASH_ACCTRL_WAITP */
}

/**
  * @brief  Enable Prefetch
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnablePrefetch(void)
{
#if defined(FLASH_ACCTRL_PREFEN)
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREFEN);
#else
  SET_BIT(FLASH->CTRL1, FLASH_CTRL1_PBEN);
#endif /* FLASH_ACCTRL_PREFEN */
}

/**
  * @brief  Disable Prefetch
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisablePrefetch(void)
{
#if defined(FLASH_ACCTRL_PREFEN)
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREFEN);
#else
  CLEAR_BIT(FLASH->CTRL1, FLASH_CTRL1_PBEN);
#endif /* FLASH_ACCTRL_PREFEN */
}

/**
  * @brief  Check if Prefetch buffer is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_FLASH_IsPrefetchEnabled(void)
{
#if defined(FLASH_ACCTRL_PREFEN)
  return (READ_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREFEN) == (FLASH_ACCTRL_PREFEN));
#else
  return (READ_BIT(FLASH->CTRL1, FLASH_CTRL1_PBEN) == (FLASH_CTRL1_PBEN));
#endif /* FLASH_ACCTRL_PREFEN */
}

#if defined(FLASH_CTRL1_HCAEN)
/**
  * @brief  Enable Flash Half Cycle Access
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableHalfCycleAccess(void)
{
  SET_BIT(FLASH->CTRL1, FLASH_CTRL1_HCAEN);
}

/**
  * @brief  Disable Flash Half Cycle Access
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableHalfCycleAccess(void)
{
  CLEAR_BIT(FLASH->CTRL1, FLASH_CTRL1_HCAEN);
}

/**
  * @brief  Check if Flash Half Cycle Access is enabled or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_FLASH_IsHalfCycleAccessEnabled(void)
{
  return (READ_BIT(FLASH->CTRL1, FLASH_CTRL1_HCAEN) == (FLASH_CTRL1_HCAEN));
}
#endif /* FLASH_CTRL1_HCAEN */

/**
  * @brief  Enable Instruction cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableInstCache(void)
{
#if defined(FLASH_ACCTRL_ICACHEEN)
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHEEN);
#else
  SET_BIT(FLASH->CTRL1, FLASH_CTRL1_ICACHEEN);
#endif /* FLASH_ACCTRL_ICACHEEN */
}

/**
  * @brief  Disable Instruction cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableInstCache(void)
{
#if defined(FLASH_ACCTRL_ICACHEEN)
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHEEN);
#else
  CLEAR_BIT(FLASH->CTRL1, FLASH_CTRL1_ICACHEEN);
#endif /* FLASH_ACCTRL_ICACHEEN */
}

/**
  * @brief  Enable Data cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableDataCache(void)
{
#if defined(FLASH_ACCTRL_DCACHEEN)
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHEEN);
#else
  SET_BIT(FLASH->CTRL1, FLASH_CTRL1_DCACHEEN);
#endif /* FLASH_ACCTRL_DCACHEEN */
}

/**
  * @brief  Disable Data cache
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableDataCache(void)
{
#if defined(FLASH_ACCTRL_DCACHEEN)
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHEEN);
#else
  CLEAR_BIT(FLASH->CTRL1, FLASH_CTRL1_DCACHEEN);
#endif /* FLASH_ACCTRL_DCACHEEN */
}

/**
  * @brief  Enable Instruction cache reset
  * @note  bit can be written only when the instruction cache is disabled
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableInstCacheReset(void)
{
#if defined(FLASH_ACCTRL_ICACHERST)
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHERST);
#else
  SET_BIT(FLASH->CTRL1, FLASH_CTRL1_ICACHERST);
#endif /* FLASH_ACCTRL_ICACHERST */
}

/**
  * @brief  Disable Instruction cache reset
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableInstCacheReset(void)
{
#if defined(FLASH_ACCTRL_ICACHERST)
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_ICACHERST);
#else
  CLEAR_BIT(FLASH->CTRL1, FLASH_CTRL1_ICACHERST);
#endif /* FLASH_ACCTRL_ICACHERST */
}

/**
  * @brief  Enable Data cache reset
  * @note bit can be written only when the data cache is disabled
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableDataCacheReset(void)
{
#if defined(FLASH_ACCTRL_DCACHERST)
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHERST);
#else
  SET_BIT(FLASH->CTRL1, FLASH_CTRL1_DCACHERST);
#endif /* FLASH_ACCTRL_DCACHERST */
}

/**
  * @brief  Disable Data cache reset
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableDataCacheReset(void)
{
#if defined(FLASH_ACCTRL_DCACHERST)
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_DCACHERST);
#else
  CLEAR_BIT(FLASH->CTRL1, FLASH_CTRL1_DCACHERST);
#endif /* FLASH_ACCTRL_DCACHERST */
}

#if defined(FLASH_CTRL1_PRFTB) || defined(FLASH_ACCTRL_PREF_TIMING)
/**
  * @brief  Enable Prefetch policy control
  * @retval None
  * @note   Prefetch buffer/cache line initiates the next prefetch after being accessed.
  */
__STATIC_INLINE void DDL_FLASH_EnablePrefetchPolicyCtrl(void)
{
#if defined(FLASH_CTRL1_PRFTB)
  SET_BIT(FLASH->CTRL1, FLASH_CTRL1_PRFTB);
#else
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREF_TIMING);
#endif /* FLASH_CTRL1_PRFTB */
}

/**
  * @brief  Disable Prefetch policy control
  * @retval None
  * @note   Prefetch buffer / the same cache line initiates the next prefetch only after being accessed twice.
  */
__STATIC_INLINE void DDL_FLASH_DisablePrefetchPolicyCtrl(void)
{
#if defined(FLASH_CTRL1_PRFTB)
  CLEAR_BIT(FLASH->CTRL1, FLASH_CTRL1_PRFTB);
#else
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_PREF_TIMING);
#endif /* FLASH_CTRL1_PRFTB */
}

#endif /* FLASH_CTRL1_PRFTB || FLASH_ACCTRL_PREF_TIMING */

#if defined(FLASH_ACCTRL_RD_INT_EN)
/**
  * @brief  Enable Flash read interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_EnableReadInt(void)
{
  SET_BIT(FLASH->ACCTRL, FLASH_ACCTRL_RD_INT_EN);
}

/**
  * @brief  Disable Flash read interrupt
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_DisableReadInt(void)
{
  CLEAR_BIT(FLASH->ACCTRL, FLASH_ACCTRL_RD_INT_EN);
}
#endif /* FLASH_ACCTRL_RD_INT_EN */

#if defined(FLASH_ACCTRL_RD_INT_TIMING)
/**
  * @brief  Set Flash read interrupt timing
  * @param  Timing This parameter can be one of the following values:
  *         @arg @ref DDL_FLASH_TRC_FAST
  *         @arg @ref DDL_FLASH_TRC_SLOW
  * @retval None
  */
__STATIC_INLINE void DDL_FLASH_SetReadIntTiming(uint32_t Timing)
{
  MODIFY_REG(FLASH->ACCTRL, FLASH_ACCTRL_RD_INT_TIMING, Timing);
}
#endif /* FLASH_ACCTRL_RD_INT_TIMING */

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


