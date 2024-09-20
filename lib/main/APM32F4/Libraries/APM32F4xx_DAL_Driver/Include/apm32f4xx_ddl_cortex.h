/**
  *
  * @file    apm32f4xx_ddl_cortex.h
  * @brief   Header file of CORTEX DDL module.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The LL CORTEX driver contains a set of generic APIs that can be
    used by user:
      (+) SYSTICK configuration used by DDL_mDelay and DDL_Init1msTick
          functions
      (+) Low power mode configuration (SCB register of Cortex-MCU)
      (+) MPU API to configure and enable regions
          (MPU services provided only on some devices)
      (+) API to access to MCU info (CPUID register)
      (+) API to enable fault handler (SHCSR accesses)

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
#ifndef APM32F4xx_DDL_CORTEX_H
#define APM32F4xx_DDL_CORTEX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

/** @defgroup CORTEX_DDL CORTEX
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup CORTEX_DDL_Exported_Constants CORTEX Exported Constants
  * @{
  */

/** @defgroup CORTEX_DDL_EC_CLKSOURCE_HCLK SYSTICK Clock Source
  * @{
  */
#define DDL_SYSTICK_CLKSOURCE_HCLK_DIV8     0x00000000U                 /*!< AHB clock divided by 8 selected as SysTick clock source.*/
#define DDL_SYSTICK_CLKSOURCE_HCLK          SysTick_CTRL_CLKSOURCE_Msk  /*!< AHB clock selected as SysTick clock source. */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_FAULT Handler Fault type
  * @{
  */
#define DDL_HANDLER_FAULT_USG               SCB_SHCSR_USGFAULTENA_Msk              /*!< Usage fault */
#define DDL_HANDLER_FAULT_BUS               SCB_SHCSR_BUSFAULTENA_Msk              /*!< Bus fault */
#define DDL_HANDLER_FAULT_MEM               SCB_SHCSR_MEMFAULTENA_Msk              /*!< Memory management fault */
/**
  * @}
  */

#if __MPU_PRESENT

/** @defgroup CORTEX_DDL_EC_CTRL_HFNMI_PRIVDEF MPU Control
  * @{
  */
#define DDL_MPU_CTRL_HFNMI_PRIVDEF_NONE     0x00000000U                                       /*!< Disable NMI and privileged SW access */
#define DDL_MPU_CTRL_HARDFAULT_NMI          MPU_CTRL_HFNMIENA_Msk                             /*!< Enables the operation of MPU during hard fault, NMI, and FAULTMASK handlers */
#define DDL_MPU_CTRL_PRIVILEGED_DEFAULT     MPU_CTRL_PRIVDEFENA_Msk                           /*!< Enable privileged software access to default memory map */
#define DDL_MPU_CTRL_HFNMI_PRIVDEF          (MPU_CTRL_HFNMIENA_Msk | MPU_CTRL_PRIVDEFENA_Msk) /*!< Enable NMI and privileged SW access */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_REGION MPU Region Number
  * @{
  */
#define DDL_MPU_REGION_NUMBER0              0x00U /*!< REGION Number 0 */
#define DDL_MPU_REGION_NUMBER1              0x01U /*!< REGION Number 1 */
#define DDL_MPU_REGION_NUMBER2              0x02U /*!< REGION Number 2 */
#define DDL_MPU_REGION_NUMBER3              0x03U /*!< REGION Number 3 */
#define DDL_MPU_REGION_NUMBER4              0x04U /*!< REGION Number 4 */
#define DDL_MPU_REGION_NUMBER5              0x05U /*!< REGION Number 5 */
#define DDL_MPU_REGION_NUMBER6              0x06U /*!< REGION Number 6 */
#define DDL_MPU_REGION_NUMBER7              0x07U /*!< REGION Number 7 */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_REGION_SIZE MPU Region Size
  * @{
  */
#define DDL_MPU_REGION_SIZE_32B             (0x04U << MPU_RASR_SIZE_Pos) /*!< 32B Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_64B             (0x05U << MPU_RASR_SIZE_Pos) /*!< 64B Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_128B            (0x06U << MPU_RASR_SIZE_Pos) /*!< 128B Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_256B            (0x07U << MPU_RASR_SIZE_Pos) /*!< 256B Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_512B            (0x08U << MPU_RASR_SIZE_Pos) /*!< 512B Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_1KB             (0x09U << MPU_RASR_SIZE_Pos) /*!< 1KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_2KB             (0x0AU << MPU_RASR_SIZE_Pos) /*!< 2KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_4KB             (0x0BU << MPU_RASR_SIZE_Pos) /*!< 4KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_8KB             (0x0CU << MPU_RASR_SIZE_Pos) /*!< 8KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_16KB            (0x0DU << MPU_RASR_SIZE_Pos) /*!< 16KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_32KB            (0x0EU << MPU_RASR_SIZE_Pos) /*!< 32KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_64KB            (0x0FU << MPU_RASR_SIZE_Pos) /*!< 64KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_128KB           (0x10U << MPU_RASR_SIZE_Pos) /*!< 128KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_256KB           (0x11U << MPU_RASR_SIZE_Pos) /*!< 256KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_512KB           (0x12U << MPU_RASR_SIZE_Pos) /*!< 512KB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_1MB             (0x13U << MPU_RASR_SIZE_Pos) /*!< 1MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_2MB             (0x14U << MPU_RASR_SIZE_Pos) /*!< 2MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_4MB             (0x15U << MPU_RASR_SIZE_Pos) /*!< 4MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_8MB             (0x16U << MPU_RASR_SIZE_Pos) /*!< 8MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_16MB            (0x17U << MPU_RASR_SIZE_Pos) /*!< 16MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_32MB            (0x18U << MPU_RASR_SIZE_Pos) /*!< 32MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_64MB            (0x19U << MPU_RASR_SIZE_Pos) /*!< 64MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_128MB           (0x1AU << MPU_RASR_SIZE_Pos) /*!< 128MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_256MB           (0x1BU << MPU_RASR_SIZE_Pos) /*!< 256MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_512MB           (0x1CU << MPU_RASR_SIZE_Pos) /*!< 512MB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_1GB             (0x1DU << MPU_RASR_SIZE_Pos) /*!< 1GB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_2GB             (0x1EU << MPU_RASR_SIZE_Pos) /*!< 2GB Size of the MPU protection region */
#define DDL_MPU_REGION_SIZE_4GB             (0x1FU << MPU_RASR_SIZE_Pos) /*!< 4GB Size of the MPU protection region */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_REGION_PRIVILEDGES MPU Region Privileges
  * @{
  */
#define DDL_MPU_REGION_NO_ACCESS            (0x00U << MPU_RASR_AP_Pos) /*!< No access*/
#define DDL_MPU_REGION_PRIV_RW              (0x01U << MPU_RASR_AP_Pos) /*!< RW privileged (privileged access only)*/
#define DDL_MPU_REGION_PRIV_RW_URO          (0x02U << MPU_RASR_AP_Pos) /*!< RW privileged - RO user (Write in a user program generates a fault) */
#define DDL_MPU_REGION_FULL_ACCESS          (0x03U << MPU_RASR_AP_Pos) /*!< RW privileged & user (Full access) */
#define DDL_MPU_REGION_PRIV_RO              (0x05U << MPU_RASR_AP_Pos) /*!< RO privileged (privileged read only)*/
#define DDL_MPU_REGION_PRIV_RO_URO          (0x06U << MPU_RASR_AP_Pos) /*!< RO privileged & user (read only) */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_TEX MPU TEX Level
  * @{
  */
#define DDL_MPU_TEX_LEVEL0                  (0x00U << MPU_RASR_TEX_Pos) /*!< b000 for TEX bits */
#define DDL_MPU_TEX_LEVEL1                  (0x01U << MPU_RASR_TEX_Pos) /*!< b001 for TEX bits */
#define DDL_MPU_TEX_LEVEL2                  (0x02U << MPU_RASR_TEX_Pos) /*!< b010 for TEX bits */
#define DDL_MPU_TEX_LEVEL4                  (0x04U << MPU_RASR_TEX_Pos) /*!< b100 for TEX bits */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_INSTRUCTION_ACCESS MPU Instruction Access
  * @{
  */
#define DDL_MPU_INSTRUCTION_ACCESS_ENABLE   0x00U            /*!< Instruction fetches enabled */
#define DDL_MPU_INSTRUCTION_ACCESS_DISABLE  MPU_RASR_XN_Msk  /*!< Instruction fetches disabled*/
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_SHAREABLE_ACCESS MPU Shareable Access
  * @{
  */
#define DDL_MPU_ACCESS_SHAREABLE            MPU_RASR_S_Msk   /*!< Shareable memory attribute */
#define DDL_MPU_ACCESS_NOT_SHAREABLE        0x00U            /*!< Not Shareable memory attribute */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_CACHEABLE_ACCESS MPU Cacheable Access
  * @{
  */
#define DDL_MPU_ACCESS_CACHEABLE            MPU_RASR_C_Msk   /*!< Cacheable memory attribute */
#define DDL_MPU_ACCESS_NOT_CACHEABLE        0x00U            /*!< Not Cacheable memory attribute */
/**
  * @}
  */

/** @defgroup CORTEX_DDL_EC_BUFFERABLE_ACCESS MPU Bufferable Access
  * @{
  */
#define DDL_MPU_ACCESS_BUFFERABLE           MPU_RASR_B_Msk   /*!< Bufferable memory attribute */
#define DDL_MPU_ACCESS_NOT_BUFFERABLE       0x00U            /*!< Not Bufferable memory attribute */
/**
  * @}
  */
#endif /* __MPU_PRESENT */
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup CORTEX_DDL_Exported_Functions CORTEX Exported Functions
  * @{
  */

/** @defgroup CORTEX_DDL_EF_SYSTICK SYSTICK
  * @{
  */

/**
  * @brief  This function checks if the Systick counter flag is active or not.
  * @note   It can be used in timeout function on application side.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

/**
  * @brief  Configures the SysTick clock source
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK_DIV8
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK
  * @retval None
  */
__STATIC_INLINE void DDL_SYSTICK_SetClkSource(uint32_t Source)
{
  if (Source == DDL_SYSTICK_CLKSOURCE_HCLK)
  {
    SET_BIT(SysTick->CTRL, DDL_SYSTICK_CLKSOURCE_HCLK);
  }
  else
  {
    CLEAR_BIT(SysTick->CTRL, DDL_SYSTICK_CLKSOURCE_HCLK);
  }
}

/**
  * @brief  Get the SysTick clock source
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK_DIV8
  *         @arg @ref DDL_SYSTICK_CLKSOURCE_HCLK
  */
__STATIC_INLINE uint32_t DDL_SYSTICK_GetClkSource(void)
{
  return READ_BIT(SysTick->CTRL, DDL_SYSTICK_CLKSOURCE_HCLK);
}

/**
  * @brief  Enable SysTick exception request
  * @retval None
  */
__STATIC_INLINE void DDL_SYSTICK_EnableIT(void)
{
  SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief  Disable SysTick exception request
  * @retval None
  */
__STATIC_INLINE void DDL_SYSTICK_DisableIT(void)
{
  CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief  Checks if the SYSTICK interrupt is enabled or disabled.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_SYSTICK_IsEnabledIT(void)
{
  return (READ_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk) == (SysTick_CTRL_TICKINT_Msk));
}

/**
  * @}
  */

/** @defgroup CORTEX_DDL_EF_LOW_POWER_MODE LOW POWER MODE
  * @{
  */

/**
  * @brief  Processor uses sleep as its low power mode
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableSleep(void)
{
  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
}

/**
  * @brief  Processor uses deep sleep as its low power mode
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableDeepSleep(void)
{
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
}

/**
  * @brief  Configures sleep-on-exit when returning from Handler mode to Thread mode.
  * @note   Setting this bit to 1 enables an interrupt-driven application to avoid returning to an
  *         empty main application.
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableSleepOnExit(void)
{
  /* Set SLEEPONEXIT bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));
}

/**
  * @brief  Do not sleep when returning to Thread mode.
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_DisableSleepOnExit(void)
{
  /* Clear SLEEPONEXIT bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));
}

/**
  * @brief  Enabled events and all interrupts, including disabled interrupts, can wakeup the
  *         processor.
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_EnableEventOnPend(void)
{
  /* Set SEVEONPEND bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @brief  Only enabled interrupts or events can wakeup the processor, disabled interrupts are
  *         excluded
  * @retval None
  */
__STATIC_INLINE void DDL_LPM_DisableEventOnPend(void)
{
  /* Clear SEVEONPEND bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @}
  */

/** @defgroup CORTEX_DDL_EF_HANDLER HANDLER
  * @{
  */

/**
  * @brief  Enable a fault in System handler control register (SHCSR)
  * @param  Fault This parameter can be a combination of the following values:
  *         @arg @ref DDL_HANDLER_FAULT_USG
  *         @arg @ref DDL_HANDLER_FAULT_BUS
  *         @arg @ref DDL_HANDLER_FAULT_MEM
  * @retval None
  */
__STATIC_INLINE void DDL_HANDLER_EnableFault(uint32_t Fault)
{
  /* Enable the system handler fault */
  SET_BIT(SCB->SHCSR, Fault);
}

/**
  * @brief  Disable a fault in System handler control register (SHCSR)
  * @param  Fault This parameter can be a combination of the following values:
  *         @arg @ref DDL_HANDLER_FAULT_USG
  *         @arg @ref DDL_HANDLER_FAULT_BUS
  *         @arg @ref DDL_HANDLER_FAULT_MEM
  * @retval None
  */
__STATIC_INLINE void DDL_HANDLER_DisableFault(uint32_t Fault)
{
  /* Disable the system handler fault */
  CLEAR_BIT(SCB->SHCSR, Fault);
}

/**
  * @}
  */

/** @defgroup CORTEX_DDL_EF_MCU_INFO MCU INFO
  * @{
  */

/**
  * @brief  Get Implementer code
  * @retval Value should be equal to 0x41 for ARM
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetImplementer(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_IMPLEMENTER_Msk) >> SCB_CPUID_IMPLEMENTER_Pos);
}

/**
  * @brief  Get Variant number (The r value in the rnpn product revision identifier)
  * @retval Value between 0 and 255 (0x0: revision 0)
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetVariant(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_VARIANT_Msk) >> SCB_CPUID_VARIANT_Pos);
}

/**
  * @brief  Get Constant number
  * @retval Value should be equal to 0xF for Cortex-M4 devices
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetConstant(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_ARCHITECTURE_Msk) >> SCB_CPUID_ARCHITECTURE_Pos);
}

/**
  * @brief  Get Part number
  * @retval Value should be equal to 0xC24 for Cortex-M4
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetParNo(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_PARTNO_Msk) >> SCB_CPUID_PARTNO_Pos);
}

/**
  * @brief  Get Revision number (The p value in the rnpn product revision identifier, indicates patch release)
  * @retval Value between 0 and 255 (0x1: patch 1)
  */
__STATIC_INLINE uint32_t DDL_CPUID_GetRevision(void)
{
  return (uint32_t)(READ_BIT(SCB->CPUID, SCB_CPUID_REVISION_Msk) >> SCB_CPUID_REVISION_Pos);
}

/**
  * @}
  */

#if __MPU_PRESENT
/** @defgroup CORTEX_DDL_EF_MPU MPU
  * @{
  */

/**
  * @brief  Enable MPU with input options
  * @param  Options This parameter can be one of the following values:
  *         @arg @ref DDL_MPU_CTRL_HFNMI_PRIVDEF_NONE
  *         @arg @ref DDL_MPU_CTRL_HARDFAULT_NMI
  *         @arg @ref DDL_MPU_CTRL_PRIVILEGED_DEFAULT
  *         @arg @ref DDL_MPU_CTRL_HFNMI_PRIVDEF
  * @retval None
  */
__STATIC_INLINE void DDL_MPU_Enable(uint32_t Options)
{
  /* Enable the MPU*/
  WRITE_REG(MPU->CTRL, (MPU_CTRL_ENABLE_Msk | Options));
  /* Ensure MPU settings take effects */
  __DSB();
  /* Sequence instruction fetches using update settings */
  __ISB();
}

/**
  * @brief  Disable MPU
  * @retval None
  */
__STATIC_INLINE void DDL_MPU_Disable(void)
{
  /* Make sure outstanding transfers are done */
  __DMB();
  /* Disable MPU*/
  WRITE_REG(MPU->CTRL, 0U);
}

/**
  * @brief  Check if MPU is enabled or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_MPU_IsEnabled(void)
{
  return (READ_BIT(MPU->CTRL, MPU_CTRL_ENABLE_Msk) == (MPU_CTRL_ENABLE_Msk));
}

/**
  * @brief  Enable a MPU region
  * @param  Region This parameter can be one of the following values:
  *         @arg @ref DDL_MPU_REGION_NUMBER0
  *         @arg @ref DDL_MPU_REGION_NUMBER1
  *         @arg @ref DDL_MPU_REGION_NUMBER2
  *         @arg @ref DDL_MPU_REGION_NUMBER3
  *         @arg @ref DDL_MPU_REGION_NUMBER4
  *         @arg @ref DDL_MPU_REGION_NUMBER5
  *         @arg @ref DDL_MPU_REGION_NUMBER6
  *         @arg @ref DDL_MPU_REGION_NUMBER7
  * @retval None
  */
__STATIC_INLINE void DDL_MPU_EnableRegion(uint32_t Region)
{
  /* Set Region number */
  WRITE_REG(MPU->RNR, Region);
  /* Enable the MPU region */
  SET_BIT(MPU->RASR, MPU_RASR_ENABLE_Msk);
}

/**
  * @brief  Configure and enable a region
  * @param  Region This parameter can be one of the following values:
  *         @arg @ref DDL_MPU_REGION_NUMBER0
  *         @arg @ref DDL_MPU_REGION_NUMBER1
  *         @arg @ref DDL_MPU_REGION_NUMBER2
  *         @arg @ref DDL_MPU_REGION_NUMBER3
  *         @arg @ref DDL_MPU_REGION_NUMBER4
  *         @arg @ref DDL_MPU_REGION_NUMBER5
  *         @arg @ref DDL_MPU_REGION_NUMBER6
  *         @arg @ref DDL_MPU_REGION_NUMBER7
  * @param  Address Value of region base address
  * @param  SubRegionDisable Sub-region disable value between Min_Data = 0x00 and Max_Data = 0xFF
  * @param  Attributes This parameter can be a combination of the following values:
  *         @arg @ref DDL_MPU_REGION_SIZE_32B or @ref DDL_MPU_REGION_SIZE_64B or @ref DDL_MPU_REGION_SIZE_128B or @ref DDL_MPU_REGION_SIZE_256B or @ref DDL_MPU_REGION_SIZE_512B
  *           or @ref DDL_MPU_REGION_SIZE_1KB or @ref DDL_MPU_REGION_SIZE_2KB or @ref DDL_MPU_REGION_SIZE_4KB or @ref DDL_MPU_REGION_SIZE_8KB or @ref DDL_MPU_REGION_SIZE_16KB
  *           or @ref DDL_MPU_REGION_SIZE_32KB or @ref DDL_MPU_REGION_SIZE_64KB or @ref DDL_MPU_REGION_SIZE_128KB or @ref DDL_MPU_REGION_SIZE_256KB or @ref DDL_MPU_REGION_SIZE_512KB
  *           or @ref DDL_MPU_REGION_SIZE_1MB or @ref DDL_MPU_REGION_SIZE_2MB or @ref DDL_MPU_REGION_SIZE_4MB or @ref DDL_MPU_REGION_SIZE_8MB or @ref DDL_MPU_REGION_SIZE_16MB
  *           or @ref DDL_MPU_REGION_SIZE_32MB or @ref DDL_MPU_REGION_SIZE_64MB or @ref DDL_MPU_REGION_SIZE_128MB or @ref DDL_MPU_REGION_SIZE_256MB or @ref DDL_MPU_REGION_SIZE_512MB
  *           or @ref DDL_MPU_REGION_SIZE_1GB or @ref DDL_MPU_REGION_SIZE_2GB or @ref DDL_MPU_REGION_SIZE_4GB
  *         @arg @ref DDL_MPU_REGION_NO_ACCESS or @ref DDL_MPU_REGION_PRIV_RW or @ref DDL_MPU_REGION_PRIV_RW_URO or @ref DDL_MPU_REGION_FULL_ACCESS
  *           or @ref DDL_MPU_REGION_PRIV_RO or @ref DDL_MPU_REGION_PRIV_RO_URO
  *         @arg @ref DDL_MPU_TEX_LEVEL0 or @ref DDL_MPU_TEX_LEVEL1 or @ref DDL_MPU_TEX_LEVEL2 or @ref DDL_MPU_TEX_LEVEL4
  *         @arg @ref DDL_MPU_INSTRUCTION_ACCESS_ENABLE or  @ref DDL_MPU_INSTRUCTION_ACCESS_DISABLE
  *         @arg @ref DDL_MPU_ACCESS_SHAREABLE or @ref DDL_MPU_ACCESS_NOT_SHAREABLE
  *         @arg @ref DDL_MPU_ACCESS_CACHEABLE or @ref DDL_MPU_ACCESS_NOT_CACHEABLE
  *         @arg @ref DDL_MPU_ACCESS_BUFFERABLE or @ref DDL_MPU_ACCESS_NOT_BUFFERABLE
  * @retval None
  */
__STATIC_INLINE void DDL_MPU_ConfigRegion(uint32_t Region, uint32_t SubRegionDisable, uint32_t Address, uint32_t Attributes)
{
  /* Set Region number */
  WRITE_REG(MPU->RNR, Region);
  /* Set base address */
  WRITE_REG(MPU->RBAR, (Address & 0xFFFFFFE0U));
  /* Configure MPU */
  WRITE_REG(MPU->RASR, (MPU_RASR_ENABLE_Msk | Attributes | SubRegionDisable << MPU_RASR_SRD_Pos));
}

/**
  * @brief  Disable a region
  * @param  Region This parameter can be one of the following values:
  *         @arg @ref DDL_MPU_REGION_NUMBER0
  *         @arg @ref DDL_MPU_REGION_NUMBER1
  *         @arg @ref DDL_MPU_REGION_NUMBER2
  *         @arg @ref DDL_MPU_REGION_NUMBER3
  *         @arg @ref DDL_MPU_REGION_NUMBER4
  *         @arg @ref DDL_MPU_REGION_NUMBER5
  *         @arg @ref DDL_MPU_REGION_NUMBER6
  *         @arg @ref DDL_MPU_REGION_NUMBER7
  * @retval None
  */
__STATIC_INLINE void DDL_MPU_DisableRegion(uint32_t Region)
{
  /* Set Region number */
  WRITE_REG(MPU->RNR, Region);
  /* Disable the MPU region */
  CLEAR_BIT(MPU->RASR, MPU_RASR_ENABLE_Msk);
}

/**
  * @}
  */

#endif /* __MPU_PRESENT */
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

#endif /* APM32F4xx_DDL_CORTEX_H */

