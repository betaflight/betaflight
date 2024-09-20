/**
  *
  * @file    apm32f4xx_ddl_pmu.h
  * @brief   Header file of PMU DDL module.
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
#ifndef APM32F4xx_DDL_PMU_H
#define APM32F4xx_DDL_PMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(PMU)

/** @defgroup PMU_DDL PMU
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup PMU_DDL_Exported_Constants PMU Exported Constants
  * @{
  */

/** @defgroup PMU_DDL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with DDL_PMU_WriteReg function
  * @{
  */
#define DDL_PMU_CTRL_SBFLGCLR                     PMU_CTRL_SBFLGCLR            /*!< Clear standby flag */
#define DDL_PMU_CTRL_WUFLGCLR                     PMU_CTRL_WUFLGCLR            /*!< Clear wakeup flag */
/**
  * @}
  */

/** @defgroup PMU_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_PMU_ReadReg function
  * @{
  */
#define DDL_PMU_CSTS_WUEFLG                     PMU_CSTS_WUEFLG             /*!< Wakeup flag */
#define DDL_PMU_CSTS_SBFLG                      PMU_CSTS_SBFLG              /*!< Standby flag */
#define DDL_PMU_CSTS_PVDOFLG                    PMU_CSTS_PVDOFLG            /*!< Power voltage detector output flag */
#define DDL_PMU_CSTS_VOS                        PMU_CSTS_VOSRFLG            /*!< Voltage scaling select flag */
#if defined(PMU_CSTS_WKUPCFG)
#define DDL_PMU_CSTS_WKUPCFG1                   PMU_CSTS_WKUPCFG            /*!< Enable WKUP pin */
#elif defined(PMU_CSTS_WKUPCFG1)
#define DDL_PMU_CSTS_WKUPCFG1                   PMU_CSTS_WKUPCFG1           /*!< Enable WKUP pin 1 */
#endif /* PMU_CSTS_WKUPCFG */
#if defined(PMU_CSTS_WKUPCFG2)
#define DDL_PMU_CSTS_WKUPCFG2                   PMU_CSTS_WKUPCFG2           /*!< Enable WKUP pin 2 */
#endif /* PMU_CSTS_WKUPCFG2 */
#if defined(PMU_CSTS_WKUPCFG3)
#define DDL_PMU_CSTS_WKUPCFG3                   PMU_CSTS_WKUPCFG3           /*!< Enable WKUP pin 3 */
#endif /* PMU_CSTS_WKUPCFG3 */
/**
  * @}
  */

/** @defgroup PMU_DDL_EC_REGU_VOLTAGE Regulator Voltage
  * @{
  */
#if defined(PMU_CTRL_VOSSEL_0)
#define DDL_PMU_REGU_VOLTAGE_SCALE3         (PMU_CTRL_VOSSEL_0)
#define DDL_PMU_REGU_VOLTAGE_SCALE2         (PMU_CTRL_VOSSEL_1)
#define DDL_PMU_REGU_VOLTAGE_SCALE1         (PMU_CTRL_VOSSEL_0 | PMU_CTRL_VOSSEL_1)
#else
#define DDL_PMU_REGU_VOLTAGE_SCALE1         (PMU_CTRL_VOSSEL)
#define DDL_PMU_REGU_VOLTAGE_SCALE2         0x00000000U
#endif /* PMU_CTRL_VOSSEL_0 */
/**
  * @}
  */

/** @defgroup PMU_DDL_EC_MODE_PMU Mode Power
  * @{
  */
#define DDL_PMU_MODE_STOP_MAINREGU              0x00000000U                                             /*!< Enter Stop mode when the CPU enters deepsleep */
#define DDL_PMU_MODE_STOP_LPREGU                (PMU_CTRL_LPDSCFG)                                      /*!< Enter Stop mode (with low power Regulator ON) when the CPU enters deepsleep */
#if defined(PMU_CTRL_MRUDS) && defined(PMU_CTRL_LPUDS) && defined(PMU_CTRL_FPDSM)
#define DDL_PMU_MODE_STOP_MAINREGU_UNDERDRIVE   (PMU_CTRL_MRUDS | PMU_CTRL_FPDSM)                       /*!< Enter Stop mode (with main Regulator in under-drive mode) when the CPU enters deepsleep */
#define DDL_PMU_MODE_STOP_LPREGU_UNDERDRIVE     (PMU_CTRL_LPDSCFG | PMU_CTRL_LPUDS | PMU_CTRL_FPDSM)    /*!< Enter Stop mode (with low power Regulator in under-drive mode) when the CPU enters deepsleep */
#endif /* PMU_CTRL_MRUDS && PMU_CTRL_LPUDS && PMU_CTRL_FPDSM */
#if defined(PMU_CTRL_MRLV) && defined(PMU_CTRL_LPRLV) && defined(PMU_CTRL_FPDSM)
#define DDL_PMU_MODE_STOP_MAINREGU_DEEPSLEEP    (PMU_CTRL_MRLV | PMU_CTRL_FPDSM)                        /*!< Enter Stop mode (with main Regulator in Deep Sleep mode) when the CPU enters deepsleep */
#define DDL_PMU_MODE_STOP_LPREGU_DEEPSLEEP      (PMU_CTRL_LPDSCFG | PMU_CTRL_LPRLV | PMU_CTRL_FPDSM)    /*!< Enter Stop mode (with low power Regulator in Deep Sleep mode) when the CPU enters deepsleep */
#endif /* PMU_CTRL_MRLV && PMU_CTRL_LPRLV && PMU_CTRL_FPDSM */
#define DDL_PMU_MODE_STANDBY                    (PMU_CTRL_PDDSCFG)                                      /*!< Enter Standby mode when the CPU enters deepsleep */
/**
  * @}
  */

/** @defgroup PMU_DDL_EC_REGU_MODE_DS_MODE  Regulator Mode In Deep Sleep Mode
 * @{
 */
#define DDL_PMU_REGU_DSMODE_MAIN            0x00000000U                 /*!< Voltage Regulator in main mode during deepsleep mode */
#define DDL_PMU_REGU_DSMODE_LOW_POWER       (PMU_CTRL_LPDSCFG)          /*!< Voltage Regulator in low-power mode during deepsleep mode */
/**
  * @}
  */

/** @defgroup PMU_DDL_EC_PVDLEVEL Power Voltage Detector Level
  * @{
  */
#define DDL_PMU_PVDLEVEL_0                  (PMU_CTRL_PLSEL_LEV0)      /*!< Voltage threshold detected by PVD 2.2 V */
#define DDL_PMU_PVDLEVEL_1                  (PMU_CTRL_PLSEL_LEV1)      /*!< Voltage threshold detected by PVD 2.3 V */
#define DDL_PMU_PVDLEVEL_2                  (PMU_CTRL_PLSEL_LEV2)      /*!< Voltage threshold detected by PVD 2.4 V */
#define DDL_PMU_PVDLEVEL_3                  (PMU_CTRL_PLSEL_LEV3)      /*!< Voltage threshold detected by PVD 2.5 V */
#define DDL_PMU_PVDLEVEL_4                  (PMU_CTRL_PLSEL_LEV4)      /*!< Voltage threshold detected by PVD 2.6 V */
#define DDL_PMU_PVDLEVEL_5                  (PMU_CTRL_PLSEL_LEV5)      /*!< Voltage threshold detected by PVD 2.7 V */
#define DDL_PMU_PVDLEVEL_6                  (PMU_CTRL_PLSEL_LEV6)      /*!< Voltage threshold detected by PVD 2.8 V */
#define DDL_PMU_PVDLEVEL_7                  (PMU_CTRL_PLSEL_LEV7)      /*!< Voltage threshold detected by PVD 2.9 V */
/**
  * @}
  */
/** @defgroup PMU_DDL_EC_WAKEUP_PIN  Wakeup Pins
  * @{
  */
#if defined(PMU_CSTS_WKUPCFG)
#define DDL_PMU_WAKEUP_PIN1                 (PMU_CSTS_WKUPCFG)         /*!< WKUP pin : PA0 */
#endif /* PMU_CSTS_WKUPCFG */
#if defined(PMU_CSTS_WKUPCFG1)
#define DDL_PMU_WAKEUP_PIN1                 (PMU_CSTS_WKUPCFG1)        /*!< WKUP pin 1 : PA0 */
#endif /* PMU_CSTS_WKUPCFG1 */
#if defined(PMU_CSTS_WKUPCFG2)
#define DDL_PMU_WAKEUP_PIN2                 (PMU_CSTS_WKUPCFG2)        /*!< WKUP pin 2 : PC0 or PC13 according to device */
#endif /* PMU_CSTS_WKUPCFG2 */
#if defined(PMU_CSTS_WKUPCFG3)
#define DDL_PMU_WAKEUP_PIN3                 (PMU_CSTS_WKUPCFG3)        /*!< WKUP pin 3 : PC1 */
#endif /* PMU_CSTS_WKUPCFG3 */
/**
  * @}
  */

/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/** @defgroup PMU_DDL_Exported_Macros PMU Exported Macros
  * @{
  */

/** @defgroup PMU_DDL_EM_WRITE_READ Common write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in PMU register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_PMU_WriteReg(__REG__, __VALUE__) WRITE_REG(PMU->__REG__, (__VALUE__))

/**
  * @brief  Read a value in PMU register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_PMU_ReadReg(__REG__) READ_REG(PMU->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PMU_DDL_Exported_Functions PMU Exported Functions
  * @{
  */

/** @defgroup PMU_DDL_EF_Configuration Configuration
  * @{
  */
#if defined(PMU_CTRL_FLASHEN)
/**
  * @brief  Enable FLASH interface STOP while system Run is ON
  * @note  This mode is enabled only with STOP low power mode.
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableFLASHInterfaceSTOP(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_FLASHEN);
}

/**
  * @brief  Disable FLASH Interface STOP while system Run is ON
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableFLASHInterfaceSTOP(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_FLASHEN);
}

/**
  * @brief  Check if FLASH Interface STOP while system Run feature is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledFLASHInterfaceSTOP(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_FLASHEN) == (PMU_CTRL_FLASHEN));
}
#endif /* PMU_CTRL_FLASHEN */

#if defined(PMU_CTRL_FSMODE)
/**
  * @brief  Enable FLASH Memory STOP while system Run is ON
  * @note  This mode is enabled only with STOP low power mode.
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableFLASHMemorySTOP(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_FSMODE);
}

/**
  * @brief  Disable FLASH Memory STOP while system Run is ON
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableFLASHMemorySTOP(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_FSMODE);
}

/**
  * @brief  Check if FLASH Memory STOP while system Run feature is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledFLASHMemorySTOP(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_FSMODE) == (PMU_CTRL_FSMODE));
}
#endif /* PMU_CTRL_FSMODE */
#if defined(PMU_CTRL_UDEN)
/**
  * @brief  Enable Under Drive Mode
  * @note  This mode is enabled only with STOP low power mode.
  *        In this mode, the 1.2V domain is preserved in reduced leakage mode. This 
  *        mode is only available when the main Regulator or the low power Regulator 
  *        is in low voltage mode.      
  * @note  If the Under-drive mode was enabled, it is automatically disabled after 
  *        exiting Stop mode. 
  *        When the voltage Regulator operates in Under-drive mode, an additional  
  *        startup delay is induced when waking up from Stop mode.
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableUnderDriveMode(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_UDEN);
}

/**
  * @brief  Disable Under Drive Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableUnderDriveMode(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_UDEN);
}

/**
  * @brief  Check if Under Drive Mode is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledUnderDriveMode(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_UDEN) == (PMU_CTRL_UDEN));
}
#endif /* PMU_CTRL_UDEN */

#if defined(PMU_CTRL_ODSWEN)
/**
  * @brief  Enable Over drive switching
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableOverDriveSwitching(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_ODSWEN);
}

/**
  * @brief  Disable Over drive switching
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableOverDriveSwitching(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_ODSWEN);
}

/**
  * @brief  Check if Over drive switching is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledOverDriveSwitching(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_ODSWEN) == (PMU_CTRL_ODSWEN));
}
#endif /* PMU_CTRL_ODSWEN */
#if defined(PMU_CTRL_ODEN)
/**
  * @brief  Enable Over drive Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableOverDriveMode(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_ODEN);
}

/**
  * @brief  Disable Over drive Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableOverDriveMode(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_ODEN);
}

/**
  * @brief  Check if Over drive switching is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledOverDriveMode(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_ODEN) == (PMU_CTRL_ODEN));
}
#endif /* PMU_CTRL_ODEN */
#if defined(PMU_CTRL_MRUDS)
/**
  * @brief  Enable Main Regulator in deepsleep under-drive Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableMainRegulatorDeepSleepUDMode(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_MRUDS);
}

/**
  * @brief  Disable Main Regulator in deepsleep under-drive Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableMainRegulatorDeepSleepUDMode(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_MRUDS);
}

/**
  * @brief  Check if Main Regulator in deepsleep under-drive Mode is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledMainRegulatorDeepSleepUDMode(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_MRUDS) == (PMU_CTRL_MRUDS));
}
#endif /* PMU_CTRL_MRUDS */

#if defined(PMU_CTRL_LPUDS)
/**
  * @brief  Enable Low Power Regulator in deepsleep under-drive Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableLowPowerRegulatorDeepSleepUDMode(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_LPUDS);
}

/**
  * @brief  Disable Low Power Regulator in deepsleep under-drive Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableLowPowerRegulatorDeepSleepUDMode(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_LPUDS);
}

/**
  * @brief  Check if Low Power Regulator in deepsleep under-drive Mode is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledLowPowerRegulatorDeepSleepUDMode(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_LPUDS) == (PMU_CTRL_LPUDS));
}
#endif /* PMU_CTRL_LPUDS */

#if defined(PMU_CTRL_MRLV)
/**
  * @brief  Enable Main Regulator low voltage Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableMainRegulatorLowVoltageMode(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_MRLV);
}

/**
  * @brief  Disable Main Regulator low voltage Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableMainRegulatorLowVoltageMode(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_MRLV);
}

/**
  * @brief  Check if Main Regulator low voltage Mode is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledMainRegulatorLowVoltageMode(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_MRLV) == (PMU_CTRL_MRLV));
}
#endif /* PMU_CTRL_MRLV */

#if defined(PMU_CTRL_LPRLV)
/**
  * @brief  Enable Low Power Regulator low voltage Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableLowPowerRegulatorLowVoltageMode(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_LPRLV);
}

/**
  * @brief  Disable Low Power Regulator low voltage Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableLowPowerRegulatorLowVoltageMode(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_LPRLV);
}

/**
  * @brief  Check if Low Power Regulator low voltage Mode is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledLowPowerRegulatorLowVoltageMode(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_LPRLV) == (PMU_CTRL_LPRLV));
}
#endif /* PMU_CTRL_LPRLV */
/**
  * @brief  Set the main internal Regulator output voltage
  * @param  VoltageScaling This parameter can be one of the following values:
  *         @arg @ref DDL_PMU_REGU_VOLTAGE_SCALE1 (*)
  *         @arg @ref DDL_PMU_REGU_VOLTAGE_SCALE2
  *         @arg @ref DDL_PMU_REGU_VOLTAGE_SCALE3
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetRegulVoltageScaling(uint32_t VoltageScaling)
{
  MODIFY_REG(PMU->CTRL, PMU_CTRL_VOSSEL, VoltageScaling);
}

/**
  * @brief  Get the main internal Regulator output voltage
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_PMU_REGU_VOLTAGE_SCALE1 (*)
  *         @arg @ref DDL_PMU_REGU_VOLTAGE_SCALE2
  *         @arg @ref DDL_PMU_REGU_VOLTAGE_SCALE3
  */
__STATIC_INLINE uint32_t DDL_PMU_GetRegulVoltageScaling(void)
{
  return (uint32_t)(READ_BIT(PMU->CTRL, PMU_CTRL_VOSSEL));
}
/**
  * @brief  Enable the Flash Power Down in Stop Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableFlashPowerDown(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_FPDSM);
}

/**
  * @brief  Disable the Flash Power Down in Stop Mode
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableFlashPowerDown(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_FPDSM);
}

/**
  * @brief  Check if the Flash Power Down in Stop Mode is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledFlashPowerDown(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_FPDSM) == (PMU_CTRL_FPDSM));
}

/**
  * @brief  Enable access to the backup domain
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableBkUpAccess(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_BPWEN);
}

/**
  * @brief  Disable access to the backup domain
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableBkUpAccess(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_BPWEN);
}

/**
  * @brief  Check if the backup domain is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledBkUpAccess(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_BPWEN) == (PMU_CTRL_BPWEN));
}
/**
  * @brief  Enable the backup Regulator
  * @note The BRE bit of the PMU_CSTS register is protected against parasitic write access.
  * The DDL_PMU_EnableBkUpAccess() must be called before using this API.
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableBkUpRegulator(void)
{
  SET_BIT(PMU->CSTS, PMU_CSTS_BKPREN);
}

/**
  * @brief  Disable the backup Regulator
  * @note The BRE bit of the PMU_CSTS register is protected against parasitic write access.
  * The DDL_PMU_EnableBkUpAccess() must be called before using this API.
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableBkUpRegulator(void)
{
  CLEAR_BIT(PMU->CSTS, PMU_CSTS_BKPREN);
}

/**
  * @brief  Check if the backup Regulator is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledBkUpRegulator(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_BKPREN) == (PMU_CSTS_BKPREN));
}

/**
  * @brief  Set voltage Regulator mode during deep sleep mode
  * @param  RegulMode This parameter can be one of the following values:
  *         @arg @ref DDL_PMU_REGU_DSMODE_MAIN
  *         @arg @ref DDL_PMU_REGU_DSMODE_LOW_POWER
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetRegulModeDS(uint32_t RegulMode)
{
  MODIFY_REG(PMU->CTRL, PMU_CTRL_LPDSCFG, RegulMode);
}

/**
  * @brief  Get voltage Regulator mode during deep sleep mode
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_PMU_REGU_DSMODE_MAIN
  *         @arg @ref DDL_PMU_REGU_DSMODE_LOW_POWER
  */
__STATIC_INLINE uint32_t DDL_PMU_GetRegulModeDS(void)
{
  return (uint32_t)(READ_BIT(PMU->CTRL, PMU_CTRL_LPDSCFG));
}

/**
  * @brief  Set Power Down mode when CPU enters deepsleep
  * @param  PDMode This parameter can be one of the following values:
  *         @arg @ref DDL_PMU_MODE_STOP_MAINREGU
  *         @arg @ref DDL_PMU_MODE_STOP_LPREGU
  *         @arg @ref DDL_PMU_MODE_STOP_MAINREGU_UNDERDRIVE (*)
  *         @arg @ref DDL_PMU_MODE_STOP_LPREGU_UNDERDRIVE (*)
  *         @arg @ref DDL_PMU_MODE_STOP_MAINREGU_DEEPSLEEP (*)
  *         @arg @ref DDL_PMU_MODE_STOP_LPREGU_DEEPSLEEP (*)
  *
  *         (*) not available on all devices
  *         @arg @ref DDL_PMU_MODE_STANDBY
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetPowerMode(uint32_t PDMode)
{
#if defined(PMU_CTRL_MRUDS) && defined(PMU_CTRL_LPUDS) && defined(PMU_CTRL_FPDSM)
  MODIFY_REG(PMU->CTRL, (PMU_CTRL_PDDSCFG | PMU_CTRL_LPDSCFG | PMU_CTRL_FPDSM | PMU_CTRL_LPUDS | PMU_CTRL_MRUDS), PDMode);
#elif defined(PMU_CTRL_MRLV) && defined(PMU_CTRL_LPRLV) && defined(PMU_CTRL_FPDSM)
  MODIFY_REG(PMU->CTRL, (PMU_CTRL_PDDSCFG | PMU_CTRL_LPDSCFG | PMU_CTRL_FPDSM | PMU_CTRL_LPRLV | PMU_CTRL_MRLV), PDMode);
#else
  MODIFY_REG(PMU->CTRL, (PMU_CTRL_PDDSCFG| PMU_CTRL_LPDSCFG), PDMode);
#endif /* PMU_CTRL_MRUDS && PMU_CTRL_LPUDS && PMU_CTRL_FPDSM */
}

/**
  * @brief  Get Power Down mode when CPU enters deepsleep
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_PMU_MODE_STOP_MAINREGU
  *         @arg @ref DDL_PMU_MODE_STOP_LPREGU
  *         @arg @ref DDL_PMU_MODE_STOP_MAINREGU_UNDERDRIVE (*)
  *         @arg @ref DDL_PMU_MODE_STOP_LPREGU_UNDERDRIVE (*)
  *         @arg @ref DDL_PMU_MODE_STOP_MAINREGU_DEEPSLEEP (*)
  *         @arg @ref DDL_PMU_MODE_STOP_LPREGU_DEEPSLEEP (*)
  *
  *         (*) not available on all devices
  *         @arg @ref DDL_PMU_MODE_STANDBY
  */
__STATIC_INLINE uint32_t DDL_PMU_GetPowerMode(void)
{
#if defined(PMU_CTRL_MRUDS) && defined(PMU_CTRL_LPUDS) && defined(PMU_CTRL_FPDSM)
  return (uint32_t)(READ_BIT(PMU->CTRL, (PMU_CTRL_PDDSCFG | PMU_CTRL_LPDSCFG | PMU_CTRL_FPDSM | PMU_CTRL_LPUDS | PMU_CTRL_MRUDS)));
#elif defined(PMU_CTRL_MRLV) && defined(PMU_CTRL_LPRLV) && defined(PMU_CTRL_FPDSM)
  return (uint32_t)(READ_BIT(PMU->CTRL, (PMU_CTRL_PDDSCFG | PMU_CTRL_LPDSCFG | PMU_CTRL_FPDSM | PMU_CTRL_LPRLV | PMU_CTRL_MRLV)));
#else
  return (uint32_t)(READ_BIT(PMU->CTRL, (PMU_CTRL_PDDSCFG| PMU_CTRL_LPDSCFG)));
#endif /* PMU_CTRL_MRUDS && PMU_CTRL_LPUDS && PMU_CTRL_FPDSM */
}

/**
  * @brief  Configure the voltage threshold detected by the Power Voltage Detector
  * @param  PVDLevel This parameter can be one of the following values:
  *         @arg @ref DDL_PMU_PVDLEVEL_0
  *         @arg @ref DDL_PMU_PVDLEVEL_1
  *         @arg @ref DDL_PMU_PVDLEVEL_2
  *         @arg @ref DDL_PMU_PVDLEVEL_3
  *         @arg @ref DDL_PMU_PVDLEVEL_4
  *         @arg @ref DDL_PMU_PVDLEVEL_5
  *         @arg @ref DDL_PMU_PVDLEVEL_6
  *         @arg @ref DDL_PMU_PVDLEVEL_7
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_SetPVDLevel(uint32_t PVDLevel)
{
  MODIFY_REG(PMU->CTRL, PMU_CTRL_PLSEL, PVDLevel);
}

/**
  * @brief  Get the voltage threshold detection
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_PMU_PVDLEVEL_0
  *         @arg @ref DDL_PMU_PVDLEVEL_1
  *         @arg @ref DDL_PMU_PVDLEVEL_2
  *         @arg @ref DDL_PMU_PVDLEVEL_3
  *         @arg @ref DDL_PMU_PVDLEVEL_4
  *         @arg @ref DDL_PMU_PVDLEVEL_5
  *         @arg @ref DDL_PMU_PVDLEVEL_6
  *         @arg @ref DDL_PMU_PVDLEVEL_7
  */
__STATIC_INLINE uint32_t DDL_PMU_GetPVDLevel(void)
{
  return (uint32_t)(READ_BIT(PMU->CTRL, PMU_CTRL_PLSEL));
}

/**
  * @brief  Enable Power Voltage Detector
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnablePVD(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_PVDEN);
}

/**
  * @brief  Disable Power Voltage Detector
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisablePVD(void)
{
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_PVDEN);
}

/**
  * @brief  Check if Power Voltage Detector is enabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledPVD(void)
{
  return (READ_BIT(PMU->CTRL, PMU_CTRL_PVDEN) == (PMU_CTRL_PVDEN));
}

/**
  * @brief  Enable the WakeUp PINx functionality
  * @param  WakeUpPin This parameter can be one of the following values:
  *         @arg @ref DDL_PMU_WAKEUP_PIN1
  *         @arg @ref DDL_PMU_WAKEUP_PIN2 (*)
  *         @arg @ref DDL_PMU_WAKEUP_PIN3 (*)
  *
  *         (*) not available on all devices
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_EnableWakeUpPin(uint32_t WakeUpPin)
{
  SET_BIT(PMU->CSTS, WakeUpPin);
}

/**
  * @brief  Disable the WakeUp PINx functionality
  * @param  WakeUpPin This parameter can be one of the following values:
  *         @arg @ref DDL_PMU_WAKEUP_PIN1
  *         @arg @ref DDL_PMU_WAKEUP_PIN2 (*)
  *         @arg @ref DDL_PMU_WAKEUP_PIN3 (*)
  *
  *         (*) not available on all devices
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_DisableWakeUpPin(uint32_t WakeUpPin)
{
  CLEAR_BIT(PMU->CSTS, WakeUpPin);
}

/**
  * @brief  Check if the WakeUp PINx functionality is enabled
  * @param  WakeUpPin This parameter can be one of the following values:
  *         @arg @ref DDL_PMU_WAKEUP_PIN1
  *         @arg @ref DDL_PMU_WAKEUP_PIN2 (*)
  *         @arg @ref DDL_PMU_WAKEUP_PIN3 (*)
  *
  *         (*) not available on all devices
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsEnabledWakeUpPin(uint32_t WakeUpPin)
{
  return (READ_BIT(PMU->CSTS, WakeUpPin) == (WakeUpPin));
}


/**
  * @}
  */

/** @defgroup PMU_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get Wake-up Flag
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_WU(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_WUEFLG) == (PMU_CSTS_WUEFLG));
}

/**
  * @brief  Get Standby Flag
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_SB(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_SBFLG) == (PMU_CSTS_SBFLG));
}

/**
  * @brief  Get Backup Regulator ready Flag
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_BRR(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_BKPRFLG) == (PMU_CSTS_BKPRFLG));
}
/**
  * @brief  Indicate whether VDD voltage is below the selected PVD threshold
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_PVDO(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_PVDOFLG) == (PMU_CSTS_PVDOFLG));
}

/**
  * @brief  Indicate whether the Regulator is ready in the selected voltage range or if its output voltage is still changing to the required voltage level
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_VOS(void)
{
  return (READ_BIT(PMU->CSTS, DDL_PMU_CSTS_VOS) == (DDL_PMU_CSTS_VOS));
}
#if defined(PMU_CTRL_ODEN)
/**
  * @brief  Indicate whether the Over-Drive mode is ready or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_OD(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_ODRDY) == (PMU_CSTS_ODRDY));
}
#endif /* PMU_CTRL_ODEN */

#if defined(PMU_CTRL_ODSWEN)
/**
  * @brief  Indicate whether the Over-Drive mode switching is ready or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_ODSW(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_ODSWRDY) == (PMU_CSTS_ODSWRDY));
}
#endif /* PMU_CTRL_ODSWEN */

#if defined(PMU_CTRL_UDEN)
/**
  * @brief  Indicate whether the Under-Drive mode is ready or not
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_PMU_IsActiveFlag_UD(void)
{
  return (READ_BIT(PMU->CSTS, PMU_CSTS_UDRDY) == (PMU_CSTS_UDRDY));
}
#endif /* PMU_CTRL_UDEN */
/**
  * @brief  Clear Standby Flag
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_SB(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_SBFLGCLR);
}

/**
  * @brief  Clear Wake-up Flags
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_WU(void)
{
  SET_BIT(PMU->CTRL, PMU_CTRL_WUFLGCLR);
}
#if defined(PMU_CSTS_UDRDY)
/**
  * @brief  Clear Under-Drive ready Flag
  * @retval None
  */
__STATIC_INLINE void DDL_PMU_ClearFlag_UD(void)
{
  WRITE_REG(PMU->CSTS, PMU_CSTS_UDRDY);
}
#endif /* PMU_CSTS_UDRDY */

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup PMU_DDL_EF_Init De-initialization function
  * @{
  */
ErrorStatus DDL_PMU_DeInit(void);
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

#endif /* defined(PMU) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_PMU_H */
