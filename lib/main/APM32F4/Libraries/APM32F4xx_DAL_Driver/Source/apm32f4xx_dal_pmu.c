/**
  *
  * @file    apm32f4xx_dal_pmu.c
  * @author  MCD Application Team
  * @brief   PMU DAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Power Controller (PMU) peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions 
  *         
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

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup PMU PMU
  * @brief PMU DAL module driver
  * @{
  */

#ifdef DAL_PMU_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup PMU_Private_Constants
  * @{
  */
  
/** @defgroup PMU_PVD_Mode_Mask PMU PVD Mode Mask
  * @{
  */     
#define PVD_MODE_IT               0x00010000U
#define PVD_MODE_EVT              0x00020000U
#define PVD_RISING_EDGE           0x00000001U
#define PVD_FALLING_EDGE          0x00000002U
/**
  * @}
  */

/**
  * @}
  */    
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup PMU_Exported_Functions PMU Exported Functions
  * @{
  */

/** @defgroup PMU_Exported_Functions_Group1 Initialization and de-initialization functions 
  *  @brief    Initialization and de-initialization functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
      After reset, the backup domain (RTC registers, RTC backup data 
      registers and backup SRAM) is protected against possible unwanted 
      write accesses. 
      To enable access to the RTC Domain and RTC registers, proceed as follows:
        (+) Enable the Power Controller (PMU) APB1 interface clock using the
            __DAL_RCM_PMU_CLK_ENABLE() macro.
        (+) Enable access to RTC domain using the DAL_PMU_EnableBkUpAccess() function.
 
@endverbatim
  * @{
  */

/**
  * @brief Deinitializes the DAL PMU peripheral registers to their default reset values.
  * @retval None
  */
void DAL_PMU_DeInit(void)
{
  __DAL_RCM_PMU_FORCE_RESET();
  __DAL_RCM_PMU_RELEASE_RESET();
}

/**
  * @brief Enables access to the backup domain (RTC registers, RTC 
  *         backup data registers and backup SRAM).
  * @note If the HSE divided by 2, 3, ..31 is used as the RTC clock, the 
  *         Backup Domain Access should be kept enabled.
  * @note The following sequence is required to bypass the delay between
  *         DBP bit programming and the effective enabling  of the backup domain.
  *         Please check the Errata Sheet for more details under "Possible delay
  *         in backup domain protection disabling/enabling after programming the
  *         DBP bit" section.
  * @retval None
  */
void DAL_PMU_EnableBkUpAccess(void)
{
  __IO uint32_t dummyread;
  *(__IO uint32_t *) CTRL_BPWEN_BB = (uint32_t)ENABLE;
  dummyread = PMU->CTRL;
  UNUSED(dummyread);
}

/**
  * @brief Disables access to the backup domain (RTC registers, RTC 
  *         backup data registers and backup SRAM).
  * @note If the HSE divided by 2, 3, ..31 is used as the RTC clock, the 
  *         Backup Domain Access should be kept enabled.
  * @note The following sequence is required to bypass the delay between
  *         DBP bit programming and the effective disabling  of the backup domain.
  *         Please check the Errata Sheet for more details under "Possible delay
  *         in backup domain protection disabling/enabling after programming the
  *         DBP bit" section.
  * @retval None
  */
void DAL_PMU_DisableBkUpAccess(void)
{
  __IO uint32_t dummyread;
  *(__IO uint32_t *) CTRL_BPWEN_BB = (uint32_t)DISABLE;
  dummyread = PMU->CTRL;
  UNUSED(dummyread);
}

/**
  * @}
  */

/** @defgroup PMU_Exported_Functions_Group2 Peripheral Control functions 
  *  @brief Low Power modes configuration functions 
  *
@verbatim

 ===============================================================================
                 ##### Peripheral Control functions #####
 ===============================================================================
     
    *** PVD configuration ***
    =========================
    [..]
      (+) The PVD is used to monitor the VDD power supply by comparing it to a 
          threshold selected by the PVD Level (PLS[2:0] bits in the PMU_CTRL).
      (+) A PVDO flag is available to indicate if VDD/VDDA is higher or lower 
          than the PVD threshold. This event is internally connected to the EINT 
          line16 and can generate an interrupt if enabled. This is done through
          __DAL_PMU_PVD_EINT_ENABLE_IT() macro.
      (+) The PVD is stopped in Standby mode.

    *** Wake-up pin configuration ***
    ================================
    [..]
      (+) Wake-up pin is used to wake up the system from Standby mode. This pin is 
          forced in input pull-down configuration and is active on rising edges.
      (+) There is one Wake-up pin: Wake-up Pin 1 on PA.00.

    *** Low Power modes configuration ***
    =====================================
    [..]
      The devices feature 3 low-power modes:
      (+) Sleep mode: Cortex-M4 core stopped, peripherals kept running.
      (+) Stop mode: all clocks are stopped, regulator running, regulator 
          in low power mode
      (+) Standby mode: 1.2V domain powered off.
   
   *** Sleep mode ***
   ==================
    [..]
      (+) Entry:
        The Sleep mode is entered by using the DAL_PMU_EnterSLEEPMode(PMU_MAINREGULATOR_ON, PMU_SLEEPENTRY_WFI)
              functions with
          (++) PMU_SLEEPENTRY_WFI: enter SLEEP mode with WFI instruction
          (++) PMU_SLEEPENTRY_WFE: enter SLEEP mode with WFE instruction
      
      -@@- The Regulator parameter is not used for the APM32F4 family 
              and is kept as parameter just to maintain compatibility with the 
              lower power families (APM32L).
      (+) Exit:
        Any peripheral interrupt acknowledged by the nested vectored interrupt 
              controller (NVIC) can wake up the device from Sleep mode.

   *** Stop mode ***
   =================
    [..]
      In Stop mode, all clocks in the 1.2V domain are stopped, the PLL, the HSI,
      and the HSE RC oscillators are disabled. Internal SRAM and register contents 
      are preserved.
      The voltage regulator can be configured either in normal or low-power mode.
      To minimize the consumption In Stop mode, FLASH can be powered off before 
      entering the Stop mode using the DAL_PMUEx_EnableFlashPowerDown() function.
      It can be switched on again by software after exiting the Stop mode using
      the DAL_PMUEx_DisableFlashPowerDown() function. 

      (+) Entry:
         The Stop mode is entered using the DAL_PMU_EnterSTOPMode(PMU_MAINREGULATOR_ON) 
             function with:
          (++) Main regulator ON.
          (++) Low Power regulator ON.
      (+) Exit:
        Any EINT Line (Internal or External) configured in Interrupt/Event mode.

   *** Standby mode ***
   ====================
    [..]
    (+)
      The Standby mode allows to achieve the lowest power consumption. It is based 
      on the Cortex-M4 deep sleep mode, with the voltage regulator disabled. 
      The 1.2V domain is consequently powered off. The PLL, the HSI oscillator and 
      the HSE oscillator are also switched off. SRAM and register contents are lost 
      except for the RTC registers, RTC backup registers, backup SRAM and Standby 
      circuitry.
   
      The voltage regulator is OFF.
      
      (++) Entry:
        (+++) The Standby mode is entered using the DAL_PMU_EnterSTANDBYMode() function.
      (++) Exit:
        (+++) WKUP pin rising edge, RTC alarm (Alarm A and Alarm B), RTC wake-up,
             tamper event, time-stamp event, external reset in NRST pin, IWDT reset.

   *** Auto-wake-up (AWU) from low-power mode ***
   =============================================
    [..]
    
     (+) The MCU can be woken up from low-power mode by an RTC Alarm event, an RTC 
      Wake-up event, a tamper event or a time-stamp event, without depending on 
      an external interrupt (Auto-wake-up mode).

      (+) RTC auto-wake-up (AWU) from the Stop and Standby modes
       
        (++) To wake up from the Stop mode with an RTC alarm event, it is necessary to 
              configure the RTC to generate the RTC alarm using the DAL_RTC_SetAlarm_IT() function.

        (++) To wake up from the Stop mode with an RTC Tamper or time stamp event, it 
             is necessary to configure the RTC to detect the tamper or time stamp event using the
                DAL_RTCEx_SetTimeStamp_IT() or DAL_RTCEx_SetTamper_IT() functions.
                  
        (++) To wake up from the Stop mode with an RTC Wake-up event, it is necessary to
              configure the RTC to generate the RTC Wake-up event using the DAL_RTCEx_SetWakeUpTimer_IT() function.

@endverbatim
  * @{
  */

/**
  * @brief Configures the voltage threshold detected by the Power Voltage Detector(PVD).
  * @param sConfigPVD pointer to an PMU_PVDTypeDef structure that contains the configuration
  *        information for the PVD.
  * @note Refer to the electrical characteristics of your device datasheet for
  *         more details about the voltage threshold corresponding to each 
  *         detection level.
  * @retval None
  */
void DAL_PMU_ConfigPVD(PMU_PVDTypeDef *sConfigPVD)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_PMU_PVD_LEVEL(sConfigPVD->PVDLevel));
  ASSERT_PARAM(IS_PMU_PVD_MODE(sConfigPVD->Mode));
  
  /* Set PLS[7:5] bits according to PVDLevel value */
  MODIFY_REG(PMU->CTRL, PMU_CTRL_PLSEL, sConfigPVD->PVDLevel);
  
  /* Clear any previous config. Keep it clear if no event or IT mode is selected */
  __DAL_PMU_PVD_EINT_DISABLE_EVENT();
  __DAL_PMU_PVD_EINT_DISABLE_IT();
  __DAL_PMU_PVD_EINT_DISABLE_RISING_EDGE();
  __DAL_PMU_PVD_EINT_DISABLE_FALLING_EDGE(); 

  /* Configure interrupt mode */
  if((sConfigPVD->Mode & PVD_MODE_IT) == PVD_MODE_IT)
  {
    __DAL_PMU_PVD_EINT_ENABLE_IT();
  }
  
  /* Configure event mode */
  if((sConfigPVD->Mode & PVD_MODE_EVT) == PVD_MODE_EVT)
  {
    __DAL_PMU_PVD_EINT_ENABLE_EVENT();
  }
  
  /* Configure the edge */
  if((sConfigPVD->Mode & PVD_RISING_EDGE) == PVD_RISING_EDGE)
  {
    __DAL_PMU_PVD_EINT_ENABLE_RISING_EDGE();
  }
  
  if((sConfigPVD->Mode & PVD_FALLING_EDGE) == PVD_FALLING_EDGE)
  {
    __DAL_PMU_PVD_EINT_ENABLE_FALLING_EDGE();
  }
}

/**
  * @brief Enables the Power Voltage Detector(PVD).
  * @retval None
  */
void DAL_PMU_EnablePVD(void)
{
  *(__IO uint32_t *) CTRL_PVDEN_BB = (uint32_t)ENABLE;
}

/**
  * @brief Disables the Power Voltage Detector(PVD).
  * @retval None
  */
void DAL_PMU_DisablePVD(void)
{
  *(__IO uint32_t *) CTRL_PVDEN_BB = (uint32_t)DISABLE;
}

/**
  * @brief Enables the Wake-up PINx functionality.
  * @param WakeUpPinx Specifies the Power Wake-Up pin to enable.
  *         This parameter can be one of the following values:
  *           @arg PMU_WAKEUP_PIN1
  * @retval None
  */
void DAL_PMU_EnableWakeUpPin(uint32_t WakeUpPinx)
{
  /* Check the parameter */
  ASSERT_PARAM(IS_PMU_WAKEUP_PIN(WakeUpPinx));

  /* Enable the wake up pin */
  SET_BIT(PMU->CSTS, WakeUpPinx);
}

/**
  * @brief Disables the Wake-up PINx functionality.
  * @param WakeUpPinx Specifies the Power Wake-Up pin to disable.
  *         This parameter can be one of the following values:
  *           @arg PMU_WAKEUP_PIN1
  * @retval None
  */
void DAL_PMU_DisableWakeUpPin(uint32_t WakeUpPinx)
{
  /* Check the parameter */
  ASSERT_PARAM(IS_PMU_WAKEUP_PIN(WakeUpPinx));  

  /* Disable the wake up pin */
  CLEAR_BIT(PMU->CSTS, WakeUpPinx);
}
  
/**
  * @brief Enters Sleep mode.
  *   
  * @note In Sleep mode, all I/O pins keep the same state as in Run mode.
  * 
  * @note In Sleep mode, the systick is stopped to avoid exit from this mode with
  *       systick interrupt when used as time base for Timeout 
  *                
  * @param Regulator Specifies the regulator state in SLEEP mode.
  *            This parameter can be one of the following values:
  *            @arg PMU_MAINREGULATOR_ON: SLEEP mode with regulator ON
  *            @arg PMU_LOWPOWERREGULATOR_ON: SLEEP mode with low power regulator ON
  * @note This parameter is not used for the APM32F4 family and is kept as parameter
  *       just to maintain compatibility with the lower power families.
  * @param SLEEPEntry Specifies if SLEEP mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg PMU_SLEEPENTRY_WFI: enter SLEEP mode with WFI instruction
  *            @arg PMU_SLEEPENTRY_WFE: enter SLEEP mode with WFE instruction
  * @retval None
  */
void DAL_PMU_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_PMU_REGULATOR(Regulator));
  ASSERT_PARAM(IS_PMU_SLEEP_ENTRY(SLEEPEntry));

  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  /* Select SLEEP mode entry -------------------------------------------------*/
  if(SLEEPEntry == PMU_SLEEPENTRY_WFI)
  {   
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __SEV();
    __WFE();
    __WFE();
  }
}

/**
  * @brief Enters Stop mode. 
  * @note In Stop mode, all I/O pins keep the same state as in Run mode.
  * @note When exiting Stop mode by issuing an interrupt or a wake-up event, 
  *         the HSI RC oscillator is selected as system clock.
  * @note When the voltage regulator operates in low power mode, an additional 
  *         startup delay is incurred when waking up from Stop mode. 
  *         By keeping the internal regulator ON during Stop mode, the consumption 
  *         is higher although the startup time is reduced.    
  * @param Regulator Specifies the regulator state in Stop mode.
  *          This parameter can be one of the following values:
  *            @arg PMU_MAINREGULATOR_ON: Stop mode with regulator ON
  *            @arg PMU_LOWPOWERREGULATOR_ON: Stop mode with low power regulator ON
  * @param STOPEntry Specifies if Stop mode in entered with WFI or WFE instruction.
  *          This parameter can be one of the following values:
  *            @arg PMU_STOPENTRY_WFI: Enter Stop mode with WFI instruction
  *            @arg PMU_STOPENTRY_WFE: Enter Stop mode with WFE instruction
  * @retval None
  */
void DAL_PMU_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_PMU_REGULATOR(Regulator));
  ASSERT_PARAM(IS_PMU_STOP_ENTRY(STOPEntry));
  
  /* Select the regulator state in Stop mode: Set PDDS and LPDS bits according to PMU_Regulator value */
  MODIFY_REG(PMU->CTRL, (PMU_CTRL_PDDSCFG | PMU_CTRL_LPDSCFG), Regulator);
  
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
  
  /* Select Stop mode entry --------------------------------------------------*/
  if(STOPEntry == PMU_STOPENTRY_WFI)
  {   
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __SEV();
    __WFE();
    __WFE();
  }
  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));  
}

/**
  * @brief Enters Standby mode.
  * @note In Standby mode, all I/O pins are high impedance except for:
  *          - Reset pad (still available) 
  *          - RTC_AF1 pin (PC13) if configured for tamper, time-stamp, RTC 
  *            Alarm out, or RTC clock calibration out.
  *          - RTC_AF2 pin (PI8) if configured for tamper or time-stamp.  
  *          - WKUP pin 1 (PA0) if enabled.       
  * @retval None
  */
void DAL_PMU_EnterSTANDBYMode(void)
{
  /* Select Standby mode */
  SET_BIT(PMU->CTRL, PMU_CTRL_PDDSCFG);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
  
  /* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
  __force_stores();
#endif
  /* Request Wait For Interrupt */
  __WFI();
}

/**
  * @brief This function handles the PMU PVD interrupt request.
  * @note This API should be called under the PVD_IRQHandler().
  * @retval None
  */
void DAL_PMU_PVD_IRQHandler(void)
{
  /* Check PMU Exti flag */
  if(__DAL_PMU_PVD_EINT_GET_FLAG() != RESET)
  {
    /* PMU PVD interrupt user callback */
    DAL_PMU_PVDCallback();
    
    /* Clear PMU Exti pending bit */
    __DAL_PMU_PVD_EINT_CLEAR_FLAG();
  }
}

/**
  * @brief  PMU PVD interrupt callback
  * @retval None
  */
__weak void DAL_PMU_PVDCallback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_PMU_PVDCallback could be implemented in the user file
   */ 
}

/**
  * @brief Indicates Sleep-On-Exit when returning from Handler mode to Thread mode. 
  * @note Set SLEEPONEXIT bit of SCR register. When this bit is set, the processor 
  *       re-enters SLEEP mode when an interruption handling is over.
  *       Setting this bit is useful when the processor is expected to run only on
  *       interruptions handling.         
  * @retval None
  */
void DAL_PMU_EnableSleepOnExit(void)
{
  /* Set SLEEPONEXIT bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));
}

/**
  * @brief Disables Sleep-On-Exit feature when returning from Handler mode to Thread mode. 
  * @note Clears SLEEPONEXIT bit of SCR register. When this bit is set, the processor 
  *       re-enters SLEEP mode when an interruption handling is over.          
  * @retval None
  */
void DAL_PMU_DisableSleepOnExit(void)
{
  /* Clear SLEEPONEXIT bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));
}

/**
  * @brief Enables CORTEX M4 SEVONPEND bit. 
  * @note Sets SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.
  * @retval None
  */
void DAL_PMU_EnableSEVOnPend(void)
{
  /* Set SEVONPEND bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @brief Disables CORTEX M4 SEVONPEND bit. 
  * @note Clears SEVONPEND bit of SCR register. When this bit is set, this causes 
  *       WFE to wake up when an interrupt moves from inactive to pended.         
  * @retval None
  */
void DAL_PMU_DisableSEVOnPend(void)
{
  /* Clear SEVONPEND bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk));
}

/**
  * @}
  */
  
/**
  * @}
  */

#endif /* DAL_PMU_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
