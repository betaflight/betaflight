/**
  *
  * @file    apm32f4xx_dal.c
  * @brief   DAL module driver.
  *          This is the common part of the DAL initialization
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
    The common DAL driver contains a set of generic and common APIs that can be
    used by the PPP peripheral drivers and the user to start using the DAL. 
    [..]
    The DAL contains two APIs' categories: 
         (+) Common DAL APIs
         (+) Services DAL APIs

  @endverbatim
  *
  */ 

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup DAL DAL
  * @brief DAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup DAL_Private_Constants
  * @{
  */
/**
  * @brief APM32F4xx DAL Driver version number V1.1.2
  */
#define __APM32F4xx_DAL_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __APM32F4xx_DAL_VERSION_SUB1   (0x01U) /*!< [23:16] sub1 version */
#define __APM32F4xx_DAL_VERSION_SUB2   (0x02U) /*!< [15:8]  sub2 version */
#define __APM32F4xx_DAL_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */ 
#define __APM32F4xx_DAL_VERSION         ((__APM32F4xx_DAL_VERSION_MAIN << 24U)\
                                        |(__APM32F4xx_DAL_VERSION_SUB1 << 16U)\
                                        |(__APM32F4xx_DAL_VERSION_SUB2 << 8U )\
                                        |(__APM32F4xx_DAL_VERSION_RC))
                                        
#define IDCODE_DEVID_MASK    0x00000FFFU

/* ------------ RCM registers bit address in the alias region ----------- */
#define SYSCFG_OFFSET             (SYSCFG_BASE - PERIPH_BASE)
/* ---  MEMRMP Register ---*/ 
/* Alias word address of UFB_MODE bit */ 
#define MEMRMP_OFFSET             SYSCFG_OFFSET 
#define UFB_MODE_BIT_NUMBER       SYSCFG_MMSEL_UFB_MODE_Pos
#define UFB_MODE_BB               (uint32_t)(PERIPH_BB_BASE + (MEMRMP_OFFSET * 32U) + (UFB_MODE_BIT_NUMBER * 4U)) 

/* ---  CMPCR Register ---*/ 
/* Alias word address of CMP_PD bit */ 
#define CMPCR_OFFSET              (SYSCFG_OFFSET + 0x20U) 
#define CMP_PD_BIT_NUMBER         SYSCFG_CCCTRL_CCPD_Pos
#define CMPCR_CMP_PD_BB           (uint32_t)(PERIPH_BB_BASE + (CMPCR_OFFSET * 32U) + (CMP_PD_BIT_NUMBER * 4U))

/* ---  MCHDLYCR Register ---*/ 
/* Alias word address of BSCKSEL bit */ 
#define MCHDLYCR_OFFSET            (SYSCFG_OFFSET + 0x30U) 
#define BSCKSEL_BIT_NUMBER         SYSCFG_MCHDLYCR_BSCKSEL_Pos
#define MCHDLYCR_BSCKSEL_BB        (uint32_t)(PERIPH_BB_BASE + (MCHDLYCR_OFFSET * 32U) + (BSCKSEL_BIT_NUMBER * 4U))
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup DAL_Private_Variables
  * @{
  */
__IO uint32_t uwTick;
uint32_t uwTickPrio   = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */
DAL_TickFreqTypeDef uwTickFreq = DAL_TICK_FREQ_DEFAULT;  /* 1KHz */
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup DAL_Exported_Functions DAL Exported Functions
  * @{
  */

/** @defgroup DAL_Exported_Functions_Group1 Initialization and de-initialization Functions 
 *  @brief    Initialization and de-initialization functions
 *
@verbatim    
 ===============================================================================
              ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initializes the Flash interface the NVIC allocation and initial clock 
          configuration. It initializes the systick also when timeout is needed 
          and the backup domain when enabled.
      (+) De-Initializes common part of the DAL.
      (+) Configure the time base source to have 1ms time base with a dedicated 
          Tick interrupt priority. 
        (++) SysTick timer is used by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
        (++) Time base configuration function (DAL_InitTick ()) is called automatically 
             at the beginning of the program after reset by DAL_Init() or at any time 
             when clock is configured, by DAL_RCM_ClockConfig(). 
        (++) Source of time base is configured  to generate interrupts at regular 
             time intervals. Care must be taken if DAL_Delay() is called from a 
             peripheral ISR process, the Tick interrupt line must have higher priority 
            (numerically lower) than the peripheral interrupt. Otherwise the caller 
            ISR process will be blocked. 
       (++) functions affecting time base configurations are declared as __weak  
             to make  override possible  in case of other  implementations in user file.
@endverbatim
  * @{
  */

/**
  * @brief  This function is used to initialize the DAL Library; it must be the first 
  *         instruction to be executed in the main program (before to call any other
  *         DAL function), it performs the following:
  *           Configure the Flash prefetch, instruction and Data caches.
  *           Configures the SysTick to generate an interrupt each 1 millisecond,
  *           which is clocked by the HSI (at this stage, the clock is not yet
  *           configured and thus the system is running from the internal HSI at 16 MHz).
  *           Set NVIC Group Priority to 4.
  *           Calls the DAL_MspInit() callback function defined in user file 
  *           "apm32f4xx_dal_msp.c" to do the global low level hardware initialization 
  *            
  * @note   SysTick is used as time base for the DAL_Delay() function, the application
  *         need to ensure that the SysTick time base is always set to 1 millisecond
  *         to have correct DAL operation.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_Init(void)
{
  /* Configure Flash prefetch, Instruction cache, Data cache */ 
#if (INSTRUCTION_CACHE_ENABLE != 0U)
  __DAL_FLASH_INSTRUCTION_CACHE_ENABLE();
#endif /* INSTRUCTION_CACHE_ENABLE */

#if (DATA_CACHE_ENABLE != 0U)
  __DAL_FLASH_DATA_CACHE_ENABLE();
#endif /* DATA_CACHE_ENABLE */

#if (PREFETCH_ENABLE != 0U)
  __DAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

  /* Set Interrupt Group Priority */
  DAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  DAL_InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
  DAL_MspInit();

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  This function de-Initializes common part of the DAL and stops the systick.
  *         This function is optional.   
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DeInit(void)
{
  /* Reset of all peripherals */
  __DAL_RCM_APB1_FORCE_RESET();
  __DAL_RCM_APB1_RELEASE_RESET();

  __DAL_RCM_APB2_FORCE_RESET();
  __DAL_RCM_APB2_RELEASE_RESET();

  __DAL_RCM_AHB1_FORCE_RESET();
  __DAL_RCM_AHB1_RELEASE_RESET();

  __DAL_RCM_AHB2_FORCE_RESET();
  __DAL_RCM_AHB2_RELEASE_RESET();

  __DAL_RCM_AHB3_FORCE_RESET();
  __DAL_RCM_AHB3_RELEASE_RESET();

  /* De-Init the low level hardware */
  DAL_MspDeInit();
    
  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Initialize the MSP.
  * @retval None
  */
__weak void DAL_MspInit(void)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes the MSP.
  * @retval None
  */
__weak void DAL_MspDeInit(void)
{
  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_MspDeInit could be implemented in the user file
   */ 
}

/**
  * @brief This function configures the source of the time base.
  *        The time source is configured  to have 1ms time base with a dedicated 
  *        Tick interrupt priority.
  * @note This function is called  automatically at the beginning of program after
  *       reset by DAL_Init() or at any time when clock is reconfigured  by DAL_RCM_ClockConfig().
  * @note In the default implementation, SysTick timer is the source of time base. 
  *       It is used to generate interrupts at regular time intervals. 
  *       Care must be taken if DAL_Delay() is called from a peripheral ISR process, 
  *       The SysTick interrupt must have higher priority (numerically lower)
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       The function is declared as __weak  to be overwritten  in case of other
  *       implementation  in user file.
  * @param TickPriority Tick interrupt priority.
  * @retval DAL status
  */
__weak DAL_StatusTypeDef DAL_InitTick(uint32_t TickPriority)
{
  /* Configure the SysTick to have interrupt in 1ms time basis*/
  if (DAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return DAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    DAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return DAL_ERROR;
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup DAL_Exported_Functions_Group2 DAL Control functions 
 *  @brief    DAL Control functions
 *
@verbatim
 ===============================================================================
                      ##### DAL Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Provide a tick value in millisecond
      (+) Provide a blocking delay in millisecond
      (+) Suspend the time base source interrupt
      (+) Resume the time base source interrupt
      (+) Get the DAL API driver version
      (+) Get the device identifier
      (+) Get the device revision identifier
      (+) Enable/Disable Debug module during SLEEP mode
      (+) Enable/Disable Debug module during STOP mode
      (+) Enable/Disable Debug module during STANDBY mode

@endverbatim
  * @{
  */

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in SysTick ISR.
 * @note This function is declared as __weak to be overwritten in case of other 
  *      implementations in user file.
  * @retval None
  */
__weak void DAL_IncTick(void)
{
  uwTick += uwTickFreq;
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other 
  *       implementations in user file.
  * @retval tick value
  */
__weak uint32_t DAL_GetTick(void)
{
  return uwTick;
}

/**
  * @brief This function returns a tick priority.
  * @retval tick priority
  */
uint32_t DAL_GetTickPrio(void)
{
  return uwTickPrio;
}

/**
  * @brief Set new tick Freq.
  * @retval Status
  */
DAL_StatusTypeDef DAL_SetTickFreq(DAL_TickFreqTypeDef Freq)
{
  DAL_StatusTypeDef status  = DAL_OK;
  DAL_TickFreqTypeDef prevTickFreq;

  ASSERT_PARAM(IS_TICKFREQ(Freq));

  if (uwTickFreq != Freq)
  {
    /* Back up uwTickFreq frequency */
    prevTickFreq = uwTickFreq;

    /* Update uwTickFreq global variable used by DAL_InitTick() */
    uwTickFreq = Freq;

    /* Apply the new tick Freq  */
    status = DAL_InitTick(uwTickPrio);

    if (status != DAL_OK)
    {
      /* Restore previous tick frequency */
      uwTickFreq = prevTickFreq;
    }
  }

  return status;
}

/**
  * @brief Return tick frequency.
  * @retval tick period in Hz
  */
DAL_TickFreqTypeDef DAL_GetTickFreq(void)
{
  return uwTickFreq;
}

/**
  * @brief This function provides minimum delay (in milliseconds) based 
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
__weak void DAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = DAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < DAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while((DAL_GetTick() - tickstart) < wait)
  {
  }
}

/**
  * @brief Suspend Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once DAL_SuspendTick()
  *       is called, the SysTick interrupt will be disabled and so Tick increment 
  *       is suspended.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
__weak void DAL_SuspendTick(void)
{
  /* Disable SysTick Interrupt */
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

/**
  * @brief Resume Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once DAL_ResumeTick()
  *       is called, the SysTick interrupt will be enabled and so Tick increment 
  *       is resumed.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
__weak void DAL_ResumeTick(void)
{
  /* Enable SysTick Interrupt */
  SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
}

/**
  * @brief  Returns the DAL revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t DAL_GetHalVersion(void)
{
  return __APM32F4xx_DAL_VERSION;
}

/**
  * @brief  Returns the device revision identifier.
  * @retval Device revision identifier
  */
uint32_t DAL_GetREVID(void)
{
  return((DBGMCU->IDCODE) >> 16U);
}

/**
  * @brief  Returns the device identifier.
  * @retval Device identifier
  */
uint32_t DAL_GetDEVID(void)
{
  return((DBGMCU->IDCODE) & IDCODE_DEVID_MASK);
}

/**
  * @brief  Enable the Debug Module during SLEEP mode
  * @retval None
  */
void DAL_DBGMCU_EnableDBGSleepMode(void)
{
  SET_BIT(DBGMCU->CFG, DBGMCU_CFG_SLEEP_CLK_STS);
}

/**
  * @brief  Disable the Debug Module during SLEEP mode
  * @retval None
  */
void DAL_DBGMCU_DisableDBGSleepMode(void)
{
  CLEAR_BIT(DBGMCU->CFG, DBGMCU_CFG_SLEEP_CLK_STS);
}

/**
  * @brief  Enable the Debug Module during STOP mode
  * @retval None
  */
void DAL_DBGMCU_EnableDBGStopMode(void)
{
  SET_BIT(DBGMCU->CFG, DBGMCU_CFG_STOP_CLK_STS);
}

/**
  * @brief  Disable the Debug Module during STOP mode
  * @retval None
  */
void DAL_DBGMCU_DisableDBGStopMode(void)
{
  CLEAR_BIT(DBGMCU->CFG, DBGMCU_CFG_STOP_CLK_STS);
}

/**
  * @brief  Enable the Debug Module during STANDBY mode
  * @retval None
  */
void DAL_DBGMCU_EnableDBGStandbyMode(void)
{
  SET_BIT(DBGMCU->CFG, DBGMCU_CFG_STANDBY_CLK_STS);
}

/**
  * @brief  Disable the Debug Module during STANDBY mode
  * @retval None
  */
void DAL_DBGMCU_DisableDBGStandbyMode(void)
{
  CLEAR_BIT(DBGMCU->CFG, DBGMCU_CFG_STANDBY_CLK_STS);
}

/**
  * @brief  Enables the I/O Compensation Cell.
  * @note   The I/O compensation cell can be used only when the device supply
  *         voltage ranges from 2.4 to 3.6 V.  
  * @retval None
  */
void DAL_EnableCompensationCell(void)
{
  *(__IO uint32_t *)CMPCR_CMP_PD_BB = (uint32_t)ENABLE;
}

/**
  * @brief  Power-down the I/O Compensation Cell.
  * @note   The I/O compensation cell can be used only when the device supply
  *         voltage ranges from 2.4 to 3.6 V.  
  * @retval None
  */
void DAL_DisableCompensationCell(void)
{
  *(__IO uint32_t *)CMPCR_CMP_PD_BB = (uint32_t)DISABLE;
}

/**
  * @brief  Returns first word of the unique device identifier (UID based on 96 bits)
  * @retval Device identifier
  */
uint32_t DAL_GetUIDw0(void)
{
  return (READ_REG(*((uint32_t *)UID_BASE)));
}

/**
  * @brief  Returns second word of the unique device identifier (UID based on 96 bits)
  * @retval Device identifier
  */
uint32_t DAL_GetUIDw1(void)
{
  return (READ_REG(*((uint32_t *)(UID_BASE + 4U))));
}

/**
  * @brief  Returns third word of the unique device identifier (UID based on 96 bits)
  * @retval Device identifier
  */
uint32_t DAL_GetUIDw2(void)
{
  return (READ_REG(*((uint32_t *)(UID_BASE + 8U))));
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

/**
  * @}
  */


