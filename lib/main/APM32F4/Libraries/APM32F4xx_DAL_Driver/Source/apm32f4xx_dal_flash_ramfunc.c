/**
  *
  * @file    apm32f4xx_dal_flash_ramfunc.c
  * @brief   FLASH RAMFUNC module driver.
  *          This file provides a FLASH firmware functions which should be 
  *          executed from internal SRAM
  *            + Stop/Start the flash interface while System Run
  *            + Enable/Disable the flash sleep while System Run
  @verbatim
  ==============================================================================
                    ##### APIs executed from Internal RAM #####
  ==============================================================================
  [..]
    *** ARM Compiler ***
    --------------------
    [..] RAM functions are defined using the toolchain options. 
         Functions that are be executed in RAM should reside in a separate
         source module. Using the 'Options for File' dialog you can simply change
         the 'Code / Const' area of a module to a memory space in physical RAM.
         Available memory areas are declared in the 'Target' tab of the 
         Options for Target' dialog.

    *** ICCARM Compiler ***
    -----------------------
    [..] RAM functions are defined using a specific toolchain keyword "__ramfunc".

    *** GNU Compiler ***
    --------------------
    [..] RAM functions are defined using a specific toolchain attribute
         "__attribute__((section(".RamFunc")))".
  
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

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup FLASH_RAMFUNC FLASH RAMFUNC
  * @brief FLASH functions executed from RAM
  * @{
  */
#ifdef DAL_FLASH_MODULE_ENABLED
#if defined(APM32F411xx)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup FLASH_RAMFUNC_Exported_Functions FLASH RAMFUNC Exported Functions
  * @{
  */

/** @defgroup FLASH_RAMFUNC_Exported_Functions_Group1 Peripheral features functions executed from internal RAM 
  *  @brief Peripheral Extended features functions 
  *
@verbatim   

 ===============================================================================
                      ##### ramfunc functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions that should be executed from RAM 
    transfers.
    
@endverbatim
  * @{
  */

/**
  * @brief Stop the flash interface while System Run
  * @note  This mode is only available for APM32F41xxx/APM32F446xx devices. 
  * @note  This mode couldn't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval DAL status
  */
__RAM_FUNC DAL_StatusTypeDef DAL_FLASHEx_StopFlashInterfaceClk(void)
{
  /* Enable Power ctrl clock */
  __DAL_RCM_PMU_CLK_ENABLE();
  /* Stop the flash interface while System Run */  
  SET_BIT(PMU->CTRL, PMU_CTRL_FLASHEN);
   
  return DAL_OK;
}

/**
  * @brief Start the flash interface while System Run
  * @note  This mode is only available for APM32F411xx/APM32F446xx devices. 
  * @note  This mode couldn't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval DAL status
  */
__RAM_FUNC DAL_StatusTypeDef DAL_FLASHEx_StartFlashInterfaceClk(void)
{
  /* Enable Power ctrl clock */
  __DAL_RCM_PMU_CLK_ENABLE();
  /* Start the flash interface while System Run */
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_FLASHEN);

  return DAL_OK;
}

/**
  * @brief Enable the flash sleep while System Run
  * @note  This mode is only available for APM32F41xxx/APM32F446xx devices. 
  * @note  This mode could n't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval DAL status
  */
__RAM_FUNC DAL_StatusTypeDef DAL_FLASHEx_EnableFlashSleepMode(void)
{
  /* Enable Power ctrl clock */
  __DAL_RCM_PMU_CLK_ENABLE();
  /* Enable the flash sleep while System Run */
  SET_BIT(PMU->CTRL, PMU_CTRL_FSMODE);

  return DAL_OK;
}

/**
  * @brief Disable the flash sleep while System Run
  * @note  This mode is only available for APM32F41xxx/APM32F446xx devices. 
  * @note  This mode couldn't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @retval DAL status
  */
__RAM_FUNC DAL_StatusTypeDef DAL_FLASHEx_DisableFlashSleepMode(void)
{
  /* Enable Power ctrl clock */
  __DAL_RCM_PMU_CLK_ENABLE();
  /* Disable the flash sleep while System Run */
  CLEAR_BIT(PMU->CTRL, PMU_CTRL_FSMODE);
  
  return DAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* APM32F411xx */
#endif /* DAL_FLASH_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

