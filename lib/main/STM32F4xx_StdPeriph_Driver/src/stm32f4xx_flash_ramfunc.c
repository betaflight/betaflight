/**
  ******************************************************************************
  * @file    stm32f4xx_flash_ramfunc.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   FLASH RAMFUNC module driver.
  *          This file provides a FLASH firmware functions which should be 
  *          executed from internal SRAM
  *            + Stop/Start the flash interface while System Run
  *            + Enable/Disable the flash sleep while System Run
  *  
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
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_flash_ramfunc.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup FLASH RAMFUNC 
  * @brief FLASH RAMFUNC driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup FLASH_RAMFUNC_Private_Functions
  * @{
  */

/** @defgroup FLASH_RAMFUNC_Group1 Peripheral features functions executed from internal RAM 
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
  * @brief Start/Stop the flash interface while System Run
  * @note  This mode is only available for STM32F411xx devices. 
  * @note  This mode could n't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @param  NewState: new state of the Smart Card mode.
  *          This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
__RAM_FUNC FLASH_FlashInterfaceCmd(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Start the flash interface while System Run */
    CLEAR_BIT(PWR->CR, PWR_CR_FISSR);
  }
  else
  {
    /* Stop the flash interface while System Run */  
    SET_BIT(PWR->CR, PWR_CR_FISSR);
  }
}

/**
  * @brief Enable/Disable the flash sleep while System Run
  * @note  This mode is only available for STM32F411xx devices. 
  * @note  This mode could n't be set while executing with the flash itself. 
  *        It should be done with specific routine executed from RAM.     
  * @param  NewState: new state of the Smart Card mode.
  *          This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
__RAM_FUNC FLASH_FlashSleepModeCmd(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the flash sleep while System Run */
    SET_BIT(PWR->CR, PWR_CR_FMSSR);
  }
  else
  {
    /* Disable the flash sleep while System Run */
    CLEAR_BIT(PWR->CR, PWR_CR_FMSSR);
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
