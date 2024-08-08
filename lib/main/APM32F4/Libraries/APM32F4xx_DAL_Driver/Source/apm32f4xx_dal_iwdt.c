/**
  *
  * @file    apm32f4xx_dal_iwdt.c
  * @brief   IWDT DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Independent Watchdog (IWDT) peripheral:
  *           + Initialization and Start functions
  *           + IO operation functions
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
  * Copyright (c) 2016 STMicroelectronics.
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
                    ##### IWDT Generic features #####
  ==============================================================================
  [..]
    (+) The IWDT can be started by either software or hardware (configurable
        through option byte).

    (+) The IWDT is clocked by the Low-Speed Internal clock (LSI) and thus stays
        active even if the main clock fails.

    (+) Once the IWDT is started, the LSI is forced ON and both cannot be
        disabled. The counter starts counting down from the reset value (0xFFF).
        When it reaches the end of count value (0x000) a reset signal is
        generated (IWDT reset).

    (+) Whenever the key value 0x0000 AAAA is written in the IWDT_KEY register,
        the IWDT_CNTRLD value is reloaded into the counter and the watchdog reset
        is prevented.

    (+) The IWDT is implemented in the VDD voltage domain that is still functional
        in STOP and STANDBY mode (IWDT reset can wake up the CPU from STANDBY).
        IWDTRST flag in RCM_CSTS register can be used to inform when an IWDT
        reset occurs.

    (+) Debug mode: When the microcontroller enters debug mode (core halted),
        the IWDT counter either continues to work normally or stops, depending
        on DBG_IWDT_STOP configuration bit in DBG module, accessible through
        __DAL_DBGMCU_FREEZE_IWDT() and __DAL_DBGMCU_UNFREEZE_IWDT() macros.

    [..] Min-max timeout value @32KHz (LSI): ~125us / ~32.7s
         The IWDT timeout may vary due to LSI clock frequency dispersion.
         APM32F4xx devices provide the capability to measure the LSI clock
         frequency (LSI clock is internally connected to TMR5 CH4 input capture).
         The measured value can be used to have an IWDT timeout with an
         acceptable accuracy.

    [..] Default timeout value (necessary for IWDT_STS status register update):
         Constant LSI_VALUE is defined based on the nominal LSI clock frequency.
         This frequency being subject to variations as mentioned above, the
         default timeout value (defined through constant DAL_IWDT_DEFAULT_TIMEOUT
         below) may become too short or too long.
         In such cases, this default timeout value can be tuned by redefining
         the constant LSI_VALUE at user-application level (based, for instance,
         on the measured LSI clock frequency as explained above).

                     ##### How to use this driver #####
  ==============================================================================
  [..]
    (#) Use IWDT using DAL_IWDT_Init() function to :
      (++) Enable instance by writing Start keyword in IWDT_KEY register. LSI
           clock is forced ON and IWDT counter starts counting down.
      (++) Enable write access to configuration registers:
          IWDT_PSC and IWDT_CNTRLD.
      (++) Configure the IWDT prescaler and counter reload value. This reload
           value will be loaded in the IWDT counter each time the watchdog is
           reloaded, then the IWDT will start counting down from this value.
      (++) Wait for status flags to be reset.

    (#) Then the application program must refresh the IWDT counter at regular
        intervals during normal operation to prevent an MCU reset, using
        DAL_IWDT_Refresh() function.

     *** IWDT DAL driver macros list ***
     ====================================
     [..]
       Below the list of most used macros in IWDT DAL driver:
      (+) __DAL_IWDT_START: Enable the IWDT peripheral
      (+) __DAL_IWDT_RELOAD_COUNTER: Reloads IWDT counter with value defined in
          the reload register

  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#ifdef DAL_IWDT_MODULE_ENABLED
/** @addtogroup IWDT
  * @brief IWDT DAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup IWDT_Private_Defines IWDT Private Defines
  * @{
  */
/* Status register needs up to 5 LSI clock periods divided by the clock
   prescaler to be updated. The number of LSI clock periods is upper-rounded to
   6 for the timeout value calculation.
   The timeout value is calculated using the highest prescaler (256) and
   the LSI_VALUE constant. The value of this constant can be changed by the user
   to take into account possible LSI clock period variations.
   The timeout value is multiplied by 1000 to be converted in milliseconds.
   LSI startup time is also considered here by adding LSI_STARTUP_TIME
   converted in milliseconds. */
#define DAL_IWDT_DEFAULT_TIMEOUT        (((6UL * 256UL * 1000UL) / LSI_VALUE) + ((LSI_STARTUP_TIME / 1000UL) + 1UL))
#define IWDT_KERNEL_UPDATE_FLAGS        (IWDT_STS_CNTUFLG | IWDT_STS_PSCUFLG)
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup IWDT_Exported_Functions
  * @{
  */

/** @addtogroup IWDT_Exported_Functions_Group1
  *  @brief    Initialization and Start functions.
  *
@verbatim
 ===============================================================================
          ##### Initialization and Start functions #####
 ===============================================================================
 [..]  This section provides functions allowing to:
      (+) Initialize the IWDT according to the specified parameters in the
          IWDT_InitTypeDef of associated handle.
      (+) Once initialization is performed in DAL_IWDT_Init function, Watchdog
          is reloaded in order to exit function with correct time base.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the IWDT according to the specified parameters in the
  *         IWDT_InitTypeDef and start watchdog. Before exiting function,
  *         watchdog is refreshed in order to have correct time base.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IWDT_Init(IWDT_HandleTypeDef *hiwdt)
{
  uint32_t tickstart;

  /* Check the IWDT handle allocation */
  if (hiwdt == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_IWDT_ALL_INSTANCE(hiwdt->Instance));
  ASSERT_PARAM(IS_IWDT_PSCESCALER(hiwdt->Init.Prescaler));
  ASSERT_PARAM(IS_IWDT_RELOAD(hiwdt->Init.Reload));

  /* Enable IWDT. LSI is turned on automatically */
  __DAL_IWDT_START(hiwdt);

  /* Enable write access to IWDT_PSC and IWDT_CNTRLD registers by writing
  0x5555 in KR */
  IWDT_ENABLE_WRITE_ACCESS(hiwdt);

  /* Write to IWDT registers the Prescaler & Reload values to work with */
  hiwdt->Instance->PSC = hiwdt->Init.Prescaler;
  hiwdt->Instance->CNTRLD = hiwdt->Init.Reload;

  /* Check pending flag, if previous update not done, return timeout */
  tickstart = DAL_GetTick();

  /* Wait for register to be updated */
  while ((hiwdt->Instance->STS & IWDT_KERNEL_UPDATE_FLAGS) != 0x00u)
  {
    if ((DAL_GetTick() - tickstart) > DAL_IWDT_DEFAULT_TIMEOUT)
    {
      if ((hiwdt->Instance->STS & IWDT_KERNEL_UPDATE_FLAGS) != 0x00u)
      {
        return DAL_TIMEOUT;
      }
    }
  }

  /* Reload IWDT counter with value defined in the reload register */
  __DAL_IWDT_RELOAD_COUNTER(hiwdt);

  /* Return function status */
  return DAL_OK;
}


/**
  * @}
  */


/** @addtogroup IWDT_Exported_Functions_Group2
  *  @brief   IO operation functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]  This section provides functions allowing to:
      (+) Refresh the IWDT.

@endverbatim
  * @{
  */

/**
  * @brief  Refresh the IWDT.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IWDT_Refresh(IWDT_HandleTypeDef *hiwdt)
{
  /* Reload IWDT counter with value defined in the reload register */
  __DAL_IWDT_RELOAD_COUNTER(hiwdt);

  /* Return function status */
  return DAL_OK;
}


/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_IWDT_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
