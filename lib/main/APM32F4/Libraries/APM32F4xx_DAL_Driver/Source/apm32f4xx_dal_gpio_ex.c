/**
  *
  * @file    apm32f4xx_dal_gpio_ex.c
  * @brief   GPIO Extension DAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the General Purpose Input/Output (GPIO) extension peripheral:
  *           + Extended features functions
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
                    ##### GPIO Peripheral extension features #####
  ==============================================================================
  [..] GPIO module on APM32F402/403 family, manage also the AFIO register:
       (+) Possibility to use the EVENTOUT Cortex feature

                     ##### How to use this driver #####
  ==============================================================================
  [..] This driver provides functions to use EVENTOUT Cortex feature
    (#) Configure EVENTOUT Cortex feature using the function DAL_GPIOEx_ConfigEventout()
    (#) Activate EVENTOUT Cortex feature using the DAL_GPIOEx_EnableEventout()
    (#) Deactivate EVENTOUT Cortex feature using the DAL_GPIOEx_DisableEventout()

  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup GPIOEx GPIOEx
  * @brief GPIO HAL module driver
  * @{
  */

#ifdef DAL_GPIO_MODULE_ENABLED
#if defined(Apm32f403xx) || defined(Apm32f402xx)
/** @defgroup GPIOEx_Exported_Functions GPIOEx Exported Functions
  * @{
  */

/** @defgroup GPIOEx_Exported_Functions_Group1 Extended features functions
 *  @brief    Extended features functions
 *
@verbatim
  ==============================================================================
                 ##### Extended features functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
    (+) Configure EVENTOUT Cortex feature using the function DAL_GPIOEx_ConfigEventout()
    (+) Activate EVENTOUT Cortex feature using the DAL_GPIOEx_EnableEventout()
    (+) Deactivate EVENTOUT Cortex feature using the DAL_GPIOEx_DisableEventout()

@endverbatim
  * @{
  */

/**
  * @brief  Configures the port and pin on which the EVENTOUT Cortex signal will be connected.
  * @param  GPIO_PortSource Select the port used to output the Cortex EVENTOUT signal.
  *   This parameter can be a value of @ref GPIOEx_Eventout_port_selection.
  * @param  GPIO_PinSource Select the pin used to output the Cortex EVENTOUT signal.
  *   This parameter can be a value of @ref GPIOEx_Eventout_pin_selection.
  * @retval None
  */
void DAL_GPIOEx_ConfigEventout(uint32_t GPIO_PortSource, uint32_t GPIO_PinSource)
{
  /* Verify the parameters */
  assert_param(IS_AFIO_EVENTOUT_PORT(GPIO_PortSource));
  assert_param(IS_AFIO_EVENTOUT_PIN(GPIO_PinSource));

  /* Apply the new configuration */
  MODIFY_REG(AFIO->EVCTRL, (AFIO_EVCTRL_PORTSEL) | (AFIO_EVCTRL_PINSEL), (GPIO_PortSource) | (GPIO_PinSource));
}

/**
  * @brief  Enables the Event Output.
  * @retval None
  */
void DAL_GPIOEx_EnableEventout(void)
{
  SET_BIT(AFIO->EVCTRL, AFIO_EVCTRL_EVOEN);
}

/**
  * @brief  Disables the Event Output.
  * @retval None
  */
void DAL_GPIOEx_DisableEventout(void)
{
  CLEAR_BIT(AFIO->EVCTRL, AFIO_EVCTRL_EVOEN);
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* Apm32f403xx || Apm32f402xx */
#endif /* DAL_GPIO_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

