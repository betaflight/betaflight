/**
  *
  * @file    apm32f4xx_dal_timebase_tmr_template.c 
  * @brief   DAL time base based on the hardware TMR Template.
  *    
  *          This file overrides the native DAL time base functions (defined as weak)
  *          the TMR time base:
  *           + Intializes the TMR peripheral generate a Period elapsed Event each 1ms
  *           + DAL_IncTick is called inside DAL_TMR_PeriodElapsedCallback ie each 1ms
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
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup DAL_TimeBase_TMR
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TMR_HandleTypeDef        TmrHandle;
/* Private function prototypes -----------------------------------------------*/
void TMR6_DAC_IRQHandler(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TMR6 as a time base source. 
  *         The time source is configured to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by DAL_Init() or at any time when clock is configured, by DAL_RCM_ClockConfig(). 
  * @param  TickPriority Tick interrupt priority.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_InitTick (uint32_t TickPriority)
{
  RCM_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB1Prescaler = 0U;
  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;
  DAL_StatusTypeDef     status;

  /* Enable TMR6 clock */
  __DAL_RCM_TMR6_CLK_ENABLE();

  /* Get clock configuration */
  DAL_RCM_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB1 prescaler */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;

  /* Compute TMR6 clock */
  if (uwAPB1Prescaler == RCM_HCLK_DIV1)
  {
    uwTimclock = DAL_RCM_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2 * DAL_RCM_GetPCLK1Freq();
  }

  /* Compute the prescaler value to have TMR6 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TMR6 */
  TmrHandle.Instance = TMR6;

  /* Initialize TMRx peripheral as follow:
  + Period = [(TMR6CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  TmrHandle.Init.Period = (1000000U / 1000U) - 1U;
  TmrHandle.Init.Prescaler = uwPrescalerValue;
  TmrHandle.Init.ClockDivision = 0U;
  TmrHandle.Init.CounterMode = TMR_COUNTERMODE_UP;
  TmrHandle.Init.AutoReloadPreload = TMR_AUTORELOAD_PRELOAD_DISABLE;
  status = DAL_TMR_Base_Init(&TmrHandle);
  if (status == DAL_OK)
  {
    /* Start the TMR time Base generation in interrupt mode */
    status = DAL_TMR_Base_Start_IT(&TmrHandle);
    if (status == DAL_OK)
    {
      /* Enable the TMR6 global Interrupt */
      DAL_NVIC_EnableIRQ(TMR6_DAC_IRQn);

      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Enable the TMR6 global Interrupt */
        DAL_NVIC_SetPriority(TMR6_DAC_IRQn, TickPriority, 0);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = DAL_ERROR;
      }
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TMR6 update interrupt.
  * @retval None
  */
void DAL_SuspendTick(void)
{
  /* Disable TMR6 update Interrupt */
  __DAL_TMR_DISABLE_IT(&TmrHandle, TMR_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TMR6 update interrupt.
  * @retval None
  */
void DAL_ResumeTick(void)
{
  /* Enable TMR6 Update interrupt */
  __DAL_TMR_ENABLE_IT(&TmrHandle, TMR_IT_UPDATE);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TMR6 interrupt took place, inside
  * DAL_TMR_IRQHandler(). It makes a direct call to DAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htmr  TMR handle
  * @retval None
  */
void DAL_TMR_PeriodElapsedCallback(TMR_HandleTypeDef *htmr)
{
  DAL_IncTick();
}

/**
  * @brief  This function handles TMR interrupt request.
  * @retval None
  */
void TMR6_DAC_IRQHandler(void)
{
  DAL_TMR_IRQHandler(&TmrHandle);
}

/**
  * @}
  */ 

/**
  * @}
  */ 


