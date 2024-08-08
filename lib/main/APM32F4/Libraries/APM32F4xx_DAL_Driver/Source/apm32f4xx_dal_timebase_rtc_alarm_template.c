/**
  *
  * @file    apm32f4xx_dal_timebase_rtc_alarm_template.c 
  * @brief   DAL time base based on the hardware RTC_ALARM Template.
  *
  *          This file override the native DAL time base functions (defined as weak)
  *          to use the RTC ALARM for time base generation:
  *           + Intializes the RTC peripheral to increment the seconds registers each 1ms
  *           + The alarm is configured to assert an interrupt when the RTC reaches 1ms 
  *           + DAL_IncTick is called at each Alarm event and the time is reset to 00:00:00
  *           + HSE (default), LSE or LSI can be selected as RTC clock source
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
    This file must be copied to the application folder and modified as follows:
    (#) Rename it to 'apm32f4xx_dal_timebase_rtc_alarm.c'
    (#) Add this file and the RTC DAL drivers to your project and uncomment
       DAL_RTC_MODULE_ENABLED define in apm32f4xx_dal_cfg.h 

    [..]
    (@) DAL RTC alarm and DAL RTC wakeup drivers can not be used with low power modes:
        The wake up capability of the RTC may be intrusive in case of prior low power mode
        configuration requiring different wake up sources.
        Application/Example behavior is no more guaranteed 
    (@) The apm32f4xx_dal_timebase_tim use is recommended for the Applications/Examples
          requiring low power modes

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"
/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup DAL_TimeBase_RTC_Alarm_Template  DAL TimeBase RTC Alarm Template
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Uncomment the line below to select the appropriate RTC Clock source for your application: 
  + RTC_CLOCK_SOURCE_HSE: can be selected for applications requiring timing precision.
  + RTC_CLOCK_SOURCE_LSE: can be selected for applications with low constraint on timing
                          precision.
  + RTC_CLOCK_SOURCE_LSI: can be selected for applications with low constraint on timing
                          precision.
  */
#define RTC_CLOCK_SOURCE_HSE
/* #define RTC_CLOCK_SOURCE_LSE */
/* #define RTC_CLOCK_SOURCE_LSI */

#ifdef RTC_CLOCK_SOURCE_HSE
  #define RTC_ASYNCH_PREDIV       99U
  #define RTC_SYNCH_PREDIV        9U
  #define RCM_RTCCLKSOURCE_1MHZ   ((uint32_t)((uint32_t)RCM_BDCTRL_RTCSRCSEL | (uint32_t)((HSE_VALUE/1000000U) << 16U)))
#else /* RTC_CLOCK_SOURCE_LSE || RTC_CLOCK_SOURCE_LSI */
  #define RTC_ASYNCH_PREDIV       0U
  #define RTC_SYNCH_PREDIV        31U
#endif /* RTC_CLOCK_SOURCE_HSE */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef        hRTC_Handle;
/* Private function prototypes -----------------------------------------------*/
void RTC_Alarm_IRQHandler(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the RTC_ALARMA as a time base source. 
  *         The time source is configured  to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by DAL_Init() or at any time when clock is configured, by DAL_RCM_ClockConfig().
  * @param  TickPriority Tick interrupt priority.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_InitTick(uint32_t TickPriority)
{
  __IO uint32_t counter = 0U;

  RCM_OscInitTypeDef        RCM_OscInitStruct;
  RCM_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  DAL_StatusTypeDef     status;

#ifdef RTC_CLOCK_SOURCE_LSE
  /* Configue LSE as RTC clock soucre */
  RCM_OscInitStruct.OscillatorType = RCM_OSCILLATORTYPE_LSE;
  RCM_OscInitStruct.PLL.PLLState = RCM_PLL_NONE;
  RCM_OscInitStruct.LSEState = RCM_LSE_ON;
  PeriphClkInitStruct.RTCClockSelection = RCM_RTCCLKSOURCE_LSE;
#elif defined (RTC_CLOCK_SOURCE_LSI)
  /* Configue LSI as RTC clock soucre */
  RCM_OscInitStruct.OscillatorType = RCM_OSCILLATORTYPE_LSI;
  RCM_OscInitStruct.PLL.PLLState = RCM_PLL_NONE;
  RCM_OscInitStruct.LSIState = RCM_LSI_ON;
  PeriphClkInitStruct.RTCClockSelection = RCM_RTCCLKSOURCE_LSI;
#elif defined (RTC_CLOCK_SOURCE_HSE)
  /* Configue HSE as RTC clock soucre */
  RCM_OscInitStruct.OscillatorType = RCM_OSCILLATORTYPE_HSE;
  RCM_OscInitStruct.PLL.PLLState = RCM_PLL_NONE;
  RCM_OscInitStruct.HSEState = RCM_HSE_ON;
  /* Ensure that RTC is clocked by 1MHz */
  PeriphClkInitStruct.RTCClockSelection = RCM_RTCCLKSOURCE_1MHZ;
#else
#error Please select the RTC Clock source
#endif /* RTC_CLOCK_SOURCE_LSE */

  status = DAL_RCM_OscConfig(&RCM_OscInitStruct);
  if (status == DAL_OK)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCM_PERIPHCLK_RTC;
    status = DAL_RCMEx_PeriphCLKConfig(&PeriphClkInitStruct);
  }
  if (status == DAL_OK)
  {
    /* Enable RTC Clock */
    __DAL_RCM_RTC_ENABLE();
    /* The time base should be 1ms
       Time base = ((RTC_ASYNCH_PREDIV + 1) * (RTC_SYNCH_PREDIV + 1)) / RTC_CLOCK
       HSE as RTC clock
         Time base = ((99 + 1) * (9 + 1)) / 1MHz
                   = 1ms
       LSE as RTC clock
         Time base = ((31 + 1) * (0 + 1)) / 32.768KHz
                   = ~1ms
       LSI as RTC clock
         Time base = ((31 + 1) * (0 + 1)) / 32KHz
                   = 1ms
    */
    hRTC_Handle.Instance = RTC;
    hRTC_Handle.Init.HourFormat = RTC_HOURFORMAT_24;
    hRTC_Handle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    hRTC_Handle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    hRTC_Handle.Init.OutPut = RTC_OUTPUT_DISABLE;
    hRTC_Handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hRTC_Handle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    status = DAL_RTC_Init(&hRTC_Handle);
  }
  if (status == DAL_OK)
  {
    /* Disable the write protection for RTC registers */
    __DAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);

    /* Disable the Alarm A interrupt */
    __DAL_RTC_ALARMA_DISABLE(&hRTC_Handle);

    /* Clear flag alarm A */
    __DAL_RTC_ALARM_CLEAR_FLAG(&hRTC_Handle, RTC_FLAG_ALRAF);

    counter = 0U;
    /* Wait till RTC ALRAWF flag is set and if Time out is reached exit */
    while (__DAL_RTC_ALARM_GET_FLAG(&hRTC_Handle, RTC_FLAG_ALRAWF) == RESET)
    {
      if (counter++ == (SystemCoreClock / 48U)) /* Timeout = ~ 1s */
      {
        status =  DAL_ERROR;
      }
    }
  }
  if (status == DAL_OK)
  {
    hRTC_Handle.Instance->ALRMA = (uint32_t)0x01U;

    /* Configure the Alarm state: Enable Alarm */
    __DAL_RTC_ALARMA_ENABLE(&hRTC_Handle);
    /* Configure the Alarm interrupt */
    __DAL_RTC_ALARM_ENABLE_IT(&hRTC_Handle, RTC_IT_ALRA);

    /* RTC Alarm Interrupt Configuration: EXTI configuration */
    __DAL_RTC_ALARM_EINT_ENABLE_IT();
    __DAL_RTC_ALARM_EINT_ENABLE_RISING_EDGE();

    /* Check if the Initialization mode is set */
    if ((hRTC_Handle.Instance->STS & RTC_STS_RINITFLG) == (uint32_t)RESET)
    {
      /* Set the Initialization mode */
      hRTC_Handle.Instance->STS = (uint32_t)RTC_INIT_MASK;
      counter = 0U;
      while ((hRTC_Handle.Instance->STS & RTC_STS_RINITFLG) == (uint32_t)RESET)
      {
        if (counter++ == (SystemCoreClock / 48U)) /* Timeout = ~ 1s */
        {
          status = DAL_ERROR;
        }
      }
    }
  }
  if (status == DAL_OK)
  {
    hRTC_Handle.Instance->DATE = 0U;
    hRTC_Handle.Instance->TIME = 0U;

    hRTC_Handle.Instance->STS &= (uint32_t)~RTC_STS_INITEN;

    /* Enable the write protection for RTC registers */
    __DAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);

    /* Enable the RTC Alarm Interrupt */
    DAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

    /* Configure the SysTick IRQ priority */
    if (TickPriority < (1UL << __NVIC_PRIO_BITS))
    {
      DAL_NVIC_SetPriority(RTC_Alarm_IRQn, TickPriority, 0U);
      uwTickPrio = TickPriority;
    }
    else
    {
      status = DAL_ERROR;
    }

  }
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling RTC ALARM interrupt.
  * @retval None
  */
void DAL_SuspendTick(void)
{
  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);
  /* Disable RTC ALARM update Interrupt */
  __DAL_RTC_ALARM_DISABLE_IT(&hRTC_Handle, RTC_IT_ALRA);
  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling RTC ALARM interrupt.
  * @retval None
  */
void DAL_ResumeTick(void)
{
  /* Disable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);
  /* Enable RTC ALARM Update interrupt */
  __DAL_RTC_ALARM_ENABLE_IT(&hRTC_Handle, RTC_IT_ALRA);
  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);
}

/**
  * @brief  ALARM A Event Callback in non blocking mode
  * @note   This function is called  when RTC_ALARM interrupt took place, inside
  * RTC_ALARM_IRQHandler(). It makes a direct call to DAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  hrtc  RTC handle
  * @retval None
  */
void DAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  __IO uint32_t counter = 0U;

  DAL_IncTick();

  __DAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Set the Initialization mode */
  hrtc->Instance->STS = (uint32_t)RTC_INIT_MASK;

  while((hrtc->Instance->STS & RTC_STS_RINITFLG) == (uint32_t)RESET)
  {
    if(counter++ == (SystemCoreClock /48U)) /* Timeout = ~ 1s */
    {
      break;
    }
  }

  hrtc->Instance->DATE = 0U;
  hrtc->Instance->TIME = 0U;

  hrtc->Instance->STS &= (uint32_t)~RTC_STS_INITEN;

  /* Enable the write protection for RTC registers */
  __DAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
}

/**
  * @brief  This function handles RTC ALARM interrupt request.
  * @retval None
  */
void RTC_Alarm_IRQHandler(void)
{
  DAL_RTC_AlarmIRQHandler(&hRTC_Handle);
}

/**
  * @}
  */

/**
  * @}
  */


