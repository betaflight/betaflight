/**
  ******************************************************************************
  * @file    usbpd_timersserver.c
  * @author  MCD Application Team
  * @brief   This file contains timer server functions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbpd_devices_conf.h"
#include "usbpd_timersserver.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_TIMESERVER
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int8_t timer_initcounter = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize a timer to manage timing in us
  * @retval None
  */
void USBPD_TIM_Init(void)
{
  if (0 == timer_initcounter)
  {
    TIMX_CLK_ENABLE;
    /***************************/
    /* Time base configuration */
    /***************************/
    /* Counter mode: select up-counting mode */
    LL_TIM_SetCounterMode(TIMX, LL_TIM_COUNTERMODE_UP);

    /* Set the pre-scaler value to have TIMx counter clock equal to 1 MHz */
    LL_TIM_SetPrescaler(TIMX, __LL_TIM_CALC_PSC(SystemCoreClock, 1000000u));

    /* Set the auto-reload value to have a counter frequency of 100Hz */
    LL_TIM_SetAutoReload(TIMX, __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIMX), 100u));

    /*********************************/
    /* Output waveform configuration */
    /*********************************/
    /* Set output compare mode: TOGGLE */
    LL_TIM_OC_SetMode(TIMX, TIMX_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_OC_SetMode(TIMX, TIMX_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_OC_SetMode(TIMX, TIMX_CHANNEL_CH3, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_OC_SetMode(TIMX, TIMX_CHANNEL_CH4, LL_TIM_OCMODE_TOGGLE);

    /* Set output channel polarity: OC is active high */
    LL_TIM_OC_SetPolarity(TIMX, TIMX_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIMX, TIMX_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIMX, TIMX_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIMX, TIMX_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);

    /* Enable counter */
    LL_TIM_EnableCounter(TIMX);
  }

  /* Enable the timer counter */
  timer_initcounter++;
}

/**
  * @brief  UnInitialize a timer to manage timing in us
  * @retval None
  */
void USBPD_TIM_DeInit(void)
{
  timer_initcounter--;
  if (0 == timer_initcounter)
  {
    TIMX_CLK_DISABLE;
  }
}

/**
  * @brief  start the timer counting
  * @param  timer id @TIM_identifier
  * @param  time in us
  * @retval None
  */
void USBPD_TIM_Start(TIM_identifier Id, uint32_t TimeUs)
{
  /* Positionne l'evenement pour sa detection */
  switch (Id)
  {
    case TIM_PORT0_CRC:
      TIMX_CHANNEL1_SETEVENT;
      break;
    case TIM_PORT0_RETRY:
      TIMX_CHANNEL2_SETEVENT;
      break;
    case TIM_PORT1_CRC:
      TIMX_CHANNEL3_SETEVENT;
      break;
    case TIM_PORT1_RETRY:
      TIMX_CHANNEL4_SETEVENT;
      break;
    default:
      break;
  }
}

/**
  * @brief  check timer expiration
  * @param  timer id @TIM_identifier
  * @retval None
  */
uint32_t USBPD_TIM_IsExpired(TIM_identifier Id)
{
  uint32_t _expired = 1u;
  switch (Id)
  {
    case TIM_PORT0_CRC:
      _expired = TIMX_CHANNEL1_GETFLAG(TIMX);
      break;
    case TIM_PORT0_RETRY:
      _expired = TIMX_CHANNEL2_GETFLAG(TIMX);
      break;
    case TIM_PORT1_CRC:
      _expired = TIMX_CHANNEL3_GETFLAG(TIMX);
      break;
    case TIM_PORT1_RETRY:
      _expired = TIMX_CHANNEL4_GETFLAG(TIMX);
      break;
    default:
      break;
  }
  return _expired;
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

