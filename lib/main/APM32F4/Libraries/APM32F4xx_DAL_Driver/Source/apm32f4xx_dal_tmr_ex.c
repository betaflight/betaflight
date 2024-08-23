/**
  *
  * @file    apm32f4xx_dal_tmr_ex.c
  * @brief   TMR DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Timer Extended peripheral:
  *           + Time Hall Sensor Interface Initialization
  *           + Time Hall Sensor Interface Start
  *           + Time Complementary signal break and dead time configuration
  *           + Time Master and Slave synchronization configuration
  *           + Timer remapping capabilities configuration
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
                      ##### TMRER Extended features #####
  ==============================================================================
  [..]
    The Timer Extended features include:
    (#) Complementary outputs with programmable dead-time for :
        (++) Output Compare
        (++) PWM generation (Edge and Center-aligned Mode)
        (++) One-pulse mode output
    (#) Synchronization circuit to control the timer with external signals and to
        interconnect several timers together.
    (#) Break input to put the timer output signals in reset state or in a known state.
    (#) Supports incremental (quadrature) encoder and hall-sensor circuitry for
        positioning purposes

            ##### How to use this driver #####
  ==============================================================================
    [..]
     (#) Initialize the TMR low level resources by implementing the following functions
         depending on the selected feature:
           (++) Hall Sensor output : DAL_TMREx_HallSensor_MspInit()

     (#) Initialize the TMR low level resources :
        (##) Enable the TMR interface clock using __DAL_RCM_TMRx_CLK_ENABLE();
        (##) TMR pins configuration
            (+++) Enable the clock for the TMR GPIOs using the following function:
              __DAL_RCM_GPIOx_CLK_ENABLE();
            (+++) Configure these TMR pins in Alternate function mode using DAL_GPIO_Init();

     (#) The external Clock can be configured, if needed (the default clock is the
         internal clock from the APBx), using the following function:
         DAL_TMR_ConfigClockSource, the clock configuration should be done before
         any start function.

     (#) Configure the TMR in the desired functioning mode using one of the
         initialization function of this driver:
          (++) DAL_TMREx_HallSensor_Init() and DAL_TMREx_ConfigCommutEvent(): to use the
               Timer Hall Sensor Interface and the commutation event with the corresponding
               Interrupt and DMA request if needed (Note that One Timer is used to interface
               with the Hall sensor Interface and another Timer should be used to use
               the commutation event).

     (#) Activate the TMR peripheral using one of the start functions:
           (++) Complementary Output Compare : DAL_TMREx_OCN_Start(), DAL_TMREx_OCN_Start_DMA(),
                DAL_TMREx_OCN_Start_IT()
           (++) Complementary PWM generation : DAL_TMREx_PWMN_Start(), DAL_TMREx_PWMN_Start_DMA(),
                DAL_TMREx_PWMN_Start_IT()
           (++) Complementary One-pulse mode output : DAL_TMREx_OnePulseN_Start(), DAL_TMREx_OnePulseN_Start_IT()
           (++) Hall Sensor output : DAL_TMREx_HallSensor_Start(), DAL_TMREx_HallSensor_Start_DMA(),
                DAL_TMREx_HallSensor_Start_IT().

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup TMREx TMREx
  * @brief TMR Extended DAL module driver
  * @{
  */

#ifdef DAL_TMR_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void TMR_DMADelayPulseNCplt(DMA_HandleTypeDef *hdma);
static void TMR_DMAErrorCCxN(DMA_HandleTypeDef *hdma);
static void TMR_CCxNChannelCmd(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ChannelNState);

/* Exported functions --------------------------------------------------------*/
/** @defgroup TMREx_Exported_Functions TMR Extended Exported Functions
  * @{
  */

/** @defgroup TMREx_Exported_Functions_Group1 Extended Timer Hall Sensor functions
  * @brief    Timer Hall Sensor functions
  *
@verbatim
  ==============================================================================
                      ##### Timer Hall Sensor functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure TMR DAL Sensor.
    (+) De-initialize TMR DAL Sensor.
    (+) Start the Hall Sensor Interface.
    (+) Stop the Hall Sensor Interface.
    (+) Start the Hall Sensor Interface and enable interrupts.
    (+) Stop the Hall Sensor Interface and disable interrupts.
    (+) Start the Hall Sensor Interface and enable DMA transfers.
    (+) Stop the Hall Sensor Interface and disable DMA transfers.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TMR Hall Sensor Interface and initialize the associated handle.
  * @note   When the timer instance is initialized in Hall Sensor Interface mode,
  *         timer channels 1 and channel 2 are reserved and cannot be used for
  *         other purpose.
  * @param  htmr TMR Hall Sensor Interface handle
  * @param  sConfig TMR Hall Sensor configuration structure
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Init(TMR_HandleTypeDef *htmr, TMR_HallSensor_InitTypeDef *sConfig)
{
  TMR_OC_InitTypeDef OC_Config;

  /* Check the TMR handle allocation */
  if (htmr == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_COUNTER_MODE(htmr->Init.CounterMode));
  ASSERT_PARAM(IS_TMR_CLOCKDIVISION_DIV(htmr->Init.ClockDivision));
  ASSERT_PARAM(IS_TMR_AUTORELOAD_PRELOAD(htmr->Init.AutoReloadPreload));
  ASSERT_PARAM(IS_TMR_IC_POLARITY(sConfig->IC1Polarity));
  ASSERT_PARAM(IS_TMR_IC_PRESCALER(sConfig->IC1Prescaler));
  ASSERT_PARAM(IS_TMR_IC_FILTER(sConfig->IC1Filter));

  if (htmr->State == DAL_TMR_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htmr->Lock = DAL_UNLOCKED;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
    /* Reset interrupt callbacks to legacy week callbacks */
    TMR_ResetCallback(htmr);

    if (htmr->HallSensor_MspInitCallback == NULL)
    {
      htmr->HallSensor_MspInitCallback = DAL_TMREx_HallSensor_MspInit;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    htmr->HallSensor_MspInitCallback(htmr);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_TMREx_HallSensor_MspInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Configure the Time base in the Encoder Mode */
  TMR_Base_SetConfig(htmr->Instance, &htmr->Init);

  /* Configure the Channel 1 as Input Channel to interface with the three Outputs of the  Hall sensor */
  TMR_TI1_SetConfig(htmr->Instance, sConfig->IC1Polarity, TMR_ICSELECTION_TRC, sConfig->IC1Filter);

  /* Reset the IC1PSC Bits */
  htmr->Instance->CCM1 &= ~TMR_CCM1_IC1PSC;
  /* Set the IC1PSC value */
  htmr->Instance->CCM1 |= sConfig->IC1Prescaler;

  /* Enable the Hall sensor interface (XOR function of the three inputs) */
  htmr->Instance->CTRL2 |= TMR_CTRL2_TI1SEL;

  /* Select the TMR_TS_TI1F_ED signal as Input trigger for the TMR */
  htmr->Instance->SMCTRL &= ~TMR_SMCTRL_TRGSEL;
  htmr->Instance->SMCTRL |= TMR_TS_TI1F_ED;

  /* Use the TMR_TS_TI1F_ED signal to reset the TMR counter each edge detection */
  htmr->Instance->SMCTRL &= ~TMR_SMCTRL_SMFSEL;
  htmr->Instance->SMCTRL |= TMR_SLAVEMODE_RESET;

  /* Program channel 2 in PWM 2 mode with the desired Commutation_Delay*/
  OC_Config.OCFastMode = TMR_OCFAST_DISABLE;
  OC_Config.OCIdleState = TMR_OCIDLESTATE_RESET;
  OC_Config.OCMode = TMR_OCMODE_PWM2;
  OC_Config.OCNIdleState = TMR_OCNIDLESTATE_RESET;
  OC_Config.OCNPolarity = TMR_OCNPOLARITY_HIGH;
  OC_Config.OCPolarity = TMR_OCPOLARITY_HIGH;
  OC_Config.Pulse = sConfig->Commutation_Delay;

  TMR_OC2_SetConfig(htmr->Instance, &OC_Config);

  /* Select OC2REF as trigger output on TRGO: write the MMS bits in the TMRx_CTRL2
    register to 101 */
  htmr->Instance->CTRL2 &= ~TMR_CTRL2_MMSEL;
  htmr->Instance->CTRL2 |= TMR_TRGO_OC2REF;

  /* Initialize the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;

  /* Initialize the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Initialize the TMR state*/
  htmr->State = DAL_TMR_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the TMR Hall Sensor interface
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_DeInit(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Disable the TMR Peripheral Clock */
  __DAL_TMR_DISABLE(htmr);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  if (htmr->HallSensor_MspDeInitCallback == NULL)
  {
    htmr->HallSensor_MspDeInitCallback = DAL_TMREx_HallSensor_MspDeInit;
  }
  /* DeInit the low level hardware */
  htmr->HallSensor_MspDeInitCallback(htmr);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  DAL_TMREx_HallSensor_MspDeInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  /* Change the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_RESET;

  /* Change the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_RESET);

  /* Change TMR state */
  htmr->State = DAL_TMR_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Initializes the TMR Hall Sensor MSP.
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval None
  */
__weak void DAL_TMREx_HallSensor_MspInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMREx_HallSensor_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes TMR Hall Sensor MSP.
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval None
  */
__weak void DAL_TMREx_HallSensor_MspDeInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMREx_HallSensor_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  Starts the TMR Hall Sensor Interface.
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Start(TMR_HandleTypeDef *htmr)
{
  uint32_t tmpsmcr;
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(htmr->Instance));

  /* Check the TMR channels state */
  if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the Input Capture channel 1
  (in the Hall Sensor Interface the three possible channels that can be used are TMR_CHANNEL_1,
  TMR_CHANNEL_2 and TMR_CHANNEL_3) */
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Hall sensor Interface.
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Stop(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(htmr->Instance));

  /* Disable the Input Capture channels 1, 2 and 3
  (in the Hall Sensor Interface the three possible channels that can be used are TMR_CHANNEL_1,
  TMR_CHANNEL_2 and TMR_CHANNEL_3) */
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Hall Sensor Interface in interrupt mode.
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Start_IT(TMR_HandleTypeDef *htmr)
{
  uint32_t tmpsmcr;
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(htmr->Instance));

  /* Check the TMR channels state */
  if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the capture compare Interrupts 1 event */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);

  /* Enable the Input Capture channel 1
  (in the Hall Sensor Interface the three possible channels that can be used are TMR_CHANNEL_1,
  TMR_CHANNEL_2 and TMR_CHANNEL_3) */
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Hall Sensor Interface in interrupt mode.
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Stop_IT(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(htmr->Instance));

  /* Disable the Input Capture channel 1
  (in the Hall Sensor Interface the three possible channels that can be used are TMR_CHANNEL_1,
  TMR_CHANNEL_2 and TMR_CHANNEL_3) */
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);

  /* Disable the capture compare Interrupts event */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Hall Sensor Interface in DMA mode.
  * @param  htmr TMR Hall Sensor Interface handle
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from TMR peripheral to memory.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t *pData, uint16_t Length)
{
  uint32_t tmpsmcr;
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(htmr->Instance));

  /* Set the TMR channel state */
  if ((channel_1_state == DAL_TMR_CHANNEL_STATE_BUSY)
      || (complementary_channel_1_state == DAL_TMR_CHANNEL_STATE_BUSY))
  {
    return DAL_BUSY;
  }
  else if ((channel_1_state == DAL_TMR_CHANNEL_STATE_READY)
           && (complementary_channel_1_state == DAL_TMR_CHANNEL_STATE_READY))
  {
    if ((pData == NULL) && (Length > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    return DAL_ERROR;
  }

  /* Enable the Input Capture channel 1
  (in the Hall Sensor Interface the three possible channels that can be used are TMR_CHANNEL_1,
  TMR_CHANNEL_2 and TMR_CHANNEL_3) */
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);

  /* Set the DMA Input Capture 1 Callbacks */
  htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMACaptureCplt;
  htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;
  /* Set the DMA error callback */
  htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

  /* Enable the DMA stream for Capture 1*/
  if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)&htmr->Instance->CC1, (uint32_t)pData, Length) != DAL_OK)
  {
    /* Return error status */
    return DAL_ERROR;
  }
  /* Enable the capture compare 1 Interrupt */
  __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Hall Sensor Interface in DMA mode.
  * @param  htmr TMR Hall Sensor Interface handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Stop_DMA(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_HALL_SENSOR_INTERFACE_INSTANCE(htmr->Instance));

  /* Disable the Input Capture channel 1
  (in the Hall Sensor Interface the three possible channels that can be used are TMR_CHANNEL_1,
  TMR_CHANNEL_2 and TMR_CHANNEL_3) */
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);


  /* Disable the capture compare Interrupts 1 event */
  __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);

  (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup TMREx_Exported_Functions_Group2 Extended Timer Complementary Output Compare functions
  *  @brief   Timer Complementary Output Compare functions
  *
@verbatim
  ==============================================================================
              ##### Timer Complementary Output Compare functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Start the Complementary Output Compare/PWM.
    (+) Stop the Complementary Output Compare/PWM.
    (+) Start the Complementary Output Compare/PWM and enable interrupts.
    (+) Stop the Complementary Output Compare/PWM and disable interrupts.
    (+) Start the Complementary Output Compare/PWM and enable DMA transfers.
    (+) Stop the Complementary Output Compare/PWM and disable DMA transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Starts the TMR Output Compare signal generation on the complementary
  *         output.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OCN_Start(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR complementary channel state */
  if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR complementary channel state */
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare channel N */
  TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_ENABLE);

  /* Enable the Main Output */
  __DAL_TMR_MOE_ENABLE(htmr);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Output Compare signal generation on the complementary
  *         output.
  * @param  htmr TMR handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OCN_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Disable the Capture compare channel N */
  TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_DISABLE);

  /* Disable the Main Output */
  __DAL_TMR_MOE_DISABLE(htmr);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR complementary channel state */
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Output Compare signal generation in interrupt mode
  *         on the complementary output.
  * @param  htmr TMR OC handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OCN_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR complementary channel state */
  if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR complementary channel state */
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Enable the TMR Output Compare interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Enable the TMR Output Compare interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Enable the TMR Output Compare interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC3);
      break;
    }


    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the TMR Break interrupt */
    __DAL_TMR_ENABLE_IT(htmr, TMR_IT_BREAK);

    /* Enable the Capture compare channel N */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_ENABLE);

    /* Enable the Main Output */
    __DAL_TMR_MOE_ENABLE(htmr);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR Output Compare signal generation in interrupt mode
  *         on the complementary output.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OCN_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpccer;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Output Compare interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Output Compare interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Output Compare interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Capture compare channel N */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_DISABLE);

    /* Disable the TMR Break interrupt (only if no more channel is active) */
    tmpccer = htmr->Instance->CCEN;
    if ((tmpccer & (TMR_CCEN_CC1NEN | TMR_CCEN_CC2NEN | TMR_CCEN_CC3NEN)) == (uint32_t)RESET)
    {
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_BREAK);
    }

    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR complementary channel state */
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Starts the TMR Output Compare signal generation in DMA mode
  *         on the complementary output.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TMR peripheral
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OCN_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Set the TMR complementary channel state */
  if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_READY)
  {
    if ((pData == NULL) && (Length > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    return DAL_ERROR;
  }

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMADelayPulseNCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAErrorCCxN ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htmr->Instance->CC1,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Output Compare DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMADelayPulseNCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAErrorCCxN ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htmr->Instance->CC2,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Output Compare DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMADelayPulseNCplt;
      htmr->hdma[TMR_DMA_ID_CC3]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAErrorCCxN ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htmr->Instance->CC3,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Output Compare DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC3);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the Capture compare channel N */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_ENABLE);

    /* Enable the Main Output */
    __DAL_TMR_MOE_ENABLE(htmr);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR Output Compare signal generation in DMA mode
  *         on the complementary output.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OCN_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Output Compare DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Output Compare DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC2);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Output Compare DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC3);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC3]);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Capture compare channel N */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_DISABLE);

    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR complementary channel state */
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @}
  */

/** @defgroup TMREx_Exported_Functions_Group3 Extended Timer Complementary PWM functions
  * @brief    Timer Complementary PWM functions
  *
@verbatim
  ==============================================================================
                 ##### Timer Complementary PWM functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Start the Complementary PWM.
    (+) Stop the Complementary PWM.
    (+) Start the Complementary PWM and enable interrupts.
    (+) Stop the Complementary PWM and disable interrupts.
    (+) Start the Complementary PWM and enable DMA transfers.
    (+) Stop the Complementary PWM and disable DMA transfers.
    (+) Start the Complementary Input Capture measurement.
    (+) Stop the Complementary Input Capture.
    (+) Start the Complementary Input Capture and enable interrupts.
    (+) Stop the Complementary Input Capture and disable interrupts.
    (+) Start the Complementary Input Capture and enable DMA transfers.
    (+) Stop the Complementary Input Capture and disable DMA transfers.
    (+) Start the Complementary One Pulse generation.
    (+) Stop the Complementary One Pulse.
    (+) Start the Complementary One Pulse and enable interrupts.
    (+) Stop the Complementary One Pulse and disable interrupts.

@endverbatim
  * @{
  */

/**
  * @brief  Starts the PWM signal generation on the complementary output.
  * @param  htmr TMR handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_PWMN_Start(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR complementary channel state */
  if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR complementary channel state */
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the complementary PWM output  */
  TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_ENABLE);

  /* Enable the Main Output */
  __DAL_TMR_MOE_ENABLE(htmr);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the PWM signal generation on the complementary output.
  * @param  htmr TMR handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_PWMN_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Disable the complementary PWM output  */
  TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_DISABLE);

  /* Disable the Main Output */
  __DAL_TMR_MOE_DISABLE(htmr);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR complementary channel state */
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the PWM signal generation in interrupt mode on the
  *         complementary output.
  * @param  htmr TMR handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_PWMN_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR complementary channel state */
  if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR complementary channel state */
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Enable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Enable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Enable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the TMR Break interrupt */
    __DAL_TMR_ENABLE_IT(htmr, TMR_IT_BREAK);

    /* Enable the complementary PWM output  */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_ENABLE);

    /* Enable the Main Output */
    __DAL_TMR_MOE_ENABLE(htmr);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the PWM signal generation in interrupt mode on the
  *         complementary output.
  * @param  htmr TMR handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_PWMN_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpccer;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the complementary PWM output  */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_DISABLE);

    /* Disable the TMR Break interrupt (only if no more channel is active) */
    tmpccer = htmr->Instance->CCEN;
    if ((tmpccer & (TMR_CCEN_CC1NEN | TMR_CCEN_CC2NEN | TMR_CCEN_CC3NEN)) == (uint32_t)RESET)
    {
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_BREAK);
    }

    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR complementary channel state */
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Starts the TMR PWM signal generation in DMA mode on the
  *         complementary output
  * @param  htmr TMR handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TMR peripheral
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_PWMN_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  /* Set the TMR complementary channel state */
  if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (TMR_CHANNEL_N_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_READY)
  {
    if ((pData == NULL) && (Length > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    return DAL_ERROR;
  }

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMADelayPulseNCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAErrorCCxN ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htmr->Instance->CC1,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMADelayPulseNCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAErrorCCxN ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htmr->Instance->CC2,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 2 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMADelayPulseNCplt;
      htmr->hdma[TMR_DMA_ID_CC3]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAErrorCCxN ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htmr->Instance->CC3,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 3 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC3);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the complementary PWM output  */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_ENABLE);

    /* Enable the Main Output */
    __DAL_TMR_MOE_ENABLE(htmr);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR PWM signal generation in DMA mode on the complementary
  *         output
  * @param  htmr TMR handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_PWMN_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC2);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC3);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC3]);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the complementary PWM output */
    TMR_CCxNChannelCmd(htmr->Instance, Channel, TMR_CCxN_DISABLE);

    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR complementary channel state */
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @}
  */

/** @defgroup TMREx_Exported_Functions_Group4 Extended Timer Complementary One Pulse functions
  * @brief    Timer Complementary One Pulse functions
  *
@verbatim
  ==============================================================================
                ##### Timer Complementary One Pulse functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Start the Complementary One Pulse generation.
    (+) Stop the Complementary One Pulse.
    (+) Start the Complementary One Pulse and enable interrupts.
    (+) Stop the Complementary One Pulse and disable interrupts.

@endverbatim
  * @{
  */

/**
  * @brief  Starts the TMR One Pulse signal generation on the complementary
  *         output.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel pulse output channel to enable
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Start(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  uint32_t input_channel = (OutputChannel == TMR_CHANNEL_1) ? TMR_CHANNEL_2 : TMR_CHANNEL_1;
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, OutputChannel));

  /* Check the TMR channels state */
  if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the complementary One Pulse output channel and the Input Capture channel */
  TMR_CCxNChannelCmd(htmr->Instance, OutputChannel, TMR_CCxN_ENABLE);
  TMR_CCxChannelCmd(htmr->Instance, input_channel, TMR_CCx_ENABLE);

  /* Enable the Main Output */
  __DAL_TMR_MOE_ENABLE(htmr);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR One Pulse signal generation on the complementary
  *         output.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel pulse output channel to disable
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Stop(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  uint32_t input_channel = (OutputChannel == TMR_CHANNEL_1) ? TMR_CHANNEL_2 : TMR_CHANNEL_1;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, OutputChannel));

  /* Disable the complementary One Pulse output channel and the Input Capture channel */
  TMR_CCxNChannelCmd(htmr->Instance, OutputChannel, TMR_CCxN_DISABLE);
  TMR_CCxChannelCmd(htmr->Instance, input_channel, TMR_CCx_DISABLE);

  /* Disable the Main Output */
  __DAL_TMR_MOE_DISABLE(htmr);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR  channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR One Pulse signal generation in interrupt mode on the
  *         complementary channel.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel pulse output channel to enable
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Start_IT(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  uint32_t input_channel = (OutputChannel == TMR_CHANNEL_1) ? TMR_CHANNEL_2 : TMR_CHANNEL_1;
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, OutputChannel));

  /* Check the TMR channels state */
  if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the TMR Capture/Compare 1 interrupt */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);

  /* Enable the TMR Capture/Compare 2 interrupt */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);

  /* Enable the complementary One Pulse output channel and the Input Capture channel */
  TMR_CCxNChannelCmd(htmr->Instance, OutputChannel, TMR_CCxN_ENABLE);
  TMR_CCxChannelCmd(htmr->Instance, input_channel, TMR_CCx_ENABLE);

  /* Enable the Main Output */
  __DAL_TMR_MOE_ENABLE(htmr);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR One Pulse signal generation in interrupt mode on the
  *         complementary channel.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel pulse output channel to disable
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  uint32_t input_channel = (OutputChannel == TMR_CHANNEL_1) ? TMR_CHANNEL_2 : TMR_CHANNEL_1;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, OutputChannel));

  /* Disable the TMR Capture/Compare 1 interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);

  /* Disable the TMR Capture/Compare 2 interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);

  /* Disable the complementary One Pulse output channel and the Input Capture channel */
  TMR_CCxNChannelCmd(htmr->Instance, OutputChannel, TMR_CCxN_DISABLE);
  TMR_CCxChannelCmd(htmr->Instance, input_channel, TMR_CCx_DISABLE);

  /* Disable the Main Output */
  __DAL_TMR_MOE_DISABLE(htmr);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR  channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup TMREx_Exported_Functions_Group5 Extended Peripheral Control functions
  * @brief    Peripheral Control functions
  *
@verbatim
  ==============================================================================
                    ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Configure the commutation event in case of use of the Hall sensor interface.
      (+) Configure Output channels for OC and PWM mode.

      (+) Configure Complementary channels, break features and dead time.
      (+) Configure Master synchronization.
      (+) Configure timer remapping capabilities.

@endverbatim
  * @{
  */

/**
  * @brief  Configure the TMR commutation event sequence.
  * @note  This function is mandatory to use the commutation event in order to
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer)
  *        configured in Hall sensor interface, this interface Timer will generate the
  *        commutation at its TRGO output (connected to Timer used in this function) each time
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @param  htmr TMR handle
  * @param  InputTrigger the Internal trigger corresponding to the Timer Interfacing with the Hall sensor
  *          This parameter can be one of the following values:
  *            @arg TMR_TS_ITR0: Internal trigger 0 selected
  *            @arg TMR_TS_ITR1: Internal trigger 1 selected
  *            @arg TMR_TS_ITR2: Internal trigger 2 selected
  *            @arg TMR_TS_ITR3: Internal trigger 3 selected
  *            @arg TMR_TS_NONE: No trigger is needed
  * @param  CommutationSource the Commutation Event source
  *          This parameter can be one of the following values:
  *            @arg TMR_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TMR_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_ConfigCommutEvent(TMR_HandleTypeDef *htmr, uint32_t  InputTrigger,
                                              uint32_t  CommutationSource)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_COMMUTATION_EVENT_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_INTERNAL_TRIGGEREVENT_SELECTION(InputTrigger));

  __DAL_LOCK(htmr);

  if ((InputTrigger == TMR_TS_ITR0) || (InputTrigger == TMR_TS_ITR1) ||
      (InputTrigger == TMR_TS_ITR2) || (InputTrigger == TMR_TS_ITR3))
  {
    /* Select the Input trigger */
    htmr->Instance->SMCTRL &= ~TMR_SMCTRL_TRGSEL;
    htmr->Instance->SMCTRL |= InputTrigger;
  }

  /* Select the Capture Compare preload feature */
  htmr->Instance->CTRL2 |= TMR_CTRL2_CCPEN;
  /* Select the Commutation event source */
  htmr->Instance->CTRL2 &= ~TMR_CTRL2_CCUSEL;
  htmr->Instance->CTRL2 |= CommutationSource;

  /* Disable Commutation Interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_COM);

  /* Disable Commutation DMA request */
  __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_COM);

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Configure the TMR commutation event sequence with interrupt.
  * @note  This function is mandatory to use the commutation event in order to
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer)
  *        configured in Hall sensor interface, this interface Timer will generate the
  *        commutation at its TRGO output (connected to Timer used in this function) each time
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @param  htmr TMR handle
  * @param  InputTrigger the Internal trigger corresponding to the Timer Interfacing with the Hall sensor
  *          This parameter can be one of the following values:
  *            @arg TMR_TS_ITR0: Internal trigger 0 selected
  *            @arg TMR_TS_ITR1: Internal trigger 1 selected
  *            @arg TMR_TS_ITR2: Internal trigger 2 selected
  *            @arg TMR_TS_ITR3: Internal trigger 3 selected
  *            @arg TMR_TS_NONE: No trigger is needed
  * @param  CommutationSource the Commutation Event source
  *          This parameter can be one of the following values:
  *            @arg TMR_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TMR_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_ConfigCommutEvent_IT(TMR_HandleTypeDef *htmr, uint32_t  InputTrigger,
                                                 uint32_t  CommutationSource)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_COMMUTATION_EVENT_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_INTERNAL_TRIGGEREVENT_SELECTION(InputTrigger));

  __DAL_LOCK(htmr);

  if ((InputTrigger == TMR_TS_ITR0) || (InputTrigger == TMR_TS_ITR1) ||
      (InputTrigger == TMR_TS_ITR2) || (InputTrigger == TMR_TS_ITR3))
  {
    /* Select the Input trigger */
    htmr->Instance->SMCTRL &= ~TMR_SMCTRL_TRGSEL;
    htmr->Instance->SMCTRL |= InputTrigger;
  }

  /* Select the Capture Compare preload feature */
  htmr->Instance->CTRL2 |= TMR_CTRL2_CCPEN;
  /* Select the Commutation event source */
  htmr->Instance->CTRL2 &= ~TMR_CTRL2_CCUSEL;
  htmr->Instance->CTRL2 |= CommutationSource;

  /* Disable Commutation DMA request */
  __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_COM);

  /* Enable the Commutation Interrupt */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_COM);

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Configure the TMR commutation event sequence with DMA.
  * @note  This function is mandatory to use the commutation event in order to
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer)
  *        configured in Hall sensor interface, this interface Timer will generate the
  *        commutation at its TRGO output (connected to Timer used in this function) each time
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @note  The user should configure the DMA in his own software, in This function only the COMDE bit is set
  * @param  htmr TMR handle
  * @param  InputTrigger the Internal trigger corresponding to the Timer Interfacing with the Hall sensor
  *          This parameter can be one of the following values:
  *            @arg TMR_TS_ITR0: Internal trigger 0 selected
  *            @arg TMR_TS_ITR1: Internal trigger 1 selected
  *            @arg TMR_TS_ITR2: Internal trigger 2 selected
  *            @arg TMR_TS_ITR3: Internal trigger 3 selected
  *            @arg TMR_TS_NONE: No trigger is needed
  * @param  CommutationSource the Commutation Event source
  *          This parameter can be one of the following values:
  *            @arg TMR_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TMR_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_ConfigCommutEvent_DMA(TMR_HandleTypeDef *htmr, uint32_t  InputTrigger,
                                                  uint32_t  CommutationSource)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_COMMUTATION_EVENT_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_INTERNAL_TRIGGEREVENT_SELECTION(InputTrigger));

  __DAL_LOCK(htmr);

  if ((InputTrigger == TMR_TS_ITR0) || (InputTrigger == TMR_TS_ITR1) ||
      (InputTrigger == TMR_TS_ITR2) || (InputTrigger == TMR_TS_ITR3))
  {
    /* Select the Input trigger */
    htmr->Instance->SMCTRL &= ~TMR_SMCTRL_TRGSEL;
    htmr->Instance->SMCTRL |= InputTrigger;
  }

  /* Select the Capture Compare preload feature */
  htmr->Instance->CTRL2 |= TMR_CTRL2_CCPEN;
  /* Select the Commutation event source */
  htmr->Instance->CTRL2 &= ~TMR_CTRL2_CCUSEL;
  htmr->Instance->CTRL2 |= CommutationSource;

  /* Enable the Commutation DMA Request */
  /* Set the DMA Commutation Callback */
  htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferCpltCallback = TMREx_DMACommutationCplt;
  htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferHalfCpltCallback = TMREx_DMACommutationHalfCplt;
  /* Set the DMA error callback */
  htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferErrorCallback = TMR_DMAError;

  /* Disable Commutation Interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_COM);

  /* Enable the Commutation DMA Request */
  __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_COM);

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Configures the TMR in master mode.
  * @param  htmr TMR handle.
  * @param  sMasterConfig pointer to a TMR_MasterConfigTypeDef structure that
  *         contains the selected trigger output (TRGO) and the Master/Slave
  *         mode.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_MasterConfigSynchronization(TMR_HandleTypeDef *htmr,
                                                        TMR_MasterConfigTypeDef *sMasterConfig)
{
  uint32_t tmpcr2;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_MASTER_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_TRGO_SOURCE(sMasterConfig->MasterOutputTrigger));
  ASSERT_PARAM(IS_TMR_MSM_STATE(sMasterConfig->MasterSlaveMode));

  /* Check input state */
  __DAL_LOCK(htmr);

  /* Change the handler state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = htmr->Instance->CTRL2;

  /* Get the TMRx SMCTRL register value */
  tmpsmcr = htmr->Instance->SMCTRL;

  /* Reset the MMS Bits */
  tmpcr2 &= ~TMR_CTRL2_MMSEL;
  /* Select the TRGO source */
  tmpcr2 |=  sMasterConfig->MasterOutputTrigger;

  /* Update TMRx CTRL2 */
  htmr->Instance->CTRL2 = tmpcr2;

  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    /* Reset the MSM Bit */
    tmpsmcr &= ~TMR_SMCTRL_MSMEN;
    /* Set master mode */
    tmpsmcr |= sMasterConfig->MasterSlaveMode;

    /* Update TMRx SMCTRL */
    htmr->Instance->SMCTRL = tmpsmcr;
  }

  /* Change the htmr state */
  htmr->State = DAL_TMR_STATE_READY;

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Configures the Break feature, dead time, Lock level, OSSI/OSSR State
  *         and the AOE(automatic output enable).
  * @param  htmr TMR handle
  * @param  sBreakDeadTimeConfig pointer to a TMR_ConfigBreakDeadConfigTypeDef structure that
  *         contains the BDTR Register configuration  information for the TMR peripheral.
  * @note   Interrupts can be generated when an active level is detected on the
  *         break input, the break 2 input or the system break input. Break
  *         interrupt can be enabled by calling the @ref __DAL_TMR_ENABLE_IT macro.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_ConfigBreakDeadTime(TMR_HandleTypeDef *htmr,
                                                TMR_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig)
{
  /* Keep this variable initialized to 0 as it is used to configure BDTR register */
  uint32_t tmpbdtr = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_BREAK_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_OSSR_STATE(sBreakDeadTimeConfig->OffStateRunMode));
  ASSERT_PARAM(IS_TMR_OSSI_STATE(sBreakDeadTimeConfig->OffStateIDLEMode));
  ASSERT_PARAM(IS_TMR_LOCK_LEVEL(sBreakDeadTimeConfig->LockLevel));
  ASSERT_PARAM(IS_TMR_DEADTIME(sBreakDeadTimeConfig->DeadTime));
  ASSERT_PARAM(IS_TMR_BREAK_STATE(sBreakDeadTimeConfig->BreakState));
  ASSERT_PARAM(IS_TMR_BREAK_POLARITY(sBreakDeadTimeConfig->BreakPolarity));
  ASSERT_PARAM(IS_TMR_AUTOMATIC_OUTPUT_STATE(sBreakDeadTimeConfig->AutomaticOutput));

  /* Check input state */
  __DAL_LOCK(htmr);

  /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
     the OSSI State, the dead time value and the Automatic Output Enable Bit */

  /* Set the BDT bits */
  MODIFY_REG(tmpbdtr, TMR_BDT_DTS, sBreakDeadTimeConfig->DeadTime);
  MODIFY_REG(tmpbdtr, TMR_BDT_LOCKCFG, sBreakDeadTimeConfig->LockLevel);
  MODIFY_REG(tmpbdtr, TMR_BDT_IMOS, sBreakDeadTimeConfig->OffStateIDLEMode);
  MODIFY_REG(tmpbdtr, TMR_BDT_RMOS, sBreakDeadTimeConfig->OffStateRunMode);
  MODIFY_REG(tmpbdtr, TMR_BDT_BRKEN, sBreakDeadTimeConfig->BreakState);
  MODIFY_REG(tmpbdtr, TMR_BDT_BRKPOL, sBreakDeadTimeConfig->BreakPolarity);
  MODIFY_REG(tmpbdtr, TMR_BDT_AOEN, sBreakDeadTimeConfig->AutomaticOutput);


  /* Set TMRx_BDT */
  htmr->Instance->BDT = tmpbdtr;

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Configures the TMRx Remapping input capabilities.
  * @param  htmr TMR handle.
  * @param  Remap specifies the TMR remapping source.
  *         For TMR1, the parameter can have the following values:                   (**)
  *           @arg TMR_TMR1_TMR3_TRGO:  TMR1 ITR2 is connected to TMR3 TRGO
  *           @arg TMR_TMR1_LPTMR:      TMR1 ITR2 is connected to LPTMR1 output
  *
  *         For TMR2, the parameter can have the following values:                   (**)
  *           @arg TMR_TMR2_TMR8_TRGO:  TMR2 ITR1 is connected to TMR8 TRGO          (*)
  *           @arg TMR_TMR2_ETH_PTP:    TMR2 ITR1 is connected to PTP trigger output (*)
  *           @arg TMR_TMR2_USBFS_SOF:  TMR2 ITR1 is connected to OTG FS SOF
  *           @arg TMR_TMR2_USBHS_SOF:  TMR2 ITR1 is connected to OTG FS SOF
  *
  *         For TMR5, the parameter can have the following values:
  *           @arg TMR_TMR5_GPIO:       TMR5 TI4 is connected to GPIO
  *           @arg TMR_TMR5_LSI:        TMR5 TI4 is connected to LSI
  *           @arg TMR_TMR5_LSE:        TMR5 TI4 is connected to LSE
  *           @arg TMR_TMR5_RTC:        TMR5 TI4 is connected to the RTC wakeup interrupt
  *           @arg TMR_TMR5_TMR3_TRGO:  TMR5 ITR1 is connected to TMR3 TRGO          (*)
  *           @arg TMR_TMR5_LPTMR:      TMR5 ITR1 is connected to LPTMR1 output      (*)
  *
  *         For TMR9, the parameter can have the following values:                   (**)
  *           @arg TMR_TMR9_TMR3_TRGO:  TMR9 ITR1 is connected to TMR3 TRGO
  *           @arg TMR_TMR9_LPTMR:      TMR9 ITR1 is connected to LPTMR1 output
  *
  *         For TMR11, the parameter can have the following values:
  *           @arg TMR_TMR11_GPIO:     TMR11 TI1 is connected to GPIO
  *           @arg TMR_TMR11_HSE:      TMR11 TI1 is connected to HSE_RTC clock
  *           @arg TMR_TMR11_SPDIFRX:  TMR11 TI1 is connected to SPDIFRX_FRAME_SYNC  (*)
  *
  *         (*)  Value not defined in all devices. \n
  *         (**) Register not available in all devices.
  *
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMREx_RemapConfig(TMR_HandleTypeDef *htmr, uint32_t Remap)
{

  /* Check parameters */
  ASSERT_PARAM(IS_TMR_REMAP(htmr->Instance, Remap));

  __DAL_LOCK(htmr);

#if defined(LPTMR_OR_TMR1_ITR2_RMP) && defined(LPTMR_OR_TMR5_ITR1_RMP) && defined(LPTMR_OR_TMR9_ITR1_RMP)
  if ((Remap & LPTMR_REMAP_MASK) == LPTMR_REMAP_MASK)
  {
    /* Connect TMRx internal trigger to LPTMR1 output */
    __DAL_RCM_LPTMR1_CLK_ENABLE();
    MODIFY_REG(LPTMR1->OR,
               (LPTMR_OR_TMR1_ITR2_RMP | LPTMR_OR_TMR5_ITR1_RMP | LPTMR_OR_TMR9_ITR1_RMP),
               Remap & ~(LPTMR_REMAP_MASK));
  }
  else
  {
    /* Set the Timer remapping configuration */
    WRITE_REG(htmr->Instance->OR, Remap);
  }
#else
  /* Set the Timer remapping configuration */
  WRITE_REG(htmr->Instance->OR, Remap);
#endif /* LPTMR_OR_TMR1_ITR2_RMP &&  LPTMR_OR_TMR5_ITR1_RMP && LPTMR_OR_TMR9_ITR1_RMP */

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup TMREx_Exported_Functions_Group6 Extended Callbacks functions
  * @brief    Extended Callbacks functions
  *
@verbatim
  ==============================================================================
                    ##### Extended Callbacks functions #####
  ==============================================================================
  [..]
    This section provides Extended TMR callback functions:
    (+) Timer Commutation callback
    (+) Timer Break callback

@endverbatim
  * @{
  */

/**
  * @brief  Hall commutation changed callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMREx_CommutCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMREx_CommutCallback could be implemented in the user file
   */
}
/**
  * @brief  Hall commutation changed half complete callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMREx_CommutHalfCpltCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMREx_CommutHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Hall Break detection callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMREx_BreakCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMREx_BreakCallback could be implemented in the user file
   */
}
/**
  * @}
  */

/** @defgroup TMREx_Exported_Functions_Group7 Extended Peripheral State functions
  * @brief    Extended Peripheral State functions
  *
@verbatim
  ==============================================================================
                ##### Extended Peripheral State functions #####
  ==============================================================================
  [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the TMR Hall Sensor interface handle state.
  * @param  htmr TMR Hall Sensor handle
  * @retval DAL state
  */
DAL_TMR_StateTypeDef DAL_TMREx_HallSensor_GetState(TMR_HandleTypeDef *htmr)
{
  return htmr->State;
}

/**
  * @brief  Return actual state of the TMR complementary channel.
  * @param  htmr TMR handle
  * @param  ChannelN TMR Complementary channel
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1
  *            @arg TMR_CHANNEL_2: TMR Channel 2
  *            @arg TMR_CHANNEL_3: TMR Channel 3
  * @retval TMR Complementary channel state
  */
DAL_TMR_ChannelStateTypeDef DAL_TMREx_GetChannelNState(TMR_HandleTypeDef *htmr,  uint32_t ChannelN)
{
  DAL_TMR_ChannelStateTypeDef channel_state;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCXN_INSTANCE(htmr->Instance, ChannelN));

  channel_state = TMR_CHANNEL_N_STATE_GET(htmr, ChannelN);

  return channel_state;
}
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup TMREx_Private_Functions TMR Extended Private Functions
  * @{
  */

/**
  * @brief  TMR DMA Commutation callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TMREx_DMACommutationCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Change the htmr state */
  htmr->State = DAL_TMR_STATE_READY;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->CommutationCallback(htmr);
#else
  DAL_TMREx_CommutCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
}

/**
  * @brief  TMR DMA Commutation half complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TMREx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Change the htmr state */
  htmr->State = DAL_TMR_STATE_READY;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->CommutationHalfCpltCallback(htmr);
#else
  DAL_TMREx_CommutHalfCpltCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
}


/**
  * @brief  TMR DMA Delay Pulse complete callback (complementary channel).
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void TMR_DMADelayPulseNCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htmr->hdma[TMR_DMA_ID_CC1])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC2])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC3])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_3, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC4])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_4;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_4, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else
  {
    /* nothing to do */
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->PWM_PulseFinishedCallback(htmr);
#else
  DAL_TMR_PWM_PulseFinishedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TMR DMA error callback (complementary channel)
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void TMR_DMAErrorCCxN(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htmr->hdma[TMR_DMA_ID_CC1])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC2])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC3])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_3, DAL_TMR_CHANNEL_STATE_READY);
  }
  else
  {
    /* nothing to do */
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->ErrorCallback(htmr);
#else
  DAL_TMR_ErrorCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  Enables or disables the TMR Capture Compare Channel xN.
  * @param  TMRx to select the TMR peripheral
  * @param  Channel specifies the TMR Channel
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1
  *            @arg TMR_CHANNEL_2: TMR Channel 2
  *            @arg TMR_CHANNEL_3: TMR Channel 3
  * @param  ChannelNState specifies the TMR Channel CCxNE bit new state.
  *          This parameter can be: TMR_CCxN_ENABLE or TMR_CCxN_Disable.
  * @retval None
  */
static void TMR_CCxNChannelCmd(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ChannelNState)
{
  uint32_t tmp;

  tmp = TMR_CCEN_CC1NEN << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxNE Bit */
  TMRx->CCEN &=  ~tmp;

  /* Set or reset the CCxNE Bit */
  TMRx->CCEN |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}
/**
  * @}
  */

#endif /* DAL_TMR_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
