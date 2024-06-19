/**
  *
  * @file    apm32f4xx_dal_dac.c
  * @brief   DAC DAL module driver.
  *         This file provides firmware functions to manage the following
  *         functionalities of the Digital to Analog Converter (DAC) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Errors functions
  *
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
                      ##### DAC Peripheral features #####
  ==============================================================================
    [..]
      *** DAC Channels ***
      ====================
    [..]
    APM32F4 devices integrate two 12-bit Digital Analog Converters

    The 2 converters (i.e. channel1 & channel2)
    can be used independently or simultaneously (dual mode):
      (#) DAC channel1 with DAC_OUT1 (PA4) as output
      (#) DAC channel2 with DAC_OUT2 (PA5) as output

      *** DAC Triggers ***
      ====================
    [..]
    Digital to Analog conversion can be non-triggered using DAC_TRIGGER_NONE
    and DAC_OUT1/DAC_OUT2 is available once writing to DHRx register.
    [..]
    Digital to Analog conversion can be triggered by:
      (#) External event: EXTI Line 9 (any GPIOx_PIN_9) using DAC_TRIGGER_EXT_IT9.
          The used pin (GPIOx_PIN_9) must be configured in input mode.

      (#) Timers TRGO: TMR2, TMR4, TMR5, TMR6, TMR7 and TMR8
          (DAC_TRIGGER_T2_TRGO, DAC_TRIGGER_T4_TRGO...)

      (#) Software using DAC_TRIGGER_SOFTWARE

      *** DAC Buffer mode feature ***
      ===============================
      [..]
      Each DAC channel integrates an output buffer that can be used to
      reduce the output impedance, and to drive external loads directly
      without having to add an external operational amplifier.
      To enable, the output buffer use
      sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
      [..]
      (@) Refer to the device datasheet for more details about output
          impedance value with and without output buffer.

       *** DAC wave generation feature ***
       ===================================
       [..]
       Both DAC channels can be used to generate
         (#) Noise wave
         (#) Triangle wave

       *** DAC data format ***
       =======================
       [..]
       The DAC data format can be:
         (#) 8-bit right alignment using DAC_ALIGN_8B_R
         (#) 12-bit left alignment using DAC_ALIGN_12B_L
         (#) 12-bit right alignment using DAC_ALIGN_12B_R

       *** DAC data value to voltage correspondence ***
       ================================================
       [..]
       The analog output voltage on each DAC channel pin is determined
       by the following equation:
       [..]
       DAC_OUTx = VREF+ * DOR / 4095
       (+) with  DOR is the Data Output Register
       [..]
          VREF+ is the input voltage reference (refer to the device datasheet)
       [..]
        e.g. To set DAC_OUT1 to 0.7V, use
       (+) Assuming that VREF+ = 3.3V, DAC_OUT1 = (3.3 * 868) / 4095 = 0.7V

       *** DMA requests ***
       =====================
       [..]
       A DMA request can be generated when an external trigger (but not a software trigger)
       occurs if DMA1 requests are enabled using DAL_DAC_Start_DMA().
       DMA1 requests are mapped as following:
      (#) DAC channel1 mapped on DMA1 Stream5 channel7 which must be 
          already configured
      (#) DAC channel2 mapped on DMA1 Stream6 channel7 which must be 
          already configured

     [..]
    (@) For Dual mode and specific signal (Triangle and noise) generation please
        refer to Extended Features Driver description

                      ##### How to use this driver #####
  ==============================================================================
    [..]
      (+) DAC APB clock must be enabled to get write access to DAC
          registers using DAL_DAC_Init()
      (+) Configure DAC_OUTx (DAC_OUT1: PA4, DAC_OUT2: PA5) in analog mode.
      (+) Configure the DAC channel using DAL_DAC_ConfigChannel() function.
      (+) Enable the DAC channel using DAL_DAC_Start() or DAL_DAC_Start_DMA() functions.


     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Start the DAC peripheral using DAL_DAC_Start()
       (+) To read the DAC last data output value, use the DAL_DAC_GetValue() function.
       (+) Stop the DAC peripheral using DAL_DAC_Stop()

     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Start the DAC peripheral using DAL_DAC_Start_DMA(), at this stage the user specify the length
           of data to be transferred at each end of conversion
           First issued trigger will start the conversion of the value previously set by DAL_DAC_SetValue().
       (+) At the middle of data transfer DAL_DAC_ConvHalfCpltCallbackCh1() or DAL_DACEx_ConvHalfCpltCallbackCh2()
           function is executed and user can add his own code by customization of function pointer
           DAL_DAC_ConvHalfCpltCallbackCh1() or DAL_DACEx_ConvHalfCpltCallbackCh2()
       (+) At The end of data transfer DAL_DAC_ConvCpltCallbackCh1() or DAL_DACEx_ConvHalfCpltCallbackCh2()
           function is executed and user can add his own code by customization of function pointer
           DAL_DAC_ConvCpltCallbackCh1() or DAL_DACEx_ConvHalfCpltCallbackCh2()
       (+) In case of transfer Error, DAL_DAC_ErrorCallbackCh1() function is executed and user can
            add his own code by customization of function pointer DAL_DAC_ErrorCallbackCh1
       (+) In case of DMA underrun, DAC interruption triggers and execute internal function DAL_DAC_IRQHandler.
           DAL_DAC_DMAUnderrunCallbackCh1() or DAL_DACEx_DMAUnderrunCallbackCh2()
           function is executed and user can add his own code by customization of function pointer
           DAL_DAC_DMAUnderrunCallbackCh1() or DAL_DACEx_DMAUnderrunCallbackCh2() and
           add his own code by customization of function pointer DAL_DAC_ErrorCallbackCh1()
       (+) Stop the DAC peripheral using DAL_DAC_Stop_DMA()

    *** Callback registration ***
    =============================================
    [..]
      The compilation define  USE_DAL_DAC_REGISTER_CALLBACKS when set to 1
      allows the user to configure dynamically the driver callbacks.

    Use Functions DAL_DAC_RegisterCallback() to register a user callback,
      it allows to register following callbacks:
      (+) ConvCpltCallbackCh1     : callback when a half transfer is completed on Ch1.
      (+) ConvHalfCpltCallbackCh1 : callback when a transfer is completed on Ch1.
      (+) ErrorCallbackCh1        : callback when an error occurs on Ch1.
      (+) DMAUnderrunCallbackCh1  : callback when an underrun error occurs on Ch1.
      (+) ConvCpltCallbackCh2     : callback when a half transfer is completed on Ch2.
      (+) ConvHalfCpltCallbackCh2 : callback when a transfer is completed on Ch2.
      (+) ErrorCallbackCh2        : callback when an error occurs on Ch2.
      (+) DMAUnderrunCallbackCh2  : callback when an underrun error occurs on Ch2.
      (+) MspInitCallback         : DAC MspInit.
      (+) MspDeInitCallback       : DAC MspdeInit.
      This function takes as parameters the DAL peripheral handle, the Callback ID
      and a pointer to the user callback function.

    Use function DAL_DAC_UnRegisterCallback() to reset a callback to the default
      weak (surcharged) function. It allows to reset following callbacks:
      (+) ConvCpltCallbackCh1     : callback when a half transfer is completed on Ch1.
      (+) ConvHalfCpltCallbackCh1 : callback when a transfer is completed on Ch1.
      (+) ErrorCallbackCh1        : callback when an error occurs on Ch1.
      (+) DMAUnderrunCallbackCh1  : callback when an underrun error occurs on Ch1.
      (+) ConvCpltCallbackCh2     : callback when a half transfer is completed on Ch2.
      (+) ConvHalfCpltCallbackCh2 : callback when a transfer is completed on Ch2.
      (+) ErrorCallbackCh2        : callback when an error occurs on Ch2.
      (+) DMAUnderrunCallbackCh2  : callback when an underrun error occurs on Ch2.
      (+) MspInitCallback         : DAC MspInit.
      (+) MspDeInitCallback       : DAC MspdeInit.
      (+) All Callbacks
      This function) takes as parameters the DAL peripheral handle and the Callback ID.

      By default, after the DAL_DAC_Init and if the state is DAL_DAC_STATE_RESET
      all callbacks are reset to the corresponding legacy weak (surcharged) functions.
      Exception done for MspInit and MspDeInit callbacks that are respectively
      reset to the legacy weak (surcharged) functions in the DAL_DAC_Init
      and  DAL_DAC_DeInit only when these callbacks are null (not registered beforehand).
      If not, MspInit or MspDeInit are not null, the DAL_DAC_Init and DAL_DAC_DeInit
      keep and use the user MspInit/MspDeInit callbacks (registered beforehand)

      Callbacks can be registered/unregistered in READY state only.
      Exception done for MspInit/MspDeInit callbacks that can be registered/unregistered
      in READY or RESET state, thus registered (user) MspInit/DeInit callbacks can be used
      during the Init/DeInit.
      In that case first register the MspInit/MspDeInit user callbacks
      using DAL_DAC_RegisterCallback before calling DAL_DAC_DeInit
      or DAL_DAC_Init function.

      When The compilation define USE_DAL_DAC_REGISTER_CALLBACKS is set to 0 or
      not defined, the callback registering feature is not available
      and weak (surcharged) callbacks are used.

     *** DAC DAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in DAC DAL driver.

      (+) __DAL_DAC_ENABLE : Enable the DAC peripheral
      (+) __DAL_DAC_DISABLE : Disable the DAC peripheral
      (+) __DAL_DAC_CLEAR_FLAG: Clear the DAC's pending flags
      (+) __DAL_DAC_GET_FLAG: Get the selected DAC's flag status

     [..]
      (@) You can refer to the DAC DAL driver header file for more useful macros

@endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#ifdef DAL_DAC_MODULE_ENABLED
#if defined(DAC)

/** @defgroup DAC DAC
  * @brief DAC driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions -------------------------------------------------------*/

/** @defgroup DAC_Exported_Functions DAC Exported Functions
  * @{
  */

/** @defgroup DAC_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DAC.
      (+) De-initialize the DAC.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the DAC peripheral according to the specified parameters
  *         in the DAC_InitStruct and initialize the associated handle.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_Init(DAC_HandleTypeDef *hdac)
{
  /* Check DAC handle */
  if (hdac == NULL)
  {
    return DAL_ERROR;
  }
  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_ALL_INSTANCE(hdac->Instance));

  if (hdac->State == DAL_DAC_STATE_RESET)
  {
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
    /* Init the DAC Callback settings */
    hdac->ConvCpltCallbackCh1           = DAL_DAC_ConvCpltCallbackCh1;
    hdac->ConvHalfCpltCallbackCh1       = DAL_DAC_ConvHalfCpltCallbackCh1;
    hdac->ErrorCallbackCh1              = DAL_DAC_ErrorCallbackCh1;
    hdac->DMAUnderrunCallbackCh1        = DAL_DAC_DMAUnderrunCallbackCh1;
#if defined(DAC_CHANNEL2_SUPPORT)
    hdac->ConvCpltCallbackCh2           = DAL_DACEx_ConvCpltCallbackCh2;
    hdac->ConvHalfCpltCallbackCh2       = DAL_DACEx_ConvHalfCpltCallbackCh2;
    hdac->ErrorCallbackCh2              = DAL_DACEx_ErrorCallbackCh2;
    hdac->DMAUnderrunCallbackCh2        = DAL_DACEx_DMAUnderrunCallbackCh2;
#endif /* DAC_CHANNEL2_SUPPORT */
    if (hdac->MspInitCallback == NULL)
    {
      hdac->MspInitCallback             = DAL_DAC_MspInit;
    }
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

    /* Allocate lock resource and initialize it */
    hdac->Lock = DAL_UNLOCKED;

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
    /* Init the low level hardware */
    hdac->MspInitCallback(hdac);
#else
    /* Init the low level hardware */
    DAL_DAC_MspInit(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */
  }

  /* Initialize the DAC state*/
  hdac->State = DAL_DAC_STATE_BUSY;

  /* Set DAC error code to none */
  hdac->ErrorCode = DAL_DAC_ERROR_NONE;

  /* Initialize the DAC state*/
  hdac->State = DAL_DAC_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Deinitialize the DAC peripheral registers to their default reset values.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_DeInit(DAC_HandleTypeDef *hdac)
{
  /* Check DAC handle */
  if (hdac == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_ALL_INSTANCE(hdac->Instance));

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_BUSY;

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  if (hdac->MspDeInitCallback == NULL)
  {
    hdac->MspDeInitCallback = DAL_DAC_MspDeInit;
  }
  /* DeInit the low level hardware */
  hdac->MspDeInitCallback(hdac);
#else
  /* DeInit the low level hardware */
  DAL_DAC_MspDeInit(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

  /* Set DAC error code to none */
  hdac->ErrorCode = DAL_DAC_ERROR_NONE;

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hdac);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Initialize the DAC MSP.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DAC_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the DAC MSP.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DAC_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup DAC_Exported_Functions_Group2 IO operation functions
  *  @brief    IO operation functions
  *
@verbatim
  ==============================================================================
             ##### IO operation functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Start conversion.
      (+) Stop conversion.
      (+) Start conversion and enable DMA transfer.
      (+) Stop conversion and disable DMA transfer.
      (+) Get result of conversion.

@endverbatim
  * @{
  */

/**
  * @brief  Enables DAC and starts conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));

  /* Process locked */
  __DAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_BUSY;

  /* Enable the Peripheral */
  __DAL_DAC_ENABLE(hdac, Channel);

  if (Channel == DAC_CHANNEL_1)
  {
    /* Check if software trigger enabled */
    if ((hdac->Instance->CTRL & (DAC_CTRL_TRGENCH1 | DAC_CTRL_TRGSELCH1)) == DAC_TRIGGER_SOFTWARE)
    {
      /* Enable the selected DAC software conversion */
      SET_BIT(hdac->Instance->SWTRG, DAC_SWTRG_SWTRG1);
    }
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  else
  {
    /* Check if software trigger enabled */
    if ((hdac->Instance->CTRL & (DAC_CTRL_TRGENCH2 | DAC_CTRL_TRGSELCH2)) == (DAC_TRIGGER_SOFTWARE << (Channel & 0x10UL)))
    {
      /* Enable the selected DAC software conversion*/
      SET_BIT(hdac->Instance->SWTRG, DAC_SWTRG_SWTRG2);
    }
  }
#endif /* DAC_CHANNEL2_SUPPORT */

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Process unlocked */
  __DAL_UNLOCK(hdac);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Disables DAC and stop conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));

  /* Disable the Peripheral */
  __DAL_DAC_DISABLE(hdac, Channel);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Enables DAC and starts conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to DAC peripheral
  * @param  Alignment Specifies the data alignment for DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));
  ASSERT_PARAM(IS_DAC_ALIGN(Alignment));

  /* Process locked */
  __DAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_BUSY;

  if (Channel == DAC_CHANNEL_1)
  {
    /* Set the DMA transfer complete callback for channel1 */
    hdac->DMA_Handle1->XferCpltCallback = DAC_DMAConvCpltCh1;

    /* Set the DMA half transfer complete callback for channel1 */
    hdac->DMA_Handle1->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh1;

    /* Set the DMA error callback for channel1 */
    hdac->DMA_Handle1->XferErrorCallback = DAC_DMAErrorCh1;

    /* Enable the selected DAC channel1 DMA request */
    SET_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAENCH1);

    /* Case of use of channel 1 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* Get DHR12R1 address */
        tmpreg = (uint32_t)&hdac->Instance->DH12R1;
        break;
      case DAC_ALIGN_12B_L:
        /* Get DHR12L1 address */
        tmpreg = (uint32_t)&hdac->Instance->DH12L1;
        break;
      case DAC_ALIGN_8B_R:
        /* Get DHR8R1 address */
        tmpreg = (uint32_t)&hdac->Instance->DH8R1;
        break;
      default:
        break;
    }
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  else
  {
    /* Set the DMA transfer complete callback for channel2 */
    hdac->DMA_Handle2->XferCpltCallback = DAC_DMAConvCpltCh2;

    /* Set the DMA half transfer complete callback for channel2 */
    hdac->DMA_Handle2->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh2;

    /* Set the DMA error callback for channel2 */
    hdac->DMA_Handle2->XferErrorCallback = DAC_DMAErrorCh2;

    /* Enable the selected DAC channel2 DMA request */
    SET_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAENCH2);

    /* Case of use of channel 2 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* Get DHR12R2 address */
        tmpreg = (uint32_t)&hdac->Instance->DH12R2;
        break;
      case DAC_ALIGN_12B_L:
        /* Get DHR12L2 address */
        tmpreg = (uint32_t)&hdac->Instance->DH12L2;
        break;
      case DAC_ALIGN_8B_R:
        /* Get DHR8R2 address */
        tmpreg = (uint32_t)&hdac->Instance->DH8R2;
        break;
      default:
        break;
    }
  }
#endif /* DAC_CHANNEL2_SUPPORT */

  /* Enable the DMA Stream */
  if (Channel == DAC_CHANNEL_1)
  {
    /* Enable the DAC DMA underrun interrupt */
    __DAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR1);

    /* Enable the DMA Stream */
    status = DAL_DMA_Start_IT(hdac->DMA_Handle1, (uint32_t)pData, tmpreg, Length);
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  else
  {
    /* Enable the DAC DMA underrun interrupt */
    __DAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR2);

    /* Enable the DMA Stream */
    status = DAL_DMA_Start_IT(hdac->DMA_Handle2, (uint32_t)pData, tmpreg, Length);
  }
#endif /* DAC_CHANNEL2_SUPPORT */

  /* Process Unlocked */
  __DAL_UNLOCK(hdac);

  if (status == DAL_OK)
  {
    /* Enable the Peripheral */
    __DAL_DAC_ENABLE(hdac, Channel);
  }
  else
  {
    hdac->ErrorCode |= DAL_DAC_ERROR_DMA;
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Disables DAC and stop conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));

  /* Disable the selected DAC channel DMA request */
  hdac->Instance->CTRL &= ~(DAC_CTRL_DMAENCH1 << (Channel & 0x10UL));

  /* Disable the Peripheral */
  __DAL_DAC_DISABLE(hdac, Channel);

  /* Disable the DMA Stream */

  /* Channel1 is used */
  if (Channel == DAC_CHANNEL_1)
  {
    /* Disable the DMA Stream */
    (void)DAL_DMA_Abort(hdac->DMA_Handle1);

    /* Disable the DAC DMA underrun interrupt */
    __DAL_DAC_DISABLE_IT(hdac, DAC_IT_DMAUDR1);
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  else /* Channel2 is used for */
  {
    /* Disable the DMA Stream */
    (void)DAL_DMA_Abort(hdac->DMA_Handle2);

    /* Disable the DAC DMA underrun interrupt */
    __DAL_DAC_DISABLE_IT(hdac, DAC_IT_DMAUDR2);
  }
#endif /* DAC_CHANNEL2_SUPPORT */

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Handles DAC interrupt request
  *         This function uses the interruption of DMA
  *         underrun.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void DAL_DAC_IRQHandler(DAC_HandleTypeDef *hdac)
{
  if (__DAL_DAC_GET_IT_SOURCE(hdac, DAC_IT_DMAUDR1))
  {
    /* Check underrun flag of DAC channel 1 */
    if (__DAL_DAC_GET_FLAG(hdac, DAC_FLAG_DMAUDR1))
    {
      /* Change DAC state to error state */
      hdac->State = DAL_DAC_STATE_ERROR;

      /* Set DAC error code to channel1 DMA underrun error */
      SET_BIT(hdac->ErrorCode, DAL_DAC_ERROR_DMAUNDERRUNCH1);

      /* Clear the underrun flag */
      __DAL_DAC_CLEAR_FLAG(hdac, DAC_FLAG_DMAUDR1);

      /* Disable the selected DAC channel1 DMA request */
      CLEAR_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAENCH1);

      /* Error callback */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
      hdac->DMAUnderrunCallbackCh1(hdac);
#else
      DAL_DAC_DMAUnderrunCallbackCh1(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */
    }
  }

#if defined(DAC_CHANNEL2_SUPPORT)
  if (__DAL_DAC_GET_IT_SOURCE(hdac, DAC_IT_DMAUDR2))
  {
    /* Check underrun flag of DAC channel 2 */
    if (__DAL_DAC_GET_FLAG(hdac, DAC_FLAG_DMAUDR2))
    {
      /* Change DAC state to error state */
      hdac->State = DAL_DAC_STATE_ERROR;

      /* Set DAC error code to channel2 DMA underrun error */
      SET_BIT(hdac->ErrorCode, DAL_DAC_ERROR_DMAUNDERRUNCH2);

      /* Clear the underrun flag */
      __DAL_DAC_CLEAR_FLAG(hdac, DAC_FLAG_DMAUDR2);

      /* Disable the selected DAC channel2 DMA request */
      CLEAR_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAENCH2);

      /* Error callback */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
      hdac->DMAUnderrunCallbackCh2(hdac);
#else
      DAL_DACEx_DMAUnderrunCallbackCh2(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */
    }
  }
#endif /* DAC_CHANNEL2_SUPPORT */
}

/**
  * @brief  Set the specified data holding register value for DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  Alignment Specifies the data alignment.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data Data to be loaded in the selected data holding register.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data)
{
  __IO uint32_t tmp = 0UL;

  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));
  ASSERT_PARAM(IS_DAC_ALIGN(Alignment));
  ASSERT_PARAM(IS_DAC_DATA(Data));

  tmp = (uint32_t)hdac->Instance;
  if (Channel == DAC_CHANNEL_1)
  {
    tmp += DAC_DH12R1_ALIGNMENT(Alignment);
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  else
  {
    tmp += DAC_DH12R2_ALIGNMENT(Alignment);
  }
#endif /* DAC_CHANNEL2_SUPPORT */

  /* Set the DAC channel selected data holding register */
  *(__IO uint32_t *) tmp = Data;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Conversion complete callback in non-blocking mode for Channel1
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DAC_ConvCpltCallbackCh1 could be implemented in the user file
   */
}

/**
  * @brief  Conversion half DMA transfer callback in non-blocking mode for Channel1
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DAC_ConvHalfCpltCallbackCh1 could be implemented in the user file
   */
}

/**
  * @brief  Error DAC callback for Channel1.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DAC_ErrorCallbackCh1 could be implemented in the user file
   */
}

/**
  * @brief  DMA underrun DAC callback for channel1.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DAC_DMAUnderrunCallbackCh1 could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup DAC_Exported_Functions_Group3 Peripheral Control functions
  *  @brief    Peripheral Control functions
  *
@verbatim
  ==============================================================================
             ##### Peripheral Control functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure channels.
      (+) Set the specified data holding register value for DAC channel.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the last data output value of the selected DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval The selected DAC channel data output value.
  */
uint32_t DAL_DAC_GetValue(DAC_HandleTypeDef *hdac, uint32_t Channel)
{
  uint32_t result = 0;

  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));

  if (Channel == DAC_CHANNEL_1)
  {
    result = hdac->Instance->DATAOCH1;
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  else
  {
    result = hdac->Instance->DATAOCH2;
  }
#endif /* DAC_CHANNEL2_SUPPORT */
  /* Returns the DAC channel data output register value */
  return result;
}

/**
  * @brief  Configures the selected DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  sConfig DAC configuration structure.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac, DAC_ChannelConfTypeDef *sConfig, uint32_t Channel)
{
  uint32_t tmpreg1;
  uint32_t tmpreg2;

  /* Check the DAC parameters */
  ASSERT_PARAM(IS_DAC_TRIGGER(sConfig->DAC_Trigger));
  ASSERT_PARAM(IS_DAC_OUTPUT_BUFFER_STATE(sConfig->DAC_OutputBuffer));
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));

  /* Process locked */
  __DAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_BUSY;

  /* Get the DAC CTRL value */
  tmpreg1 = hdac->Instance->CTRL;
  /* Clear BUFFDx, TRGENx, TRGSELx, WAVENx and MAMPSELx bits */
  tmpreg1 &= ~(((uint32_t)(DAC_CTRL_MAMPSELCH1 | DAC_CTRL_WAVENCH1 | DAC_CTRL_TRGSELCH1 | DAC_CTRL_TRGENCH1 | DAC_CTRL_BUFFDCH1)) << (Channel & 0x10UL));
  /* Configure for the selected DAC channel: buffer output, trigger */
  /* Set TRGSELx and TRGENx bits according to DAC_Trigger value */
  /* Set BUFFDx bit according to DAC_OutputBuffer value */
  tmpreg2 = (sConfig->DAC_Trigger | sConfig->DAC_OutputBuffer);
  /* Calculate CTRL register value depending on DAC_Channel */
  tmpreg1 |= tmpreg2 << (Channel & 0x10UL);
  /* Write to DAC CTRL */
  hdac->Instance->CTRL = tmpreg1;
  /* Disable wave generation */
  CLEAR_BIT(hdac->Instance->CTRL, (DAC_CTRL_WAVENCH1 << (Channel & 0x10UL)));

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Process unlocked */
  __DAL_UNLOCK(hdac);

  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup DAC_Exported_Functions_Group4 Peripheral State and Errors functions
  *  @brief   Peripheral State and Errors functions
  *
@verbatim
  ==============================================================================
            ##### Peripheral State and Errors functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DAC state.
      (+) Check the DAC Errors.

@endverbatim
  * @{
  */

/**
  * @brief  return the DAC handle state
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval DAL state
  */
DAL_DAC_StateTypeDef DAL_DAC_GetState(DAC_HandleTypeDef *hdac)
{
  /* Return DAC handle state */
  return hdac->State;
}


/**
  * @brief  Return the DAC error code
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval DAC Error Code
  */
uint32_t DAL_DAC_GetError(DAC_HandleTypeDef *hdac)
{
  return hdac->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions
  * @{
  */

/** @addtogroup DAC_Exported_Functions_Group1
  * @{
  */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User DAC Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param  hdac DAC handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_DAC_ERROR_INVALID_CALLBACK   DAC Error Callback ID
  *          @arg @ref DAL_DAC_CH1_COMPLETE_CB_ID       DAC CH1 Complete Callback ID
  *          @arg @ref DAL_DAC_CH1_HALF_COMPLETE_CB_ID  DAC CH1 Half Complete Callback ID
  *          @arg @ref DAL_DAC_CH1_ERROR_ID             DAC CH1 Error Callback ID
  *          @arg @ref DAL_DAC_CH1_UNDERRUN_CB_ID       DAC CH1 UnderRun Callback ID
  *          @arg @ref DAL_DAC_CH2_COMPLETE_CB_ID       DAC CH2 Complete Callback ID
  *          @arg @ref DAL_DAC_CH2_HALF_COMPLETE_CB_ID  DAC CH2 Half Complete Callback ID
  *          @arg @ref DAL_DAC_CH2_ERROR_ID             DAC CH2 Error Callback ID
  *          @arg @ref DAL_DAC_CH2_UNDERRUN_CB_ID       DAC CH2 UnderRun Callback ID
  *          @arg @ref DAL_DAC_MSPINIT_CB_ID            DAC MSP Init Callback ID
  *          @arg @ref DAL_DAC_MSPDEINIT_CB_ID          DAC MSP DeInit Callback ID
  *
  * @param  pCallback pointer to the Callback function
  * @retval status
  */
DAL_StatusTypeDef DAL_DAC_RegisterCallback(DAC_HandleTypeDef *hdac, DAL_DAC_CallbackIDTypeDef CallbackID,
                                           pDAC_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hdac->ErrorCode |= DAL_DAC_ERROR_INVALID_CALLBACK;
    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hdac);

  if (hdac->State == DAL_DAC_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_DAC_CH1_COMPLETE_CB_ID :
        hdac->ConvCpltCallbackCh1 = pCallback;
        break;
      case DAL_DAC_CH1_HALF_COMPLETE_CB_ID :
        hdac->ConvHalfCpltCallbackCh1 = pCallback;
        break;
      case DAL_DAC_CH1_ERROR_ID :
        hdac->ErrorCallbackCh1 = pCallback;
        break;
      case DAL_DAC_CH1_UNDERRUN_CB_ID :
        hdac->DMAUnderrunCallbackCh1 = pCallback;
        break;
#if defined(DAC_CHANNEL2_SUPPORT)
      case DAL_DAC_CH2_COMPLETE_CB_ID :
        hdac->ConvCpltCallbackCh2 = pCallback;
        break;
      case DAL_DAC_CH2_HALF_COMPLETE_CB_ID :
        hdac->ConvHalfCpltCallbackCh2 = pCallback;
        break;
      case DAL_DAC_CH2_ERROR_ID :
        hdac->ErrorCallbackCh2 = pCallback;
        break;
      case DAL_DAC_CH2_UNDERRUN_CB_ID :
        hdac->DMAUnderrunCallbackCh2 = pCallback;
        break;
#endif /* DAC_CHANNEL2_SUPPORT */
      case DAL_DAC_MSPINIT_CB_ID :
        hdac->MspInitCallback = pCallback;
        break;
      case DAL_DAC_MSPDEINIT_CB_ID :
        hdac->MspDeInitCallback = pCallback;
        break;
      default :
        /* Update the error code */
        hdac->ErrorCode |= DAL_DAC_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hdac->State == DAL_DAC_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_DAC_MSPINIT_CB_ID :
        hdac->MspInitCallback = pCallback;
        break;
      case DAL_DAC_MSPDEINIT_CB_ID :
        hdac->MspDeInitCallback = pCallback;
        break;
      default :
        /* Update the error code */
        hdac->ErrorCode |= DAL_DAC_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hdac->ErrorCode |= DAL_DAC_ERROR_INVALID_CALLBACK;
    /* update return status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hdac);
  return status;
}

/**
  * @brief  Unregister a User DAC Callback
  *         DAC Callback is redirected to the weak (surcharged) predefined callback
  * @param  hdac DAC handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_DAC_CH1_COMPLETE_CB_ID          DAC CH1 transfer Complete Callback ID
  *          @arg @ref DAL_DAC_CH1_HALF_COMPLETE_CB_ID     DAC CH1 Half Complete Callback ID
  *          @arg @ref DAL_DAC_CH1_ERROR_ID                DAC CH1 Error Callback ID
  *          @arg @ref DAL_DAC_CH1_UNDERRUN_CB_ID          DAC CH1 UnderRun Callback ID
  *          @arg @ref DAL_DAC_CH2_COMPLETE_CB_ID          DAC CH2 Complete Callback ID
  *          @arg @ref DAL_DAC_CH2_HALF_COMPLETE_CB_ID     DAC CH2 Half Complete Callback ID
  *          @arg @ref DAL_DAC_CH2_ERROR_ID                DAC CH2 Error Callback ID
  *          @arg @ref DAL_DAC_CH2_UNDERRUN_CB_ID          DAC CH2 UnderRun Callback ID
  *          @arg @ref DAL_DAC_MSPINIT_CB_ID               DAC MSP Init Callback ID
  *          @arg @ref DAL_DAC_MSPDEINIT_CB_ID             DAC MSP DeInit Callback ID
  *          @arg @ref DAL_DAC_ALL_CB_ID                   DAC All callbacks
  * @retval status
  */
DAL_StatusTypeDef DAL_DAC_UnRegisterCallback(DAC_HandleTypeDef *hdac, DAL_DAC_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hdac);

  if (hdac->State == DAL_DAC_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_DAC_CH1_COMPLETE_CB_ID :
        hdac->ConvCpltCallbackCh1 = DAL_DAC_ConvCpltCallbackCh1;
        break;
      case DAL_DAC_CH1_HALF_COMPLETE_CB_ID :
        hdac->ConvHalfCpltCallbackCh1 = DAL_DAC_ConvHalfCpltCallbackCh1;
        break;
      case DAL_DAC_CH1_ERROR_ID :
        hdac->ErrorCallbackCh1 = DAL_DAC_ErrorCallbackCh1;
        break;
      case DAL_DAC_CH1_UNDERRUN_CB_ID :
        hdac->DMAUnderrunCallbackCh1 = DAL_DAC_DMAUnderrunCallbackCh1;
        break;
#if defined(DAC_CHANNEL2_SUPPORT)
      case DAL_DAC_CH2_COMPLETE_CB_ID :
        hdac->ConvCpltCallbackCh2 = DAL_DACEx_ConvCpltCallbackCh2;
        break;
      case DAL_DAC_CH2_HALF_COMPLETE_CB_ID :
        hdac->ConvHalfCpltCallbackCh2 = DAL_DACEx_ConvHalfCpltCallbackCh2;
        break;
      case DAL_DAC_CH2_ERROR_ID :
        hdac->ErrorCallbackCh2 = DAL_DACEx_ErrorCallbackCh2;
        break;
      case DAL_DAC_CH2_UNDERRUN_CB_ID :
        hdac->DMAUnderrunCallbackCh2 = DAL_DACEx_DMAUnderrunCallbackCh2;
        break;
#endif /* DAC_CHANNEL2_SUPPORT */
      case DAL_DAC_MSPINIT_CB_ID :
        hdac->MspInitCallback = DAL_DAC_MspInit;
        break;
      case DAL_DAC_MSPDEINIT_CB_ID :
        hdac->MspDeInitCallback = DAL_DAC_MspDeInit;
        break;
      case DAL_DAC_ALL_CB_ID :
        hdac->ConvCpltCallbackCh1 = DAL_DAC_ConvCpltCallbackCh1;
        hdac->ConvHalfCpltCallbackCh1 = DAL_DAC_ConvHalfCpltCallbackCh1;
        hdac->ErrorCallbackCh1 = DAL_DAC_ErrorCallbackCh1;
        hdac->DMAUnderrunCallbackCh1 = DAL_DAC_DMAUnderrunCallbackCh1;
#if defined(DAC_CHANNEL2_SUPPORT)
        hdac->ConvCpltCallbackCh2 = DAL_DACEx_ConvCpltCallbackCh2;
        hdac->ConvHalfCpltCallbackCh2 = DAL_DACEx_ConvHalfCpltCallbackCh2;
        hdac->ErrorCallbackCh2 = DAL_DACEx_ErrorCallbackCh2;
        hdac->DMAUnderrunCallbackCh2 = DAL_DACEx_DMAUnderrunCallbackCh2;
#endif /* DAC_CHANNEL2_SUPPORT */
        hdac->MspInitCallback = DAL_DAC_MspInit;
        hdac->MspDeInitCallback = DAL_DAC_MspDeInit;
        break;
      default :
        /* Update the error code */
        hdac->ErrorCode |= DAL_DAC_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hdac->State == DAL_DAC_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_DAC_MSPINIT_CB_ID :
        hdac->MspInitCallback = DAL_DAC_MspInit;
        break;
      case DAL_DAC_MSPDEINIT_CB_ID :
        hdac->MspDeInitCallback = DAL_DAC_MspDeInit;
        break;
      default :
        /* Update the error code */
        hdac->ErrorCode |= DAL_DAC_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hdac->ErrorCode |= DAL_DAC_ERROR_INVALID_CALLBACK;
    /* update return status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hdac);
  return status;
}
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DAC_Private_Functions
  * @{
  */

/**
  * @brief  DMA conversion complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ConvCpltCallbackCh1(hdac);
#else
  DAL_DAC_ConvCpltCallbackCh1(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

  hdac->State = DAL_DAC_STATE_READY;
}

/**
  * @brief  DMA half transfer complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAHalfConvCpltCh1(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* Conversion complete callback */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ConvHalfCpltCallbackCh1(hdac);
#else
  DAL_DAC_ConvHalfCpltCallbackCh1(hdac);
#endif  /* USE_DAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAErrorCh1(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Set DAC error code to DMA error */
  hdac->ErrorCode |= DAL_DAC_ERROR_DMA;

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ErrorCallbackCh1(hdac);
#else
  DAL_DAC_ErrorCallbackCh1(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

  hdac->State = DAL_DAC_STATE_READY;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAC */

#endif /* DAL_DAC_MODULE_ENABLED */

/**
  * @}
  */

