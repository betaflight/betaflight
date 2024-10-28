/**
  *
  * @file    apm32f4xx_dal_adc_ex.c
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the ADC extension peripheral:
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
    (#)Initialize the ADC low level resources by implementing the DAL_ADC_MspInit():
       (##) Enable the ADC interface clock using __DAL_RCM_ADC_CLK_ENABLE()
       (##) ADC pins configuration
             (+++) Enable the clock for the ADC GPIOs using the following function:
                   __DAL_RCM_GPIOx_CLK_ENABLE()
             (+++) Configure these ADC pins in analog mode using DAL_GPIO_Init()
       (##) In case of using interrupts (e.g. DAL_ADC_Start_IT())
             (+++) Configure the ADC interrupt priority using DAL_NVIC_SetPriority()
             (+++) Enable the ADC IRQ handler using DAL_NVIC_EnableIRQ()
             (+++) In ADC IRQ handler, call DAL_ADC_IRQHandler()
      (##) In case of using DMA to control data transfer (e.g. DAL_ADC_Start_DMA())
             (+++) Enable the DMAx interface clock using __DAL_RCM_DMAx_CLK_ENABLE()
             (+++) Configure and enable two DMA streams stream for managing data
                 transfer from peripheral to memory (output stream)
             (+++) Associate the initialized DMA handle to the ADC DMA handle
                 using  __DAL_LINKDMA()
             (+++) Configure the priority and enable the NVIC for the transfer complete
                 interrupt on the two DMA Streams. The output stream should have higher
                 priority than the input stream.
     (#) Configure the ADC Prescaler, conversion resolution and data alignment
         using the DAL_ADC_Init() function.

     (#) Configure the ADC Injected channels group features, use DAL_ADC_Init()
         and DAL_ADC_ConfigChannel() functions.

     (#) Three operation modes are available within this driver:

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Start the ADC peripheral using DAL_ADCEx_InjectedStart()
       (+) Wait for end of conversion using DAL_ADC_PollForConversion(), at this stage
           user can specify the value of timeout according to his end application
       (+) To read the ADC converted values, use the DAL_ADCEx_InjectedGetValue() function.
       (+) Stop the ADC peripheral using DAL_ADCEx_InjectedStop()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Start the ADC peripheral using DAL_ADCEx_InjectedStart_IT()
       (+) Use DAL_ADC_IRQHandler() called under ADC_IRQHandler() Interrupt subroutine
       (+) At ADC end of conversion DAL_ADCEx_InjectedConvCpltCallback() function is executed and user can
            add his own code by customization of function pointer DAL_ADCEx_InjectedConvCpltCallback 
       (+) In case of ADC Error, DAL_ADCEx_InjectedErrorCallback() function is executed and user can 
            add his own code by customization of function pointer DAL_ADCEx_InjectedErrorCallback
       (+) Stop the ADC peripheral using DAL_ADCEx_InjectedStop_IT()

     *** Multi mode ADCs Regular channels configuration ***
     ======================================================
     [..]
       (+) Select the Multi mode ADC regular channels features (dual or triple mode)
          and configure the DMA mode using DAL_ADCEx_MultiModeConfigChannel() functions.
       (+) Start the ADC peripheral using DAL_ADCEx_MultiModeStart_DMA(), at this stage the user specify the length
           of data to be transferred at each end of conversion
       (+) Read the ADCs converted values using the DAL_ADCEx_MultiModeGetValue() function.


    @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup ADCEx ADCEx
  * @brief ADC Extended driver modules
  * @{
  */ 

#ifdef DAL_ADC_MODULE_ENABLED
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup ADCEx_Private_Functions
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
#if defined(ADC_MULTIMODE_SUPPORT)
static void ADC_MultiModeDMAConvCplt(DMA_HandleTypeDef *hdma);
static void ADC_MultiModeDMAError(DMA_HandleTypeDef *hdma);
static void ADC_MultiModeDMAHalfConvCplt(DMA_HandleTypeDef *hdma);
#endif /* ADC_MULTIMODE_SUPPORT */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup ADCEx_Exported_Functions ADC Exported Functions
  * @{
  */

/** @defgroup ADCEx_Exported_Functions_Group1  Extended features functions 
  *  @brief    Extended features functions  
  *
@verbatim   
 ===============================================================================
                 ##### Extended features functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Start conversion of injected channel.
      (+) Stop conversion of injected channel.
      (+) Start multimode and enable DMA transfer.
      (+) Stop multimode and disable DMA transfer.
      (+) Get result of injected channel conversion.
      (+) Get result of multimode conversion.
      (+) Configure injected channels.
      (+) Configure multimode.
               
@endverbatim
  * @{
  */

/**
  * @brief  Enables the selected ADC software start conversion of the injected channels.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc)
{
  __IO uint32_t counter = 0U;
  uint32_t tmp1 = 0U, tmp2 = 0U;
  ADC_Common_TypeDef *tmpADC_Common;
  
  /* Process locked */
  __DAL_LOCK(hadc);
  
  /* Enable the ADC peripheral */
  
  /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
  if((hadc->Instance->CTRL2 & ADC_CTRL2_ADCEN) != ADC_CTRL2_ADCEN)
  {  
    /* Enable the Peripheral */
    __DAL_ADC_ENABLE(hadc);
    
    /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while(counter != 0U)
    {
      counter--;
    }
  }
  
  /* Start conversion if ADC is effectively enabled */
  if(DAL_IS_BIT_SET(hadc->Instance->CTRL2, ADC_CTRL2_ADCEN))
  {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to injected group conversion results    */
    /* - Set state bitfield related to injected operation                     */
    ADC_STATE_CLR_SET(hadc->State,
                      DAL_ADC_STATE_READY | DAL_ADC_STATE_INJ_EOC,
                      DAL_ADC_STATE_INJ_BUSY);
    
    /* Check if a regular conversion is ongoing */
    /* Note: On this device, there is no ADC error code fields related to     */
    /*       conversions on group injected only. In case of conversion on     */
    /*       going on group regular, no error code is reset.                  */
    if (DAL_IS_BIT_CLR(hadc->State, DAL_ADC_STATE_REG_BUSY))
    {
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE(hadc);
    }
    
    /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
    __DAL_UNLOCK(hadc);
    
    /* Clear injected group conversion flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __DAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);

    /* Pointer to the common control register to which is belonging hadc    */
    /* (Depending on APM32F4 product, there may be up to 3 ADC and 1 common */
    /* control register)                                                    */
    tmpADC_Common = ADC_COMMON_REGISTER(hadc);

    /* Check if Multimode enabled */
    if(DAL_IS_BIT_CLR(tmpADC_Common->CCTRL, ADC_CCTRL_ADCMSEL))
    {
      tmp1 = DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_INJEXTTRGEN);
      tmp2 = DAL_IS_BIT_CLR(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN);
      if(tmp1 && tmp2)
      {
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CTRL2 |= ADC_CTRL2_INJSWSC;
      }
    }
    else
    {
      tmp1 = DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_INJEXTTRGEN);
      tmp2 = DAL_IS_BIT_CLR(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN);
      if(((hadc->Instance == ADC1) || (hadc->Instance == ADC2)) && tmp1 && tmp2)  
      {
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CTRL2 |= ADC_CTRL2_INJSWSC;
      }
    }
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, DAL_ADC_STATE_ERROR_INTERNAL);

    /* Set ADC error code to ADC IP internal error */
    SET_BIT(hadc->ErrorCode, DAL_ADC_ERROR_INTERNAL);
  }
  
  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Enables the interrupt and starts ADC conversion of injected channels.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  *
  * @retval DAL status.
  */
DAL_StatusTypeDef DAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc)
{
  __IO uint32_t counter = 0U;
  uint32_t tmp1 = 0U, tmp2 = 0U;
  ADC_Common_TypeDef *tmpADC_Common;
  
  /* Process locked */
  __DAL_LOCK(hadc);
  
  /* Enable the ADC peripheral */
  
  /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
  if((hadc->Instance->CTRL2 & ADC_CTRL2_ADCEN) != ADC_CTRL2_ADCEN)
  {  
    /* Enable the Peripheral */
    __DAL_ADC_ENABLE(hadc);
    
    /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while(counter != 0U)
    {
      counter--;
    }
  }
  
  /* Start conversion if ADC is effectively enabled */
  if(DAL_IS_BIT_SET(hadc->Instance->CTRL2, ADC_CTRL2_ADCEN))
  {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to injected group conversion results    */
    /* - Set state bitfield related to injected operation                     */
    ADC_STATE_CLR_SET(hadc->State,
                      DAL_ADC_STATE_READY | DAL_ADC_STATE_INJ_EOC,
                      DAL_ADC_STATE_INJ_BUSY);
    
    /* Check if a regular conversion is ongoing */
    /* Note: On this device, there is no ADC error code fields related to     */
    /*       conversions on group injected only. In case of conversion on     */
    /*       going on group regular, no error code is reset.                  */
    if (DAL_IS_BIT_CLR(hadc->State, DAL_ADC_STATE_REG_BUSY))
    {
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE(hadc);
    }
    
    /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
    __DAL_UNLOCK(hadc);
    
    /* Clear injected group conversion flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __DAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);
    
    /* Enable end of conversion interrupt for injected channels */
    __DAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);

    /* Pointer to the common control register to which is belonging hadc    */
    /* (Depending on APM32F4 product, there may be up to 3 ADC and 1 common */
    /* control register)                                                    */
    tmpADC_Common = ADC_COMMON_REGISTER(hadc);
    
    /* Check if Multimode enabled */
    if(DAL_IS_BIT_CLR(tmpADC_Common->CCTRL, ADC_CCTRL_ADCMSEL))
    {
      tmp1 = DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_INJEXTTRGEN);
      tmp2 = DAL_IS_BIT_CLR(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN);
      if(tmp1 && tmp2)
      {
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CTRL2 |= ADC_CTRL2_INJSWSC;
      }
    }
    else
    {
      tmp1 = DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_INJEXTTRGEN);
      tmp2 = DAL_IS_BIT_CLR(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN);
      if(((hadc->Instance == ADC1) || (hadc->Instance == ADC2)) && tmp1 && tmp2)  
      {
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CTRL2 |= ADC_CTRL2_INJSWSC;
      }
    }
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, DAL_ADC_STATE_ERROR_INTERNAL);

    /* Set ADC error code to ADC IP internal error */
    SET_BIT(hadc->ErrorCode, DAL_ADC_ERROR_INTERNAL);
  }
  
  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stop conversion of injected channels. Disable ADC peripheral if
  *         no regular conversion is on going.
  * @note   If ADC must be disabled and if conversion is on going on 
  *         regular group, function DAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @note   If injected group mode auto-injection is enabled,
  *         function DAL_ADC_Stop must be used.
  * @note   In case of auto-injection mode, DAL_ADC_Stop must be used.
  * @param  hadc ADC handle
  * @retval None
  */
DAL_StatusTypeDef DAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc)
{
  DAL_StatusTypeDef tmp_dal_status = DAL_OK;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __DAL_LOCK(hadc);
    
  /* Stop potential conversion and disable ADC peripheral                     */
  /* Conditioned to:                                                          */
  /* - No conversion on the other group (regular group) is intended to        */
  /*   continue (injected and regular groups stop conversion and ADC disable  */
  /*   are common)                                                            */
  /* - In case of auto-injection mode, DAL_ADC_Stop must be used.             */
  if(((hadc->State & DAL_ADC_STATE_REG_BUSY) == RESET)  &&
     DAL_IS_BIT_CLR(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN)   )
  {
    /* Stop potential conversion on going, on regular and injected groups */
    /* Disable ADC peripheral */
    __DAL_ADC_DISABLE(hadc);
    
    /* Check if ADC is effectively disabled */
    if(DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_ADCEN))
    {
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        DAL_ADC_STATE_REG_BUSY | DAL_ADC_STATE_INJ_BUSY,
                        DAL_ADC_STATE_READY);
    }
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, DAL_ADC_STATE_ERROR_CONFIG);
      
    tmp_dal_status = DAL_ERROR;
  }
  
  /* Process unlocked */
  __DAL_UNLOCK(hadc);
  
  /* Return function status */
  return tmp_dal_status;
}

/**
  * @brief  Poll for injected conversion complete
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  Timeout Timeout value in millisecond.  
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */ 
  tickstart = DAL_GetTick();

  /* Check End of conversion flag */
  while(!(__DAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC)))
  {
    /* Check for the Timeout */
    if(Timeout != DAL_MAX_DELAY)
    {
      if((Timeout == 0U)||((DAL_GetTick() - tickstart ) > Timeout))
      {
        /* New check to avoid false timeout detection in case of preemption */
        if(!(__DAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC)))
        {
          hadc->State= DAL_ADC_STATE_TIMEOUT;
          /* Process unlocked */
          __DAL_UNLOCK(hadc);
          return DAL_TIMEOUT;
        }
      }
    }
  }
  
  /* Clear injected group conversion flag */
  __DAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JSTRT | ADC_FLAG_JEOC);
    
  /* Update ADC state machine */
  SET_BIT(hadc->State, DAL_ADC_STATE_INJ_EOC);
  
  /* Determine whether any further conversion upcoming on group injected      */
  /* by external trigger, continuous mode or scan sequence on going.          */
  /* Note: On APM32F4, there is no independent flag of end of sequence.       */
  /*       The test of scan sequence on going is done either with scan        */
  /*       sequence disabled or with end of conversion flag set to            */
  /*       of end of sequence.                                                */
  if(ADC_IS_SOFTWARE_START_INJECTED(hadc)                    &&
     (DAL_IS_BIT_CLR(hadc->Instance->INJSEQ, ADC_INJSEQ_INJSEQLEN)  ||
      DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_EOCSEL)    ) &&
     (DAL_IS_BIT_CLR(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN) &&
      (ADC_IS_SOFTWARE_START_REGULAR(hadc)       &&
      (hadc->Init.ContinuousConvMode == DISABLE)   )       )   )
  {
    /* Set ADC state */
    CLEAR_BIT(hadc->State, DAL_ADC_STATE_INJ_BUSY);
    
    if (DAL_IS_BIT_CLR(hadc->State, DAL_ADC_STATE_REG_BUSY))
    { 
      SET_BIT(hadc->State, DAL_ADC_STATE_READY);
    }
  }
  
  /* Return ADC state */
  return DAL_OK;
}      
  
/**
  * @brief  Stop conversion of injected channels, disable interruption of 
  *         end-of-conversion. Disable ADC peripheral if no regular conversion
  *         is on going.
  * @note   If ADC must be disabled and if conversion is on going on 
  *         regular group, function DAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @note   If injected group mode auto-injection is enabled,
  *         function DAL_ADC_Stop must be used.
  * @param  hadc ADC handle
  * @retval None
  */
DAL_StatusTypeDef DAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc)
{
  DAL_StatusTypeDef tmp_dal_status = DAL_OK;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Process locked */
  __DAL_LOCK(hadc);
    
  /* Stop potential conversion and disable ADC peripheral                     */
  /* Conditioned to:                                                          */
  /* - No conversion on the other group (regular group) is intended to        */
  /*   continue (injected and regular groups stop conversion and ADC disable  */
  /*   are common)                                                            */
  /* - In case of auto-injection mode, DAL_ADC_Stop must be used.             */ 
  if(((hadc->State & DAL_ADC_STATE_REG_BUSY) == RESET)  &&
     DAL_IS_BIT_CLR(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN)   )
  {
    /* Stop potential conversion on going, on regular and injected groups */
    /* Disable ADC peripheral */
    __DAL_ADC_DISABLE(hadc);
    
    /* Check if ADC is effectively disabled */
    if(DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_ADCEN))
    {
      /* Disable ADC end of conversion interrupt for injected channels */
      __DAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
      
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        DAL_ADC_STATE_REG_BUSY | DAL_ADC_STATE_INJ_BUSY,
                        DAL_ADC_STATE_READY);
    }
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, DAL_ADC_STATE_ERROR_CONFIG);
      
    tmp_dal_status = DAL_ERROR;
  }
  
  /* Process unlocked */
  __DAL_UNLOCK(hadc);
  
  /* Return function status */
  return tmp_dal_status;
}

/**
  * @brief  Gets the converted value from data register of injected channel.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  InjectedRank the ADC injected rank.
  *          This parameter can be one of the following values:
  *            @arg ADC_INJECTED_RANK_1: Injected Channel1 selected
  *            @arg ADC_INJECTED_RANK_2: Injected Channel2 selected
  *            @arg ADC_INJECTED_RANK_3: Injected Channel3 selected
  *            @arg ADC_INJECTED_RANK_4: Injected Channel4 selected
  * @retval None
  */
uint32_t DAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef* hadc, uint32_t InjectedRank)
{
  __IO uint32_t tmp = 0U;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_INJECTED_RANK(InjectedRank));
  
  /* Clear injected group conversion flag to have similar behaviour as        */
  /* regular group: reading data register also clears end of conversion flag. */
  __DAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);
  
  /* Return the selected ADC converted value */ 
  switch(InjectedRank)
  {  
    case ADC_INJECTED_RANK_4:
    {
      tmp =  hadc->Instance->INJDATA4;
    }  
    break;
    case ADC_INJECTED_RANK_3: 
    {  
      tmp =  hadc->Instance->INJDATA3;
    }  
    break;
    case ADC_INJECTED_RANK_2: 
    {  
      tmp =  hadc->Instance->INJDATA2;
    }
    break;
    case ADC_INJECTED_RANK_1:
    {
      tmp =  hadc->Instance->INJDATA1;
    }
    break;
    default:
    break;  
  }
  return tmp;
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  Enables ADC DMA request after last transfer (Multi-ADC mode) and enables ADC peripheral
  * 
  * @note   Caution: This function must be used only with the ADC master.  
  *
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  pData   Pointer to buffer in which transferred from ADC peripheral to memory will be stored. 
  * @param  Length  The length of data to be transferred from ADC peripheral to memory.  
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
{
  __IO uint32_t counter = 0U;
  ADC_Common_TypeDef *tmpADC_Common;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
  ASSERT_PARAM(IS_ADC_EXT_TRIG_EDGE(hadc->Init.ExternalTrigConvEdge));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hadc->Init.DMAContinuousRequests));
  
  /* Process locked */
  __DAL_LOCK(hadc);
  
  /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
  if((hadc->Instance->CTRL2 & ADC_CTRL2_ADCEN) != ADC_CTRL2_ADCEN)
  {  
    /* Enable the Peripheral */
    __DAL_ADC_ENABLE(hadc);
    
    /* Delay for temperature sensor stabilization time */
    /* Compute number of CPU cycles to wait for */
    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while(counter != 0U)
    {
      counter--;
    }
  }
  
  /* Start conversion if ADC is effectively enabled */
  if(DAL_IS_BIT_SET(hadc->Instance->CTRL2, ADC_CTRL2_ADCEN))
  {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
    ADC_STATE_CLR_SET(hadc->State,
                      DAL_ADC_STATE_READY | DAL_ADC_STATE_REG_EOC | DAL_ADC_STATE_REG_OVR,
                      DAL_ADC_STATE_REG_BUSY);
    
    /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
    if (READ_BIT(hadc->Instance->CTRL1, ADC_CTRL1_INJGACEN) != RESET)
    {
      ADC_STATE_CLR_SET(hadc->State, DAL_ADC_STATE_INJ_EOC, DAL_ADC_STATE_INJ_BUSY);  
    }
    
    /* State machine update: Check if an injected conversion is ongoing */
    if (DAL_IS_BIT_SET(hadc->State, DAL_ADC_STATE_INJ_BUSY))
    {
      /* Reset ADC error code fields related to conversions on group regular */
      CLEAR_BIT(hadc->ErrorCode, (DAL_ADC_ERROR_OVR | DAL_ADC_ERROR_DMA));         
    }
    else
    {
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE(hadc);
    }
    
    /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
    __DAL_UNLOCK(hadc);
    
    /* Set the DMA transfer complete callback */
    hadc->DMA_Handle->XferCpltCallback = ADC_MultiModeDMAConvCplt;
    
    /* Set the DMA half transfer complete callback */
    hadc->DMA_Handle->XferHalfCpltCallback = ADC_MultiModeDMAHalfConvCplt;
    
    /* Set the DMA error callback */
    hadc->DMA_Handle->XferErrorCallback = ADC_MultiModeDMAError ;
    
    /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
    /* start (in case of SW start):                                           */
    
    /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __DAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC);

    /* Enable ADC overrun interrupt */
    __DAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

    /* Pointer to the common control register to which is belonging hadc    */
    /* (Depending on APM32F4 product, there may be up to 3 ADC and 1 common */
    /* control register)                                                    */
    tmpADC_Common = ADC_COMMON_REGISTER(hadc);

    if (hadc->Init.DMAContinuousRequests != DISABLE)
    {
      /* Enable the selected ADC DMA request after last transfer */
      tmpADC_Common->CCTRL |= ADC_CCTRL_DMAMODEDISSEL;
    }
    else
    {
      /* Disable the selected ADC EOC rising on each regular channel conversion */
      tmpADC_Common->CCTRL &= ~ADC_CCTRL_DMAMODEDISSEL;
    }
    
    /* Enable the DMA Stream */
    DAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&tmpADC_Common->CDATA, (uint32_t)pData, Length);
    
    /* if no external trigger present enable software conversion of regular channels */
    if((hadc->Instance->CTRL2 & ADC_CTRL2_REGEXTTRGEN) == RESET) 
    {
      /* Enable the selected ADC software conversion for regular group */
      hadc->Instance->CTRL2 |= (uint32_t)ADC_CTRL2_REGCHSC;
    }
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, DAL_ADC_STATE_ERROR_INTERNAL);

    /* Set ADC error code to ADC IP internal error */
    SET_BIT(hadc->ErrorCode, DAL_ADC_ERROR_INTERNAL);
  }
  
  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Disables ADC DMA (multi-ADC mode) and disables ADC peripheral    
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef* hadc)
{
  DAL_StatusTypeDef tmp_dal_status = DAL_OK;
  ADC_Common_TypeDef *tmpADC_Common;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_ALL_INSTANCE(hadc->Instance));
  
  /* Process locked */
  __DAL_LOCK(hadc);
  
  /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
  __DAL_ADC_DISABLE(hadc);

  /* Pointer to the common control register to which is belonging hadc    */
  /* (Depending on APM32F4 product, there may be up to 3 ADC and 1 common */
  /* control register)                                                    */
  tmpADC_Common = ADC_COMMON_REGISTER(hadc);

  /* Check if ADC is effectively disabled */
  if(DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_ADCEN))
  {
    /* Disable the selected ADC DMA mode for multimode */
    tmpADC_Common->CCTRL &= ~ADC_CCTRL_DMAMODEDISSEL;
    
    /* Disable the DMA channel (in case of DMA in circular mode or stop while */
    /* DMA transfer is on going)                                              */
    tmp_dal_status = DAL_DMA_Abort(hadc->DMA_Handle);
    
    /* Disable ADC overrun interrupt */
    __DAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);
    
    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      DAL_ADC_STATE_REG_BUSY | DAL_ADC_STATE_INJ_BUSY,
                      DAL_ADC_STATE_READY);
  }
  
  /* Process unlocked */
  __DAL_UNLOCK(hadc);
  
  /* Return function status */
  return tmp_dal_status;
}

/**
  * @brief  Returns the last ADC1, ADC2 and ADC3 regular conversions results 
  *         data in the selected multi mode.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval The converted data value.
  */
uint32_t DAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef* hadc)
{
  ADC_Common_TypeDef *tmpADC_Common;

  UNUSED(hadc);

  /* Pointer to the common control register to which is belonging hadc    */
  /* (Depending on APM32F4 product, there may be up to 3 ADC and 1 common */
  /* control register)                                                    */
  tmpADC_Common = ADC_COMMON_REGISTER(hadc);

  /* Return the multi mode conversion value */
  return tmpADC_Common->CDATA;
}
#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @brief  Injected conversion complete callback in non blocking mode 
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
__weak void DAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_ADC_InjectedConvCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Configures for the selected ADC injected channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  sConfigInjected ADC configuration structure for injected channel. 
  * @retval None
  */
DAL_StatusTypeDef DAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc, ADC_InjectionConfTypeDef* sConfigInjected)
{
  
#if (USE_FULL_ASSERT == 1U)  
  uint32_t tmp = 0U;
  
#endif /* USE_FULL_ASSERT  */

  ADC_Common_TypeDef *tmpADC_Common;

  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_CHANNEL(sConfigInjected->InjectedChannel));
  ASSERT_PARAM(IS_ADC_INJECTED_RANK(sConfigInjected->InjectedRank));
  ASSERT_PARAM(IS_ADC_SAMPLE_TIME(sConfigInjected->InjectedSamplingTime));
  ASSERT_PARAM(IS_ADC_EXT_INJEC_TRIG(sConfigInjected->ExternalTrigInjecConv));
  ASSERT_PARAM(IS_ADC_INJECTED_LENGTH(sConfigInjected->InjectedNbrOfConversion));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(sConfigInjected->AutoInjectedConv));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(sConfigInjected->InjectedDiscontinuousConvMode));

#if (USE_FULL_ASSERT == 1U)
  tmp = ADC_GET_RESOLUTION(hadc);
  ASSERT_PARAM(IS_ADC_RANGE(tmp, sConfigInjected->InjectedOffset));
#endif /* USE_FULL_ASSERT  */

  if(sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
  {
    ASSERT_PARAM(IS_ADC_EXT_INJEC_TRIG_EDGE(sConfigInjected->ExternalTrigInjecConvEdge));
  }

  /* Process locked */
  __DAL_LOCK(hadc);
  
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (sConfigInjected->InjectedChannel > ADC_CHANNEL_9)
  {
    /* Clear the old sample time */
    hadc->Instance->SMPTIM1 &= ~ADC_SMPTIM1(ADC_SMPTIM1_SMPCYCCFG10, sConfigInjected->InjectedChannel);
    
    /* Set the new sample time */
    hadc->Instance->SMPTIM1 |= ADC_SMPTIM1(sConfigInjected->InjectedSamplingTime, sConfigInjected->InjectedChannel);
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Clear the old sample time */
    hadc->Instance->SMPTIM2 &= ~ADC_SMPTIM2(ADC_SMPTIM2_SMPCYCCFG0, sConfigInjected->InjectedChannel);
    
    /* Set the new sample time */
    hadc->Instance->SMPTIM2 |= ADC_SMPTIM2(sConfigInjected->InjectedSamplingTime, sConfigInjected->InjectedChannel);
  }
  
  /*---------------------------- ADCx JSQR Configuration -----------------*/
  hadc->Instance->INJSEQ &= ~(ADC_INJSEQ_INJSEQLEN);
  hadc->Instance->INJSEQ |=  ADC_REGSEQ1(sConfigInjected->InjectedNbrOfConversion);
  
  /* Rank configuration */
  
  /* Clear the old SQx bits for the selected rank */
  hadc->Instance->INJSEQ &= ~ADC_INJSEQ(ADC_INJSEQ_INJSEQC1, sConfigInjected->InjectedRank,sConfigInjected->InjectedNbrOfConversion);
   
  /* Set the SQx bits for the selected rank */
  hadc->Instance->INJSEQ |= ADC_INJSEQ(sConfigInjected->InjectedChannel, sConfigInjected->InjectedRank,sConfigInjected->InjectedNbrOfConversion);

  /* Enable external trigger if trigger selection is different of software  */
  /* start.                                                                 */
  /* Note: This configuration keeps the hardware feature of parameter       */
  /*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
  /*       software start.                                                  */ 
  if(sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
  {  
    /* Select external trigger to start conversion */
    hadc->Instance->CTRL2 &= ~(ADC_CTRL2_INJGEXTTRGSEL);
    hadc->Instance->CTRL2 |=  sConfigInjected->ExternalTrigInjecConv;
    
    /* Select external trigger polarity */
    hadc->Instance->CTRL2 &= ~(ADC_CTRL2_INJEXTTRGEN);
    hadc->Instance->CTRL2 |= sConfigInjected->ExternalTrigInjecConvEdge;
  }
  else
  {
    /* Reset the external trigger */
    hadc->Instance->CTRL2 &= ~(ADC_CTRL2_INJGEXTTRGSEL);
    hadc->Instance->CTRL2 &= ~(ADC_CTRL2_INJEXTTRGEN);  
  }
  
  if (sConfigInjected->AutoInjectedConv != DISABLE)
  {
    /* Enable the selected ADC automatic injected group conversion */
    hadc->Instance->CTRL1 |= ADC_CTRL1_INJGACEN;
  }
  else
  {
    /* Disable the selected ADC automatic injected group conversion */
    hadc->Instance->CTRL1 &= ~(ADC_CTRL1_INJGACEN);
  }
  
  if (sConfigInjected->InjectedDiscontinuousConvMode != DISABLE)
  {
    /* Enable the selected ADC injected discontinuous mode */
    hadc->Instance->CTRL1 |= ADC_CTRL1_INJDISCEN;
  }
  else
  {
    /* Disable the selected ADC injected discontinuous mode */
    hadc->Instance->CTRL1 &= ~(ADC_CTRL1_INJDISCEN);
  }
  
  switch(sConfigInjected->InjectedRank)
  {
    case 1U:
      /* Set injected channel 1 offset */
      hadc->Instance->INJDOF1 &= ~(ADC_INJDOF1_INJDOF1);
      hadc->Instance->INJDOF1 |= sConfigInjected->InjectedOffset;
      break;
    case 2U:
      /* Set injected channel 2 offset */
      hadc->Instance->INJDOF2 &= ~(ADC_INJDOF2_INJDOF2);
      hadc->Instance->INJDOF2 |= sConfigInjected->InjectedOffset;
      break;
    case 3U:
      /* Set injected channel 3 offset */
      hadc->Instance->INJDOF3 &= ~(ADC_INJDOF3_INJDOF3);
      hadc->Instance->INJDOF3 |= sConfigInjected->InjectedOffset;
      break;
    default:
      /* Set injected channel 4 offset */
      hadc->Instance->INJDOF4 &= ~(ADC_INJDOF4_INJDOF4);
      hadc->Instance->INJDOF4 |= sConfigInjected->InjectedOffset;
      break;
  }

  /* Pointer to the common control register to which is belonging hadc    */
  /* (Depending on APM32F4 product, there may be up to 3 ADC and 1 common */
  /* control register)                                                    */
    tmpADC_Common = ADC_COMMON_REGISTER(hadc);

  /* if ADC1 Channel_18 is selected enable VBAT Channel */
  if ((hadc->Instance == ADC1) && (sConfigInjected->InjectedChannel == ADC_CHANNEL_VBAT))
  {
    /* Enable the VBAT channel*/
    tmpADC_Common->CCTRL |= ADC_CCTRL_VBATEN;
  }
  
  /* if ADC1 Channel_16 or Channel_17 is selected enable TSVREFEN Channel(Temperature sensor and VREFINT) */
  if ((hadc->Instance == ADC1) && ((sConfigInjected->InjectedChannel == ADC_CHANNEL_TEMPSENSOR) || (sConfigInjected->InjectedChannel == ADC_CHANNEL_VREFINT)))
  {
    /* Enable the TSVREFEN channel*/
    tmpADC_Common->CCTRL |= ADC_CCTRL_TSVREFEN;
  }
  
  /* Process unlocked */
  __DAL_UNLOCK(hadc);
  
  /* Return function status */
  return DAL_OK;
}

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  Configures the ADC multi-mode 
  * @param  hadc       pointer to a ADC_HandleTypeDef structure that contains
  *                     the configuration information for the specified ADC.  
  * @param  multimode  pointer to an ADC_MultiModeTypeDef structure that contains 
  *                     the configuration information for  multimode.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef* hadc, ADC_MultiModeTypeDef* multimode)
{

  ADC_Common_TypeDef *tmpADC_Common;

  /* Check the parameters */
  ASSERT_PARAM(IS_ADC_MODE(multimode->Mode));
  ASSERT_PARAM(IS_ADC_DMA_ACCESS_MODE(multimode->DMAAccessMode));
  ASSERT_PARAM(IS_ADC_SAMPLING_DELAY(multimode->TwoSamplingDelay));
  
  /* Process locked */
  __DAL_LOCK(hadc);

  /* Pointer to the common control register to which is belonging hadc    */
  /* (Depending on APM32F4 product, there may be up to 3 ADC and 1 common */
  /* control register)                                                    */
  tmpADC_Common = ADC_COMMON_REGISTER(hadc);

  /* Set ADC mode */
  tmpADC_Common->CCTRL &= ~(ADC_CCTRL_ADCMSEL);
  tmpADC_Common->CCTRL |= multimode->Mode;
  
  /* Set the ADC DMA access mode */
  tmpADC_Common->CCTRL &= ~(ADC_CCTRL_DMAMODE);
  tmpADC_Common->CCTRL |= multimode->DMAAccessMode;
  
  /* Set delay between two sampling phases */
  tmpADC_Common->CCTRL &= ~(ADC_CCTRL_SMPDEL2);
  tmpADC_Common->CCTRL |= multimode->TwoSamplingDelay;
  
  /* Process unlocked */
  __DAL_UNLOCK(hadc);
  
  /* Return function status */
  return DAL_OK;
}
#endif /* ADC_MULTIMODE_SUPPORT */

/**
  * @}
  */

#if defined(ADC_MULTIMODE_SUPPORT)
/**
  * @brief  DMA transfer complete callback. 
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void ADC_MultiModeDMAConvCplt(DMA_HandleTypeDef *hdma)   
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  
  /* Update state machine on conversion status if not in error state */
  if (DAL_IS_BIT_CLR(hadc->State, DAL_ADC_STATE_ERROR_INTERNAL | DAL_ADC_STATE_ERROR_DMA))
  {
    /* Update ADC state machine */
    SET_BIT(hadc->State, DAL_ADC_STATE_REG_EOC);
    
    /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On APM32F4, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
    if(ADC_IS_SOFTWARE_START_REGULAR(hadc)                   &&
       (hadc->Init.ContinuousConvMode == DISABLE)            &&
       (DAL_IS_BIT_CLR(hadc->Instance->REGSEQ1, ADC_REGSEQ1_REGSEQLEN) || 
        DAL_IS_BIT_CLR(hadc->Instance->CTRL2, ADC_CTRL2_EOCSEL)  )   )
    {
      /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* DAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
      __DAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);
      
      /* Set ADC state */
      CLEAR_BIT(hadc->State, DAL_ADC_STATE_REG_BUSY);   
      
      if (DAL_IS_BIT_CLR(hadc->State, DAL_ADC_STATE_INJ_BUSY))
      {
        SET_BIT(hadc->State, DAL_ADC_STATE_READY);
      }
    }
    
    /* Conversion complete callback */
    DAL_ADC_ConvCpltCallback(hadc);
  }
  else
  {
    /* Call DMA error callback */
    hadc->DMA_Handle->XferErrorCallback(hdma);
  }
}

/**
  * @brief  DMA half transfer complete callback. 
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void ADC_MultiModeDMAHalfConvCplt(DMA_HandleTypeDef *hdma)   
{
    ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
    /* Conversion complete callback */
    DAL_ADC_ConvHalfCpltCallback(hadc); 
}

/**
  * @brief  DMA error callback 
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void ADC_MultiModeDMAError(DMA_HandleTypeDef *hdma)   
{
    ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
    hadc->State= DAL_ADC_STATE_ERROR_DMA;
    /* Set ADC error code to DMA error */
    hadc->ErrorCode |= DAL_ADC_ERROR_DMA;
    DAL_ADC_ErrorCallback(hadc); 
}
#endif /* ADC_MULTIMODE_SUPPORT */
/**
  * @}
  */

#endif /* DAL_ADC_MODULE_ENABLED */
/**
  * @}
  */ 

/**
  * @}
  */ 

