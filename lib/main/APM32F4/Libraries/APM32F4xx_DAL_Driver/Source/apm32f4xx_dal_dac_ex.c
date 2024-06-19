/**
  *
  * @file    apm32f4xx_dal_dac_ex.c
  * @brief   Extended DAC DAL module driver.
  *          This file provides firmware functions to manage the extended
  *          functionalities of the DAC peripheral.
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
                      ##### How to use this driver #####
  ==============================================================================
    [..]

     *** Dual mode IO operation ***
     ==============================
     [..]
      (+) When Dual mode is enabled (i.e. DAC Channel1 and Channel2 are used simultaneously) :
          Use DAL_DACEx_DualGetValue() to get digital data to be converted and use
          DAL_DACEx_DualSetValue() to set digital value to converted simultaneously in
          Channel 1 and Channel 2.

     *** Signal generation operation ***
     ===================================
     [..]
      (+) Use DAL_DACEx_TriangleWaveGenerate() to generate Triangle signal.
      (+) Use DAL_DACEx_NoiseWaveGenerate() to generate Noise signal.

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

/** @defgroup DACEx DACEx
  * @brief DAC Extended DAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup DACEx_Exported_Functions DACEx Exported Functions
  * @{
  */

/** @defgroup DACEx_Exported_Functions_Group2 IO operation functions
  *  @brief    Extended IO operation functions
  *
@verbatim
  ==============================================================================
                 ##### Extended features functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Start conversion.
      (+) Stop conversion.
      (+) Start conversion and enable DMA transfer.
      (+) Stop conversion and disable DMA transfer.
      (+) Get result of conversion.
      (+) Get result of dual mode conversion.

@endverbatim
  * @{
  */

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Enables DAC and starts conversion of both channels.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DACEx_DualStart(DAC_HandleTypeDef *hdac)
{
  uint32_t tmp_swtrig = 0UL;


  /* Process locked */
  __DAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_BUSY;

  /* Enable the Peripheral */
  __DAL_DAC_ENABLE(hdac, DAC_CHANNEL_1);
  __DAL_DAC_ENABLE(hdac, DAC_CHANNEL_2);

  /* Check if software trigger enabled */
  if ((hdac->Instance->CTRL & (DAC_CTRL_TRGENCH1 | DAC_CTRL_TRGSELCH1)) == DAC_TRIGGER_SOFTWARE)
  {
    tmp_swtrig |= DAC_SWTRG_SWTRG1;
  }
  if ((hdac->Instance->CTRL & (DAC_CTRL_TRGENCH2 | DAC_CTRL_TRGSELCH2)) == (DAC_TRIGGER_SOFTWARE << (DAC_CHANNEL_2 & 0x10UL)))
  {
    tmp_swtrig |= DAC_SWTRG_SWTRG2;
  }
  /* Enable the selected DAC software conversion*/
  SET_BIT(hdac->Instance->SWTRG, tmp_swtrig);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Process unlocked */
  __DAL_UNLOCK(hdac);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Disables DAC and stop conversion of both channels.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DACEx_DualStop(DAC_HandleTypeDef *hdac)
{

  /* Disable the Peripheral */
  __DAL_DAC_DISABLE(hdac, DAC_CHANNEL_1);
  __DAL_DAC_DISABLE(hdac, DAC_CHANNEL_2);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Return function status */
  return DAL_OK;
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @brief  Enable or disable the selected DAC channel wave generation.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  Amplitude Select max triangle amplitude.
  *          This parameter can be one of the following values:
  *            @arg DAC_TRIANGLEAMPLITUDE_1: Select max triangle amplitude of 1
  *            @arg DAC_TRIANGLEAMPLITUDE_3: Select max triangle amplitude of 3
  *            @arg DAC_TRIANGLEAMPLITUDE_7: Select max triangle amplitude of 7
  *            @arg DAC_TRIANGLEAMPLITUDE_15: Select max triangle amplitude of 15
  *            @arg DAC_TRIANGLEAMPLITUDE_31: Select max triangle amplitude of 31
  *            @arg DAC_TRIANGLEAMPLITUDE_63: Select max triangle amplitude of 63
  *            @arg DAC_TRIANGLEAMPLITUDE_127: Select max triangle amplitude of 127
  *            @arg DAC_TRIANGLEAMPLITUDE_255: Select max triangle amplitude of 255
  *            @arg DAC_TRIANGLEAMPLITUDE_511: Select max triangle amplitude of 511
  *            @arg DAC_TRIANGLEAMPLITUDE_1023: Select max triangle amplitude of 1023
  *            @arg DAC_TRIANGLEAMPLITUDE_2047: Select max triangle amplitude of 2047
  *            @arg DAC_TRIANGLEAMPLITUDE_4095: Select max triangle amplitude of 4095
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));
  ASSERT_PARAM(IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(Amplitude));

  /* Process locked */
  __DAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_BUSY;

  /* Enable the triangle wave generation for the selected DAC channel */
  MODIFY_REG(hdac->Instance->CTRL, ((DAC_CTRL_WAVENCH1) | (DAC_CTRL_MAMPSELCH1)) << (Channel & 0x10UL),
             (DAC_CTRL_WAVENCH1_1 | Amplitude) << (Channel & 0x10UL));

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Process unlocked */
  __DAL_UNLOCK(hdac);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Enable or disable the selected DAC channel wave generation.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  Amplitude Unmask DAC channel LFSR for noise wave generation.
  *          This parameter can be one of the following values:
  *            @arg DAC_LFSRUNMASK_BIT0: Unmask DAC channel LFSR bit0 for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS1_0: Unmask DAC channel LFSR bit[1:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS2_0: Unmask DAC channel LFSR bit[2:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS3_0: Unmask DAC channel LFSR bit[3:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS4_0: Unmask DAC channel LFSR bit[4:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS5_0: Unmask DAC channel LFSR bit[5:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS6_0: Unmask DAC channel LFSR bit[6:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS7_0: Unmask DAC channel LFSR bit[7:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS8_0: Unmask DAC channel LFSR bit[8:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS9_0: Unmask DAC channel LFSR bit[9:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS10_0: Unmask DAC channel LFSR bit[10:0] for noise wave generation
  *            @arg DAC_LFSRUNMASK_BITS11_0: Unmask DAC channel LFSR bit[11:0] for noise wave generation
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_CHANNEL(Channel));
  ASSERT_PARAM(IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(Amplitude));

  /* Process locked */
  __DAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_BUSY;

  /* Enable the noise wave generation for the selected DAC channel */
  MODIFY_REG(hdac->Instance->CTRL, ((DAC_CTRL_WAVENCH1) | (DAC_CTRL_MAMPSELCH1)) << (Channel & 0x10UL),
             (DAC_CTRL_WAVENCH1_0 | Amplitude) << (Channel & 0x10UL));

  /* Change DAC state */
  hdac->State = DAL_DAC_STATE_READY;

  /* Process unlocked */
  __DAL_UNLOCK(hdac);

  /* Return function status */
  return DAL_OK;
}

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Set the specified data holding register value for dual DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *               the configuration information for the specified DAC.
  * @param  Alignment Specifies the data alignment for dual channel DAC.
  *          This parameter can be one of the following values:
  *            DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data1 Data for DAC Channel1 to be loaded in the selected data holding register.
  * @param  Data2 Data for DAC Channel2 to be loaded in the selected data  holding register.
  * @note   In dual mode, a unique register access is required to write in both
  *          DAC channels at the same time.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DACEx_DualSetValue(DAC_HandleTypeDef *hdac, uint32_t Alignment, uint32_t Data1, uint32_t Data2)
{
  uint32_t data;
  uint32_t tmp;

  /* Check the parameters */
  ASSERT_PARAM(IS_DAC_ALIGN(Alignment));
  ASSERT_PARAM(IS_DAC_DATA(Data1));
  ASSERT_PARAM(IS_DAC_DATA(Data2));

  /* Calculate and set dual DAC data holding register value */
  if (Alignment == DAC_ALIGN_8B_R)
  {
    data = ((uint32_t)Data2 << 8U) | Data1;
  }
  else
  {
    data = ((uint32_t)Data2 << 16U) | Data1;
  }

  tmp = (uint32_t)hdac->Instance;
  tmp += DAC_DH12RDUAL_ALIGNMENT(Alignment);

  /* Set the dual DAC selected data holding register */
  *(__IO uint32_t *)tmp = data;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Conversion complete callback in non-blocking mode for Channel2.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DACEx_ConvCpltCallbackCh2 could be implemented in the user file
   */
}

/**
  * @brief  Conversion half DMA transfer callback in non-blocking mode for Channel2.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DACEx_ConvHalfCpltCallbackCh2 could be implemented in the user file
   */
}

/**
  * @brief  Error DAC callback for Channel2.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DACEx_ErrorCallbackCh2 could be implemented in the user file
   */
}

/**
  * @brief  DMA underrun DAC callback for Channel2.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void DAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_DACEx_DMAUnderrunCallbackCh2 could be implemented in the user file
   */
}
#endif /* DAC_CHANNEL2_SUPPORT */


/**
  * @}
  */

/** @defgroup DACEx_Exported_Functions_Group3 Peripheral Control functions
  *  @brief    Extended Peripheral Control functions
  *
@verbatim
  ==============================================================================
             ##### Peripheral Control functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Set the specified data holding register value for DAC channel.

@endverbatim
  * @{
  */

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Return the last data output value of the selected DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval The selected DAC channel data output value.
  */
uint32_t DAL_DACEx_DualGetValue(DAC_HandleTypeDef *hdac)
{
  uint32_t tmp = 0UL;

  tmp |= hdac->Instance->DATAOCH1;

  tmp |= hdac->Instance->DATAOCH2 << 16UL;

  /* Returns the DAC channel data output register value */
  return tmp;
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @}
  */
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup DACEx_Private_Functions DACEx private functions
  *  @brief    Extended private functions
  * @{
  */

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  DMA conversion complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAConvCpltCh2(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ConvCpltCallbackCh2(hdac);
#else
  DAL_DACEx_ConvCpltCallbackCh2(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

  hdac->State = DAL_DAC_STATE_READY;
}

/**
  * @brief  DMA half transfer complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAHalfConvCpltCh2(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* Conversion complete callback */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ConvHalfCpltCallbackCh2(hdac);
#else
  DAL_DACEx_ConvHalfCpltCallbackCh2(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAErrorCh2(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Set DAC error code to DMA error */
  hdac->ErrorCode |= DAL_DAC_ERROR_DMA;

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ErrorCallbackCh2(hdac);
#else
  DAL_DACEx_ErrorCallbackCh2(hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

  hdac->State = DAL_DAC_STATE_READY;
}
#endif /* DAC_CHANNEL2_SUPPORT */

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

