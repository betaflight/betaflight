/**
  ******************************************************************************
  * @file     um324xx_hal_dac.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-18
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */
#ifdef HAL_DAC_MODULE_ENABLED
/** @defgroup DAC_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef *hdac)
{
    /* Check DAC handle */
    if (hdac == NULL)
    {
        return HAL_ERROR;
    }

    if (hdac->State == HAL_DAC_STATE_RESET)
    {
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
        /* Init the DAC Callback settings */
        hdac->ConvCpltCallbackCh0           = HAL_DAC_ConvCpltCallbackCh0;
        hdac->ErrorCallbackCh0              = HAL_DAC_ErrorCallbackCh0;
        hdac->DMAUnderrunCallbackCh0        = HAL_DAC_DMAUnderrunCallbackCh0;
#if defined(DAC_CHANNEL1_SUPPORT)
        hdac->ConvCpltCallbackCh1           = HAL_DACEx_ConvCpltCallbackCh1;
        hdac->ErrorCallbackCh1              = HAL_DACEx_ErrorCallbackCh1;
        hdac->DMAUnderrunCallbackCh1        = HAL_DACEx_DMAUnderrunCallbackCh1;
#endif /* DAC_CHANNEL1_SUPPORT */
        if (hdac->MspInitCallback == NULL)
        {
            hdac->MspInitCallback             = HAL_DAC_MspInit;
        }
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

        /* Allocate lock resource and initialize it */
        hdac->Lock = HAL_UNLOCKED;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
        /* Init the low level hardware */
        hdac->MspInitCallback(hdac);
#else
        /* Init the low level hardware */
        HAL_DAC_MspInit(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
    }
 
    /* Initialize the DAC state */
    hdac->State = HAL_DAC_STATE_BUSY;

    /* Set DAC error code to none */
    hdac->ErrorCode = HAL_DAC_ERROR_NONE;

    /* Initialize the DAC state */
    hdac->State = HAL_DAC_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Deinitialize the DAC peripheral registers to their default reset values.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef *hdac)
{
    /* Check DAC handle */
    if (hdac == NULL)
    {
        return HAL_ERROR;
    }

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_BUSY;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
    if (hdac->MspDeInitCallback == NULL)
    {
        hdac->MspDeInitCallback = HAL_DAC_MspDeInit;
    }
    /* DeInit the low level hardware */
    hdac->MspDeInitCallback(hdac);
#else
    /* DeInit the low level hardware */
    HAL_DAC_MspDeInit(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

    /* Set DAC error code to none */
    hdac->ErrorCode = HAL_DAC_ERROR_NONE;

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hdac);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Initialize the DAC MSP.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdac);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_DAC_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitialize the DAC MSP.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdac);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_DAC_MspDeInit could be implemented in the user file
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
  *            @arg DAC_CHANNEL_0: DAC Channel0 selected
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel)
{
    /* Process locked */
    __HAL_LOCK(hdac);

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_BUSY;

    /* Enable the Peripheral */
    __HAL_DAC_ENABLE(hdac, Channel);

    if (Channel == DAC_CHANNEL_0)
    {
        /* Check if software trigger enabled */
        if ((hdac->Instance->CTRL & (DAC_CTRL_TEN0 | DAC_CTRL_TSEL0)) == DAC_TRIGGER_SOFTWARE)
        {
            /* Enable the selected DAC software conversion */
            SET_BIT(hdac->Instance->SWTRG, DAC_SWTRG_SWTRG0);
        }
    }
#if defined(DAC_CHANNEL1_SUPPORT)
    else
    {
        /* Check if software trigger enabled */
        if ((hdac->Instance->CTRL & (DAC_CTRL_TEN1 | DAC_CTRL_TSEL1)) == (DAC_TRIGGER_SOFTWARE << (Channel & 0x10UL)))
        {
            /* Enable the selected DAC software conversion */
            SET_BIT(hdac->Instance->SWTRG, DAC_SWTRG_SWTRG1);
        }
    }
#endif /* DAC_CHANNEL1_SUPPORT */

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_READY;

    /* Process unlocked */
    __HAL_UNLOCK(hdac);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Disables DAC and stop conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_0: DAC Channel0 selected
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel)
{
    /* Disable the Peripheral */
    __HAL_DAC_DISABLE(hdac, Channel);

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Enables DAC and starts conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_0: DAC Channel0 selected
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to DAC peripheral
  * @param  Alignment Specifies the data alignment for DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t tmpreg = 0U;
	
	/* Process locked */
	__HAL_LOCK(hdac);
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_BUSY;
	
	if (Channel == DAC_CHANNEL_0)
	{
		/* Set the DMA half transfer complete callback for channel0 */
		hdac->DMA_Handle0->XferBlockCallback = DAC_DMAConvCpltCh0;
	
		/* Set the DMA error callback for channel0 */
		hdac->DMA_Handle0->XferErrorCallback = DAC_DMAErrorCh0;
	
		/* Enable the selected DAC channel0 DMA request */
		SET_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAEN0);
	
		/* Case of use of channel 0 */
		switch (Alignment)
		{
			case DAC_ALIGN_12B_R:
				/* Get DHR12R0 address */
				tmpreg = (uint32_t)&hdac->Instance->DHR12R0;
				break;
			case DAC_ALIGN_12B_L:
				/* Get DHR12L0 address */
				tmpreg = (uint32_t)&hdac->Instance->DHR12L0;
				break;
			case DAC_ALIGN_8B_R:
				/* Get DHR8R0 address */
				tmpreg = (uint32_t)&hdac->Instance->DHR8R0;
				break;
			default:
				break;
		}
	}
   #if defined(UM324xF)
#if defined(DAC_CHANNEL1_SUPPORT)
	else
	{
		/* Set the DMA half transfer complete callback for channel1 */
		hdac->DMA_Handle1->XferBlockCallback = DAC_DMAConvCpltCh1;
	
		/* Set the DMA error callback for channel1 */
		hdac->DMA_Handle1->XferErrorCallback = DAC_DMAErrorCh1;
	
		/* Enable the selected DAC channel1 DMA request */
		SET_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAEN1);
	
		/* Case of use of channel 1 */
		switch (Alignment)
		{
			case DAC_ALIGN_12B_R:
				/* Get DHR12R1 address */
				tmpreg = (uint32_t)&hdac->Instance->DHR12R1;
				break;
			case DAC_ALIGN_12B_L:
				/* Get DHR12L1 address */
				tmpreg = (uint32_t)&hdac->Instance->DHR12L1;
				break;
			case DAC_ALIGN_8B_R:
				/* Get DHR8R1 address */
				tmpreg = (uint32_t)&hdac->Instance->DHR8R1;
				break;
			default:
				break;
		}
	}
#endif /* DAC_CHANNEL1_SUPPORT */
#endif	
	/* Enable the DMA Stream */
	if (Channel == DAC_CHANNEL_0)
	{
		/* Enable the DAC DMA  interrupt */
		__HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAIT0);
	
		/* Enable the DMA Stream */
		status = HAL_DMA_Start_IT(hdac->DMA_Handle0, (uint32_t)pData, tmpreg, Length);
	}
#if defined(UM324xF)
#if defined(DAC_CHANNEL1_SUPPORT)
	else
	{
		/* Enable the DAC DMA  interrupt */
		__HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAIT1);
	
		/* Enable the DMA Stream */
		status = HAL_DMA_Start_IT(hdac->DMA_Handle1, (uint32_t)pData, tmpreg, Length);
	}
#endif /* DAC_CHANNEL1_SUPPORT */
#endif 
	
	/* Process Unlocked */
	__HAL_UNLOCK(hdac);
	
	if (status == HAL_OK)
	{
		/* Enable the Peripheral */
		__HAL_DAC_ENABLE(hdac, Channel);
	}
	else
	{
		hdac->ErrorCode |= HAL_DAC_ERROR_DMA;
	}
	
	/* Return function status */
	return status;
}

/**
  * @brief  Handles DAC interrupt request
  *         This function uses the interruption of DMA
  *         underrun.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_IRQHandler(DAC_HandleTypeDef *hdac)
{
	if (__HAL_DAC_GET_IT_SOURCE(hdac, DAC_IT_DMAIT0))
	{
		/* Check underrun flag of DAC channel 0 */
		if (__HAL_DAC_GET_FLAG(hdac, DAC_FLAG_DMAIT0))
		{
		/* Change DAC state to error state */
		hdac->State = HAL_DAC_STATE_ERROR;
	
		/* Set DAC error code to channel0 DMA underrun error */
		SET_BIT(hdac->ErrorCode, HAL_DAC_ERROR_DMAUNDERRUNCH0);
	
		/* Clear the underrun flag */
		__HAL_DAC_CLEAR_FLAG(hdac, DAC_FLAG_DMAIT0);
	
		/* Disable the selected DAC channel0 DMA request */
		CLEAR_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAEN0);
	
		/* Error callback */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
		hdac->DMAUnderrunCallbackCh0(hdac);
#else
		HAL_DAC_DMAUnderrunCallbackCh0(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
		}
	}
#if defined(UM324xF)
#if defined(DAC_CHANNEL1_SUPPORT)
	if (__HAL_DAC_GET_IT_SOURCE(hdac, DAC_IT_DMAIT1))
	{
		/* Check underrun flag of DAC channel 1 */
		if (__HAL_DAC_GET_FLAG(hdac, DAC_FLAG_DMAIT1))
		{
			/* Change DAC state to error state */
			hdac->State = HAL_DAC_STATE_ERROR;
		
			/* Set DAC error code to channel1 DMA underrun error */
			SET_BIT(hdac->ErrorCode, HAL_DAC_ERROR_DMAUNDERRUNCH1);
		
			/* Clear the underrun flag */
			__HAL_DAC_CLEAR_FLAG(hdac, DAC_FLAG_DMAIT1);
		
			/* Disable the selected DAC channel1 DMA request */
			CLEAR_BIT(hdac->Instance->CTRL, DAC_CTRL_DMAEN1);
		
			/* Error callback */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
			hdac->DMAUnderrunCallbackCh1(hdac);
#else
			HAL_DACEx_DMAUnderrunCallbackCh1(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
		}
	}
#endif /* DAC_CHANNEL1_SUPPORT */
#endif 
}

/**
  * @brief  Set the specified data holding register value for DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_0: DAC Channel0 selected
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  * @param  Alignment Specifies the data alignment.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data Data to be loaded in the selected data holding register.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data)
{
    __IO uint32_t tmp = 0UL;

    tmp = (uint32_t)hdac->Instance;
    if (Channel == DAC_CHANNEL_0)
    {
        tmp += DAC_DHR12R0_ALIGNMENT(Alignment);
    }
#if defined(UM324xF)
#if defined(DAC_CHANNEL1_SUPPORT)
    else
    {
        tmp += DAC_DHR12R1_ALIGNMENT(Alignment);
    }
#endif /* DAC_CHANNEL1_SUPPORT */
#endif
    /* Set the DAC channel selected data holding register */
    *(__IO uint32_t *) tmp = Data;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Enable or disable the selected DAC channel wave generation.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_0: DAC Channel0 selected
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude)
{
	/* Process locked */
	__HAL_LOCK(hdac);
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_BUSY;
	
	/* Enable the triangle wave generation for the selected DAC channel */
	MODIFY_REG(hdac->Instance->CTRL, ((DAC_CTRL_WAVE0) | (DAC_CTRL_MAMP0)) << (Channel & 0x10UL),
				(DAC_CTRL_WAVE0_1 | Amplitude) << (Channel & 0x10UL));
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_READY;
	
	/* Process unlocked */
	__HAL_UNLOCK(hdac);
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Enable or disable the selected DAC channel wave generation.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_0: DAC Channel0 selected
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude)
{
	/* Process locked */
	__HAL_LOCK(hdac);
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_BUSY;
	
	/* Enable the triangle wave generation for the selected DAC channel */
	MODIFY_REG(hdac->Instance->CTRL, ((DAC_CTRL_WAVE0) | (DAC_CTRL_MAMP0)) << (Channel & 0x10UL),
				(DAC_CTRL_WAVE0_0 | Amplitude) << (Channel & 0x10UL));
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_READY;
	
	/* Process unlocked */
	__HAL_UNLOCK(hdac);
	
	/* Return function status */
	return HAL_OK;
}
#if defined(UM324xF)
#if defined(DAC_CHANNEL1_SUPPORT)
/**
  * @brief  Enables DAC and starts conversion of both channels.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DACEx_DualStart(DAC_HandleTypeDef *hdac)
{
	uint32_t tmp_swtrig = 0UL;
	
	/* Process locked */
	__HAL_LOCK(hdac);
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_BUSY;
	
	/* Enable the Peripheral */
	__HAL_DAC_ENABLE(hdac, DAC_CHANNEL_0);
	__HAL_DAC_ENABLE(hdac, DAC_CHANNEL_1);
	
	/* Check if software trigger enabled */
	if ((hdac->Instance->CTRL & (DAC_CTRL_TEN0 | DAC_CTRL_TSEL0)) == DAC_TRIGGER_SOFTWARE)
	{
		tmp_swtrig |= DAC_SWTRG_SWTRG0;
	}
	if ((hdac->Instance->CTRL & (DAC_CTRL_TEN1 | DAC_CTRL_TSEL1)) == (DAC_TRIGGER_SOFTWARE << (DAC_CHANNEL_1 & 0x10UL)))
	{
		tmp_swtrig |= DAC_SWTRG_SWTRG1;
	}
	/* Enable the selected DAC software conversion*/
	SET_BIT(hdac->Instance->SWTRG, tmp_swtrig);
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_READY;
	
	/* Process unlocked */
	__HAL_UNLOCK(hdac);
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Disables DAC and stop conversion of both channels.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DACEx_DualStop(DAC_HandleTypeDef *hdac)
{
	/* Disable the Peripheral */
	__HAL_DAC_DISABLE(hdac, DAC_CHANNEL_0);
	__HAL_DAC_DISABLE(hdac, DAC_CHANNEL_1);
	
	/* Change DAC state */
	hdac->State = HAL_DAC_STATE_READY;
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Set the specified data holding register value for dual DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *               the configuration information for the specified DAC.
  * @param  Alignment Specifies the data alignment for dual channel DAC.
  *          This parameter can be one of the following values:
  *            DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data0 Data for DAC Channel1 to be loaded in the selected data holding register.
  * @param  Data1 Data for DAC Channel2 to be loaded in the selected data  holding register.
  * @note   In dual mode, a unique register access is required to write in both
  *          DAC channels at the same time.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DACEx_DualSetValue(DAC_HandleTypeDef *hdac, uint32_t Alignment, uint32_t Data0, uint32_t Data1)
{
	uint32_t data;
	uint32_t tmp;

	/* Calculate and set dual DAC data holding register value */
	if (Alignment == DAC_ALIGN_8B_R)
	{
		data = ((uint32_t)Data1 << 8U) | Data0;
	}
	else
	{
		data = ((uint32_t)Data1 << 16U) | Data0;
	}
	
	tmp = (uint32_t)hdac->Instance;
	tmp += DAC_DHR12RD_ALIGNMENT(Alignment);
	
	/* Set the dual DAC selected data holding register */
	*(__IO uint32_t *)tmp = data;
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Return the last data output value of the selected DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval The selected DAC channel data output value.
  */
uint32_t HAL_DACEx_DualGetValue(DAC_HandleTypeDef *hdac)
{
	uint32_t tmp = 0UL;
	
	tmp |= hdac->Instance->DOR0;
	
	tmp |= hdac->Instance->DOR1 << 16UL;
	
	/* Returns the DAC channel data output register value */
	return tmp;
}

#endif
#endif

/**
  * @brief  Conversion complete callback in non-blocking mode for Channel0
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DAC_ConvCpltCallbackCh0(DAC_HandleTypeDef *hdac)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdac);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_DAC_ConvCpltCallbackCh1 could be implemented in the user file
     */
}

/**
  * @brief  Error DAC callback for Channel0.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DAC_ErrorCallbackCh0(DAC_HandleTypeDef *hdac)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdac);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_DAC_ErrorCallbackCh1 could be implemented in the user file
     */
}

/**
  * @brief  DMA underrun DAC callback for channel0.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DAC_DMAUnderrunCallbackCh0(DAC_HandleTypeDef *hdac)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdac);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_DAC_DMAUnderrunCallbackCh1 could be implemented in the user file
     */
}

/**
  * @brief  Conversion complete callback in non-blocking mode for Channel1.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DACEx_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_DACEx_ConvCpltCallbackCh1 could be implemented in the user file
   */
}

/**
  * @brief  Error DAC callback for Channel1.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DACEx_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_DACEx_ErrorCallbackCh1 could be implemented in the user file
   */
}

/**
  * @brief  DMA underrun DAC callback for Channel1.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
__weak void HAL_DACEx_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_DACEx_DMAUnderrunCallbackCh1 could be implemented in the user file
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
  * @brief  Configures the selected DAC channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  sConfig DAC configuration structure.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_0: DAC Channel0 selected
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac, DAC_ChannelConfTypeDef *sConfig, uint32_t Channel)
{
    uint32_t tmpreg1;
    uint32_t tmpreg2;

    /* Process locked */
    __HAL_LOCK(hdac);

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_BUSY;
	
    /* Get the DAC CTRL value */
    tmpreg1 = hdac->Instance->CTRL;
    /* Clear BUFx, TENx, TSELx, WAVEx and MAMPx bits */
    tmpreg1 &= ~(((uint32_t)(DAC_CTRL_MAMP0 | DAC_CTRL_WAVE0 | DAC_CTRL_TSEL0 | DAC_CTRL_TEN0 | DAC_CTRL_BUF0 | DAC_CTRL_DAC0_VREF_MODE)) << (Channel & 0x10UL));
    /* Configure for the selected DAC channel: buffer output, trigger */
    /* Set TSELx and TENx bits according to DAC_Trigger value */
    /* Set BUFx bit according to DAC_OutputBuffer value */
    tmpreg2 = (sConfig->DAC_Trigger | sConfig->DAC_OutputBuffer | sConfig->Vrefset);
    /* Calculate CR register value depending on DAC_Channel */
    tmpreg1 |= tmpreg2 << (Channel & 0x10UL);
    /* Write to DAC CTRL */
    hdac->Instance->CTRL = tmpreg1;
    /* Disable wave generation */
    CLEAR_BIT(hdac->Instance->CTRL, (DAC_CTRL_WAVE0 << (Channel & 0x10UL)));

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_READY;

    /* Process unlocked */
    __HAL_UNLOCK(hdac);

    /* Return function status */
    return HAL_OK;
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
  * @retval HAL state
  */
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef *hdac)
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
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac)
{
    return hdac->ErrorCode;
}

/**
  * @brief  DMA conversion complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAConvCpltCh0(DMA_HandleTypeDef *hdma)
{
	DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
	hdac->ConvCpltCallbackCh0(hdac);
#else
	HAL_DAC_ConvCpltCallbackCh0(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

	hdac->State = HAL_DAC_STATE_READY;
}

/**
  * @brief  DMA error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAErrorCh0(DMA_HandleTypeDef *hdma)
{
	DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	/* Set DAC error code to DMA error */
	hdac->ErrorCode |= HAL_DAC_ERROR_DMA;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
	hdac->ErrorCallbackCh0(hdac);
#else
	HAL_DAC_ErrorCallbackCh0(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

	hdac->State = HAL_DAC_STATE_READY;
}

#if defined(UM324xF)
#if defined(DAC_CHANNEL1_SUPPORT)
/**
  * @brief  DMA conversion complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef *hdac = (DAC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ConvCpltCallbackCh0(hdac);
#else
  HAL_DACEx_ConvCpltCallbackCh1(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

  hdac->State = HAL_DAC_STATE_READY;
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
  hdac->ErrorCode |= HAL_DAC_ERROR_DMA;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  hdac->ErrorCallbackCh0(hdac);
#else
  HAL_DACEx_ErrorCallbackCh1(hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

  hdac->State = HAL_DAC_STATE_READY;
}
#endif /* DAC_CHANNEL1_SUPPORT */
#endif 
/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_DAC_MODULE_ENABLED */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
