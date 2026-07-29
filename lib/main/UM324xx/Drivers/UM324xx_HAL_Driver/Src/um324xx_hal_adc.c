/**
  ******************************************************************************
  * @file     um324xx_hal_adc.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-19
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

/** @defgroup ADC_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup ADC_Private_Functions
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
static void ADC_Init(ADC_HandleTypeDef* hadc);
static void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma);
static void ADC_DMAError(DMA_HandleTypeDef *hdma);
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters 
  *         in the ADC_InitStruct and initializes the ADC MSP.
  *           
  * @note   This function is used to configure the global features of the ADC ( 
  *         ClockPrescaler, number of conversion), however,
  *         the rest of the configuration parameters are specific to the regular
  *         channels group (scan mode activation, continuous mode activation,
  *         External trigger source and edge).
  *             
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef tmp_hal_status = HAL_OK;
	
	/* Check ADC handle */
	if(hadc == NULL)
	{
		return HAL_ERROR;
	}
	
	if(hadc->State == HAL_ADC_STATE_RESET)
	{
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
		/* Init the ADC Callback settings */
		hadc->ConvCpltCallback              = HAL_ADC_ConvCpltCallback;                 /* Legacy weak callback */
		hadc->LevelOutOfWindowCallback      = HAL_ADC_LevelOutOfWindowCallback;         /* Legacy weak callback */
		hadc->ErrorCallback                 = HAL_ADC_ErrorCallback;                    /* Legacy weak callback */
		hadc->InjectedConvCpltCallback      = HAL_ADCEx_InjectedConvCpltCallback;       /* Legacy weak callback */
		if (hadc->MspInitCallback == NULL)
		{
			hadc->MspInitCallback = HAL_ADC_MspInit; /* Legacy weak MspInit  */
		}
	
		/* Init the low level hardware */
		hadc->MspInitCallback(hadc);
#else
		/* Init the low level hardware */
		HAL_ADC_MspInit(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

		/* Initialize ADC error code */
		ADC_CLEAR_ERRORCODE(hadc);
		
		/* Allocate lock resource and initialize it */
		hadc->Lock = HAL_UNLOCKED;
	}
  
	/* Configuration of ADC parameters if previous preliminary actions are      */
	/* correctly completed.                                                     */
	if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
	{
		/* Set ADC state */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
						HAL_ADC_STATE_BUSY_INTERNAL);
		
		/* Set ADC parameters */
		ADC_Init(hadc);
		
		/* Set ADC error code to none */
		ADC_CLEAR_ERRORCODE(hadc);
		
		/* Set the ADC state */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_BUSY_INTERNAL,
						HAL_ADC_STATE_READY);
	}
	else
	{
		tmp_hal_status = HAL_ERROR;
	}
	
	/* Release Lock */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return tmp_hal_status;
}

/**
  * @brief  Deinitializes the ADCx peripheral registers to their default reset values. 
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef tmp_hal_status = HAL_OK;
	
	/* Check ADC handle */
	if(hadc == NULL)
	{
		return HAL_ERROR;
	}

	/* Set ADC state */
	SET_BIT(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL);
	
	/* Stop potential conversion on going, on regular and injected groups */
	/* Disable ADC peripheral */
	__HAL_ADC_DISABLE(hadc);
	
	/* Configuration of ADC parameters if previous preliminary actions are      */ 
	/* correctly completed.                                                     */
	if(HAL_IS_BIT_CLR(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
		if (hadc->MspDeInitCallback == NULL)
		{
			hadc->MspDeInitCallback = HAL_ADC_MspDeInit; /* Legacy weak MspDeInit  */
		}
		
		/* DeInit the low level hardware: RCC clock, NVIC */
		hadc->MspDeInitCallback(hadc);
#else
		/* DeInit the low level hardware: RCC clock, NVIC */
		HAL_ADC_MspDeInit(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
		
		/* Set ADC error code to none */
		ADC_CLEAR_ERRORCODE(hadc);
		
		/* Set ADC state */
		hadc->State = HAL_ADC_STATE_RESET;
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return tmp_hal_status;
}

#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User ADC Callback
  *         To be used instead of the weak predefined callback
  * @param  hadc Pointer to a ADC_HandleTypeDef structure that contains
  *                the configuration information for the specified ADC.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_ADC_CONVERSION_COMPLETE_CB_ID      ADC conversion complete callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_CB_ID      ADC analog watchdog callback ID
  *          @arg @ref HAL_ADC_ERROR_CB_ID                    ADC error callback ID
  *          @arg @ref HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID  ADC group injected conversion complete callback ID
  *          @arg @ref HAL_ADC_MSPINIT_CB_ID                  ADC Msp Init callback ID
  *          @arg @ref HAL_ADC_MSPDEINIT_CB_ID                ADC Msp DeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_RegisterCallback(ADC_HandleTypeDef *hadc, HAL_ADC_CallbackIDTypeDef CallbackID, pADC_CallbackTypeDef pCallback)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	if (pCallback == NULL)
	{
		/* Update the error code */
		hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;
	
		return HAL_ERROR;
	}
	
	if ((hadc->State & HAL_ADC_STATE_READY) != 0UL)
	{
		switch (CallbackID)
		{
			case HAL_ADC_CONVERSION_COMPLETE_CB_ID :
				hadc->ConvCpltCallback = pCallback;
				break;
		
			case HAL_ADC_LEVEL_OUT_OF_WINDOW_CB_ID :
				hadc->LevelOutOfWindowCallback = pCallback;
				break;
		
			case HAL_ADC_ERROR_CB_ID :
				hadc->ErrorCallback = pCallback;
				break;
		
			case HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID :
				hadc->InjectedConvCpltCallback = pCallback;
				break;
		
			case HAL_ADC_MSPINIT_CB_ID :
				hadc->MspInitCallback = pCallback;
				break;
		
			case HAL_ADC_MSPDEINIT_CB_ID :
				hadc->MspDeInitCallback = pCallback;
				break;
		
			default :
				/* Update the error code */
				hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;
		
				/* Return error status */
				status = HAL_ERROR;
				break;
		}
	}
	else if (HAL_ADC_STATE_RESET == hadc->State)
	{
		switch (CallbackID)
		{
			case HAL_ADC_MSPINIT_CB_ID :
				hadc->MspInitCallback = pCallback;
				break;
		
			case HAL_ADC_MSPDEINIT_CB_ID :
				hadc->MspDeInitCallback = pCallback;
				break;
		
			default :
				/* Update the error code */
				hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;
		
				/* Return error status */
				status = HAL_ERROR;
				break;
			}
	}
	else
	{
		/* Update the error code */
		hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;
	
		/* Return error status */
		status =  HAL_ERROR;
	}
	
	return status;
}

/**
  * @brief  Unregister a ADC Callback
  *         ADC callback is redirected to the weak predefined callback
  * @param  hadc Pointer to a ADC_HandleTypeDef structure that contains
  *                the configuration information for the specified ADC.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_ADC_CONVERSION_COMPLETE_CB_ID      ADC conversion complete callback ID
  *          @arg @ref HAL_ADC_LEVEL_OUT_OF_WINDOW_CB_ID      ADC analog watchdog callback ID
  *          @arg @ref HAL_ADC_ERROR_CB_ID                    ADC error callback ID
  *          @arg @ref HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID  ADC group injected conversion complete callback ID
  *          @arg @ref HAL_ADC_MSPINIT_CB_ID                  ADC Msp Init callback ID
  *          @arg @ref HAL_ADC_MSPDEINIT_CB_ID                ADC Msp DeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_UnRegisterCallback(ADC_HandleTypeDef *hadc, HAL_ADC_CallbackIDTypeDef CallbackID)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	if ((hadc->State & HAL_ADC_STATE_READY) != 0UL)
	{
		switch (CallbackID)
		{
			case HAL_ADC_CONVERSION_COMPLETE_CB_ID :
				hadc->ConvCpltCallback = HAL_ADC_ConvCpltCallback;
				break;
		
			case HAL_ADC_LEVEL_OUT_OF_WINDOW_CB_ID :
				hadc->LevelOutOfWindowCallback = HAL_ADC_LevelOutOfWindowCallback;
				break;
		
			case HAL_ADC_ERROR_CB_ID :
				hadc->ErrorCallback = HAL_ADC_ErrorCallback;
				break;
		
			case HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID :
				hadc->InjectedConvCpltCallback = HAL_ADCEx_InjectedConvCpltCallback;
				break;
		
			case HAL_ADC_MSPINIT_CB_ID :
				hadc->MspInitCallback = HAL_ADC_MspInit; /* Legacy weak MspInit              */
				break;
		
			case HAL_ADC_MSPDEINIT_CB_ID :
				hadc->MspDeInitCallback = HAL_ADC_MspDeInit; /* Legacy weak MspDeInit            */
				break;
		
			default :
				/* Update the error code */
				hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;
		
				/* Return error status */
				status =  HAL_ERROR;
				break;
		}
	}
	else if (HAL_ADC_STATE_RESET == hadc->State)
	{
		switch (CallbackID)
		{
			case HAL_ADC_MSPINIT_CB_ID :
				hadc->MspInitCallback = HAL_ADC_MspInit;                   /* Legacy weak MspInit              */
				break;
		
			case HAL_ADC_MSPDEINIT_CB_ID :
				hadc->MspDeInitCallback = HAL_ADC_MspDeInit;               /* Legacy weak MspDeInit            */
				break;
		
			default :
				/* Update the error code */
				hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;
		
				/* Return error status */
				status =  HAL_ERROR;
				break;
		}
	}
	else
	{
		/* Update the error code */
		hadc->ErrorCode |= HAL_ADC_ERROR_INVALID_CALLBACK;
	
		/* Return error status */
		status =  HAL_ERROR;
	}
	
	return status;
}
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */

/**
  * @brief  Initializes the ADC MSP.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval None
  */
__weak void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_MspInit could be implemented in the user file
   */ 
}

/**
  * @brief  DeInitializes the ADC MSP.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval None
  */
__weak void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_MspDeInit could be implemented in the user file
   */ 
}

/** @defgroup ADC_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions 
 *
@verbatim   
 ===============================================================================
             ##### IO operation functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Start conversion of regular channel.
      (+) Stop conversion of regular channel.
      (+) Start conversion of regular channel and enable interrupt.
      (+) Stop conversion of regular channel and disable interrupt.
      (+) Start conversion of regular channel and enable DMA transfer.
      (+) Stop conversion of regular channel and disable DMA transfer.
      (+) Handle ADC interrupt request. 
               
@endverbatim
  * @{
  */

/**
  * @brief  Enables ADC and starts conversion of the regular channels.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc)
{
	__IO uint32_t counter = 0U;
  
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Enable the ADC peripheral */
	/* Check if ADC peripheral is disabled in order to enable it and wait during 
	Tstab time the ADC's stabilization */
	if((hadc->Instance->DGCTRL & ADC_DGCTRL_ADCC_EN) != ADC_DGCTRL_ADCC_EN)
	{
		/* Enable the Peripheral */
		__HAL_ADC_ENABLE(hadc);
		
		/* Delay for ADC stabilization time */
		/* Compute number of CPU cycles to wait for */
		counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
		while(counter != 0U)
		{
			counter--;
		}
	}
	
	/* Start conversion if ADC is effectively enabled */
	if(HAL_IS_BIT_SET(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Set ADC state                                                          */
		/* - Clear state bitfield related to regular group conversion results     */
		/* - Set state bitfield related to regular group operation                */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
						HAL_ADC_STATE_REG_BUSY);
	
		/* Process unlocked */
		/* Unlock before starting ADC conversions: in case of potential           */
		/* interruption, to let the process to ADC IRQ Handler.                   */
		__HAL_UNLOCK(hadc);

		/* Clear regular group conversion flag and overrun flag */
		/* (To ensure of no unknown state from potential previous ADC operations) */
		__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EORC);
	
		/* Enable the selected ADC software conversion for regular group */
		hadc->Instance->ANCTRL |= (uint32_t)ADC_ANCTRL_RGL_START;
	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
	
		/* Set ADC error code to ADC IP internal error */
		SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
	}
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Disables ADC and stop conversion of regular channels.
  * 
  * @note   Caution: This function will stop also injected channels.  
  *
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc)
{
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Stop potential conversion on going, on regular and injected groups */
	/* Disable ADC peripheral */
	__HAL_ADC_DISABLE(hadc);
	
	/* Check if ADC is effectively disabled */
	if(HAL_IS_BIT_CLR(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Set ADC state */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
						HAL_ADC_STATE_READY);
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Poll for regular conversion complete
  * @note   ADC conversion flags EOS (end of sequence) and EORC (end of
  *         conversion) are cleared by this function.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  Timeout Timeout value in millisecond.  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout)
{
	uint32_t tickstart = 0U;
	
	/* Get tick */
	tickstart = HAL_GetTick();
	
	/* Check End of conversion flag */
	while(!(__HAL_ADC_GET_IT_FLAG(hadc, ADC_IT_FLAG_EORC)))
	{
		/* Check if timeout is disabled (set to infinite wait) */
		if(Timeout != HAL_MAX_DELAY)
		{
			if((Timeout == 0U) || ((HAL_GetTick() - tickstart ) > Timeout))
			{
				/* New check to avoid false timeout detection in case of preemption */
				if(!(__HAL_ADC_GET_IT_FLAG(hadc, ADC_IT_FLAG_EORC)))
				{
					/* Update ADC state machine to timeout */
					SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);
					
					/* Process unlocked */
					__HAL_UNLOCK(hadc);
					
					return HAL_TIMEOUT;
				}
			}
		}
	}
	
	/* Clear regular group conversion flag */
	__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EORC);
	
	/* Update ADC state */
	SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);
	CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);
	
	/* Return ADC state */
	return HAL_OK;
}

/**
  * @brief  Poll for conversion event                                           
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains         
  *         the configuration information for the specified ADC.                
  * @param  EventType the ADC event type.                                       
  *          This parameter can be one of the following values:                 
  *            @arg ADC_WDG_INJ_EVENT  : ADC injection Analog watch Dog event.                                           
  *            @arg ADC_WDG_RGL_EVENT  : ADC regular Analog watch Dog event.                  
  *            @arg ADC_WDG_CH_EVENT   : ADC Channel Data Analog watch Dog  event.
  *            @arg ADC_FIFO_OVF_EVENT : ADC RXFIFO Overrun event.
  *            @arg ADC_FIFO_AVL_EVENT : ADC RXFIFO available event.
  *            @arg ADC_EOIC_EVENT     : ADC injection conversion end event.
  *            @arg ADC_EORC_EVENT     : ADC regular conversion end event.
  * @param  Timeout Timeout value in millisecond.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout)
{
	uint32_t tickstart = 0U;
	
	/* Get tick */
	tickstart = HAL_GetTick();
	
	/* Check selected event flag */
	while(!(__HAL_ADC_GET_IT_FLAG(hadc,EventType)))
	{
		/* Check for the Timeout */
		if(Timeout != HAL_MAX_DELAY)
		{
			if((Timeout == 0U) || ((HAL_GetTick() - tickstart ) > Timeout))
			{
				/* New check to avoid false timeout detection in case of preemption */
				if(!(__HAL_ADC_GET_IT_FLAG(hadc,EventType)))
				{
					/* Update ADC state machine to timeout */
					SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);
					
					/* Process unlocked */
					__HAL_UNLOCK(hadc);
					
					return HAL_TIMEOUT;
				}
			}
		}
	}
	
	/* Return ADC state */
	return HAL_OK;
}

/**
  * @brief  Enables the interrupt and starts ADC conversion of regular channels.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc)
{
	__IO uint32_t counter = 0U;
	
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Enable the ADC peripheral */
	/* Check if ADC peripheral is disabled in order to enable it and wait during 
	Tstab time the ADC's stabilization */
	if((hadc->Instance->DGCTRL & ADC_DGCTRL_ADCC_EN) != ADC_DGCTRL_ADCC_EN)
	{
		/* Enable the Peripheral */
		__HAL_ADC_ENABLE(hadc);
		
		/* Delay for ADC stabilization time */
		/* Compute number of CPU cycles to wait for */
		counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
		while(counter != 0U)
		{
			counter--;
		}
	}
	
	/* Start conversion if ADC is effectively enabled */
	if(HAL_IS_BIT_SET(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Set ADC state                                                          */
		/* - Clear state bitfield related to regular group conversion results     */
		/* - Set state bitfield related to regular group operation                */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
						HAL_ADC_STATE_REG_BUSY);
	
		/* Process unlocked */
		/* Unlock before starting ADC conversions: in case of potential           */
		/* interruption, to let the process to ADC IRQ Handler.                   */
		__HAL_UNLOCK(hadc);

		/* Clear regular group conversion flag and overrun flag */
		/* (To ensure of no unknown state from potential previous ADC operations) */
		__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EORC);
				
		/* Enable end of conversion interrupt for regular group */
		__HAL_ADC_ENABLE_IT(hadc, ADC_IT_EORC);
		
		/* Enable the selected ADC software conversion for regular group */
		hadc->Instance->ANCTRL |= (uint32_t)ADC_ANCTRL_RGL_START;
	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
	
		/* Set ADC error code to ADC IP internal error */
		SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
	}
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Disables the interrupt and stop ADC conversion of regular channels.
  * 
  * @note   Caution: This function will stop also injected channels.
  *
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc)
{
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Stop potential conversion on going, on regular and injected groups */
	/* Disable ADC peripheral */
	__HAL_ADC_DISABLE(hadc);
	
	/* Check if ADC is effectively disabled */
	if(HAL_IS_BIT_CLR(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Disable ADC end of conversion interrupt for regular group */
		__HAL_ADC_DISABLE_IT(hadc, ADC_IT_EORC);
	
		/* Set ADC state */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
						HAL_ADC_STATE_READY);
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Handles ADC interrupt request
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	
	uint32_t tmp_int = hadc->Instance->INTEN;
	uint32_t tmp_intstat = hadc->Instance->INTSTAT;
	
	tmp1 = tmp_intstat & ADC_IT_FLAG_EORC;
	tmp2 = tmp_int & ADC_IT_EORC;
	/* Check End of conversion flag for regular channels */
	if(tmp1 && tmp2)
	{
		/* Update state machine on conversion status if not in error state */
		if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
		{
			/* Set ADC state */
			SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);
		}
		
		if((hadc->Instance->DGCTRL & ADC_DGCTRL_RGL_TRIG_EDG) == ADC_EXTERNALTRIGCONVEDGE_NONE)
		{
			/* Disable ADC end of single conversion interrupt on group regular */
			__HAL_ADC_DISABLE_IT(hadc, ADC_IT_EORC);
		}
		
		/* Set ADC state */
		CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);
		
		if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
		{
			SET_BIT(hadc->State, HAL_ADC_STATE_READY);
		}
	}
    
	/* Clear regular group conversion flag */
	__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EORC);	
		/* Conversion complete callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
		hadc->ConvCpltCallback(hadc);
#else
		HAL_ADC_ConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
		
	tmp1 = tmp_intstat & ADC_IT_FLAG_EOIC;
	tmp2 = tmp_int & ADC_IT_EOIC;
	/* Check End of conversion flag for injected channels */
	if(tmp1 && tmp2)
	{
		/* Update state machine on conversion status if not in error state */
		if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
		{
			/* Set ADC state */
			SET_BIT(hadc->State, HAL_ADC_STATE_INJ_EOC);
		}
		
		if((hadc->Instance->DGCTRL & ADC_DGCTRL_INJ_TRIG_EDG) == ADC_EXTERNALTRIGINJCONVEDGE_NONE)
		{
			/* Disable ADC end of single conversion interrupt on group injected */
			__HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOIC);
		}
		
		/* Set ADC state */
		CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
		
		if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
		{
			SET_BIT(hadc->State, HAL_ADC_STATE_READY);
		}
	}
	/* Clear injected group conversion flag */
	__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EOIC);	
		/* Conversion complete callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
		hadc->InjectedConvCpltCallback(hadc);
#else
		HAL_ADCEx_InjectedConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
		
	tmp1 = tmp_intstat & (ADC_IT_FLAG_WDG_INJ | ADC_IT_FLAG_WDG_RGL | ADC_IT_FLAG_WDG_CH);
	tmp2 = tmp_int & (ADC_IT_WDG_INJ | ADC_IT_WDG_RGL | ADC_IT_WDG_CH);
	/* Check Analog watchdog flag */
	if(tmp1 && tmp2)
	{
		if(__HAL_ADC_GET_IT_FLAG(hadc, ADC_IT_FLAG_WDG_RGL))
		{
			/* Set ADC state */
			SET_BIT(hadc->State, HAL_ADC_STATE_AWD_RGL);

			/* Clear the ADC analog watchdog flag */
			__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_WDG_RGL);
		}
		
		if(__HAL_ADC_GET_IT_FLAG(hadc, ADC_IT_FLAG_WDG_INJ))
		{
			/* Set ADC state */
			SET_BIT(hadc->State, HAL_ADC_STATE_AWD_INJ);

			/* Clear the ADC analog watchdog flag */
			__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_WDG_INJ);
		}
		
		if(__HAL_ADC_GET_IT_FLAG(hadc, ADC_IT_FLAG_WDG_CH))
		{
			/* Set ADC state */
			SET_BIT(hadc->State, HAL_ADC_STATE_AWD_CH);

			/* Clear the ADC analog watchdog flag */
			__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_WDG_CH);
		}
		
		/* Level out of window callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
		hadc->LevelOutOfWindowCallback(hadc);
#else
		HAL_ADC_LevelOutOfWindowCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
	}
}

/**
  * @brief  Enables ADC DMA request after last transfer (Single-ADC mode) and enables ADC peripheral  
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from ADC peripheral to memory.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
{
	__IO uint32_t counter = 0U;
	
	/* Process locked */
	__HAL_LOCK(hadc);
	
	
	/* Enable the ADC peripheral */
	/* Check if ADC peripheral is disabled in order to enable it and wait during 
	Tstab time the ADC's stabilization */
	if((hadc->Instance->DGCTRL & ADC_DGCTRL_ADCC_EN) != ADC_DGCTRL_ADCC_EN)
	{
		/* Enable the Peripheral */
		__HAL_ADC_ENABLE(hadc);
		
		/* Delay for ADC stabilization time */
		/* Compute number of CPU cycles to wait for */
		counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
		while(counter != 0U)
		{
			counter--;
		}
	}
	
	/* Check ADC DMA Mode                                                     */
	/* - disable the DMA Mode if it is already enabled                        */
	if(((hadc->Instance->DGCTRL & ADC_DGCTRL_DMA_MOD) == ADC_DGCTRL_DMA_MOD_0) ||\
	      ((hadc->Instance->DGCTRL & ADC_DGCTRL_DMA_MOD) == ADC_DGCTRL_DMA_MOD_1))
	{
		CLEAR_BIT(hadc->Instance->DGCTRL, ADC_DGCTRL_DMA_MOD);
	}
	
	
	/* Start conversion if ADC is effectively enabled */
	if(HAL_IS_BIT_SET(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Set ADC state                                                          */
		/* - Clear state bitfield related to regular group conversion results     */
		/* - Set state bitfield related to regular group operation                */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC,
						HAL_ADC_STATE_REG_BUSY);
		
		/* State machine update: Check if an injected conversion is ongoing */
		if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
		{
			/* Reset ADC error code fields related to conversions on group regular */
			CLEAR_BIT(hadc->ErrorCode,  HAL_ADC_ERROR_DMA);
		}
		else
		{
			/* Reset ADC all error code fields */
			ADC_CLEAR_ERRORCODE(hadc);
		}
		
		/* Process unlocked */
		/* Unlock before starting ADC conversions: in case of potential           */
		/* interruption, to let the process to ADC IRQ Handler.                   */
		__HAL_UNLOCK(hadc);
	
		/* Set the DMA transfer complete callback */
		hadc->DMA_Handle->XferBlockCallback = ADC_DMAConvCplt;
		
		/* Set the DMA error callback */
		hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;
	
		/* Manage ADC and DMA start:  DMA start, ADC     						  */
		/* start (in case of SW start):                                           */
		/* Clear regular group conversion flag and overrun flag */
		/* (To ensure of no unknown state from potential previous ADC operations) */
		__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EORC);
		
		/* Enable ADC FIFO */
		hadc->Instance->DGCTRL |= ADC_DGCTRL_FIFO_EN;

		/* Enable ADC DMA mode1 */
		hadc->Instance->DGCTRL |= ADC_DGCTRL_DMA_MOD_0;
		#if defined(UM324xF)
		if(Length == ADC_REGULAR_LENGTH_16)
		{
			/* Set ADC watermark */
			MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_WATERMARK, ADC_DGCTRL_WATERMARK);
			/* Set DMA source burst */
          
            MODIFY_REG(*(&(hadc->DMA_Handle->Instance->CTL0)  + (CHANNEL_OFFSET*hadc->DMA_Handle->DmaChannelSel)), DMA_CTL0_SRC_MSIZE, DMA_CTL0_SRC_MSIZE_0 | DMA_CTL0_SRC_MSIZE_1);
         
		}
		else if(Length == ADC_REGULAR_LENGTH_8)
		{
			/* Set ADC watermark */
			MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_WATERMARK, ADC_DGCTRL_WATERMARK_1);
			/* Set DMA source burst */
            
            MODIFY_REG(*(&(hadc->DMA_Handle->Instance->CTL0)  + (CHANNEL_OFFSET*hadc->DMA_Handle->DmaChannelSel)), DMA_CTL0_SRC_MSIZE, DMA_CTL0_SRC_MSIZE_1);
          
		}
		else if(Length == ADC_REGULAR_LENGTH_4)
		{
			/* Set ADC watermark */
			MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_WATERMARK, ADC_DGCTRL_WATERMARK_0);
			/* Set DMA source burst */
           
            MODIFY_REG(*(&(hadc->DMA_Handle->Instance->CTL0)  + (CHANNEL_OFFSET*hadc->DMA_Handle->DmaChannelSel)), DMA_CTL0_SRC_MSIZE, DMA_CTL0_SRC_MSIZE_0);
           
		}
		else
		{
			/* Set ADC watermark */
			CLEAR_BIT(hadc->Instance->DGCTRL, ADC_DGCTRL_WATERMARK);
			/* Set DMA source burst */
          
            CLEAR_BIT(*(&(hadc->DMA_Handle->Instance->CTL0)  + (CHANNEL_OFFSET*hadc->DMA_Handle->DmaChannelSel)), DMA_CTL0_SRC_MSIZE);
          
		}
		#endif
		
		/* Start the DMA channel */
		HAL_DMA_Start(hadc->DMA_Handle, (uint32_t)&hadc->Instance->FIFO_OUT, (uint32_t)pData, Length); 

		/* Enable the selected ADC software conversion for regular group */
		hadc->Instance->ANCTRL |= (uint32_t)ADC_ANCTRL_RGL_START;
	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
	
		/* Set ADC error code to ADC IP internal error */
		SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
	}
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Disables ADC DMA (Single-ADC mode) and disables ADC peripheral    
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef tmp_hal_status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Stop potential conversion on going, on regular and injected groups */
	/* Disable ADC peripheral */
	__HAL_ADC_DISABLE(hadc);
	
	/* Check if ADC is effectively disabled */
	if(HAL_IS_BIT_CLR(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Disable the selected ADC DMA mode */
		hadc->Instance->DGCTRL &= ~ADC_DGCTRL_DMA_MOD;
		
		/* Disable the DMA channel */
		if (hadc->DMA_Handle->State == HAL_DMA_STATE_BUSY)
		{
			tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);
			
			/* Check if DMA channel effectively disabled */
			if (tmp_hal_status != HAL_OK)
			{
				/* Update ADC state machine to error */
				SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);
			}
		}
		
		/* Set ADC state */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
						HAL_ADC_STATE_READY);
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return tmp_hal_status;
}

/**
  * @brief  Gets the converted value from data register of regular channel.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  channel  ADC channel
  * @retval Converted value
  */
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc, uint32_t channel)
{       
	/* Return the selected ADC converted value */
	return hadc->Instance->CHDAT[channel];
}

/**
  * @brief  Returns the last ADC0, ADC1 conversions results
  *         data in the dual mode.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval The converted data value.
  */
uint32_t HAL_ADCEx_DualModeGetValue(ADC_HandleTypeDef* hadc)
{
	/* Return the dual mode conversion value */
	return hadc->Instance->DUALDAT;
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
__weak void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode 
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
__weak void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Analog watchdog callback in non blocking mode 
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
__weak void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_LevelOoutOfWindowCallback could be implemented in the user file
   */
}

/**
  * @brief  Error ADC callback.
  * @note   In case of error due to overrun when using ADC with DMA transfer 
  *         (HAL ADC handle parameter "ErrorCode" to state "HAL_ADC_ERROR_OVR"):
  *         - Reinitialize the DMA using function "HAL_ADC_Stop_DMA()".
  *         - If needed, restart a new ADC conversion using function
  *           "HAL_ADC_Start_DMA()"
  *           (this function is also clearing overrun flag)
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
__weak void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  Injected conversion complete callback in non blocking mode 
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
__weak void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_InjectedConvCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Configures for the selected ADC regular channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  sConfig ADC configuration structure. 
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig)
{
	/* Process locked */
	__HAL_LOCK(hadc);
		
    #if defined(UM32x42x)
    if((sConfig->Channel==ADC_CHANNEL_14) || (sConfig->Channel==ADC_CHANNEL_15))
    {
        __HAL_PMU_UNLOCK_REGISTER();
        PMU->SASR |= (0x01<<(sConfig->Channel+6));
        __HAL_PMU_LOCK_REGISTER();
        
        SYSCFG->MISCCR |= (0x01<<(sConfig->Channel-6));
    }
    #endif
    
	/* if ADC_Channel_10 ... ADC_Channel_19 is selected */
	if (sConfig->Channel > ADC_CHANNEL_9)
	{
		/* Clear the old average times */
		hadc->Instance->CHAVGCFG1 &= ~ADC_CHAVGCFG1(ADC_CHAVGCFG1_CH10_AVG_SEL, sConfig->Channel);
		
		/* Set the new average times */
		hadc->Instance->CHAVGCFG1 |= ADC_CHAVGCFG1(sConfig->AverageTimes, sConfig->Channel);
	}
	else /* ADC_Channel include in ADC_Channel_[0..9] */
	{
		/* Clear the old average times */
		hadc->Instance->CHAVGCFG0 &= ~ADC_CHAVGCFG0(ADC_CHAVGCFG0_CH0_AVG_SEL, sConfig->Channel);
		
		/* Set the new average times */
		hadc->Instance->CHAVGCFG0 |= ADC_CHAVGCFG0(sConfig->AverageTimes, sConfig->Channel);
	}
	
	/* For Rank 1 to 6 */
	if (sConfig->Rank < 7U)
	{
		/* Clear the old RGL_CHx_SEL bits for the selected rank */
		hadc->Instance->RGLCHCFG0 &= ~ADC_RGLCHCFG0_RK(ADC_RGLCHCFG0_RGL_CH1_SEL, sConfig->Rank);
		
		/* Set the RGL_CHx_SEL bits bits for the selected rank */
		hadc->Instance->RGLCHCFG0 |= ADC_RGLCHCFG0_RK(sConfig->Channel, sConfig->Rank);
	}
	/* For Rank 7 to 12 */
	else if (sConfig->Rank < 13U)
	{
		/* Clear the old RGL_CHx_SEL bits for the selected rank */
		hadc->Instance->RGLCHCFG1 &= ~ADC_RGLCHCFG1_RK(ADC_RGLCHCFG1_RGL_CH7_SEL, sConfig->Rank);
		
		/* Set the RGL_CHx_SEL bits bits for the selected rank */
		hadc->Instance->RGLCHCFG1 |= ADC_RGLCHCFG1_RK(sConfig->Channel, sConfig->Rank);
	}
	/* For Rank 13 to 18 */
	else  if (sConfig->Rank < 19U)
	{
		/* Clear the old RGL_CHx_SEL bits for the selected rank */
		hadc->Instance->RGLCHCFG2 &= ~ADC_RGLCHCFG2_RK(ADC_RGLCHCFG2_RGL_CH13_SEL, sConfig->Rank);
		
		/* Set the RGL_CHx_SEL bits bits for the selected rank */
		hadc->Instance->RGLCHCFG2 |= ADC_RGLCHCFG2_RK(sConfig->Channel, sConfig->Rank);
	}
    /* For Rank 19 to 20 */
    else
    {
		/* Clear the old RGL_CHx_SEL bits for the selected rank */
		hadc->Instance->RGLCHCFG3 &= ~ADC_RGLCHCFG3_RK(ADC_RGLCHCFG3_RGL_CH19_SEL, sConfig->Rank);
		
		/* Set the RGL_CHx_SEL bits bits for the selected rank */
		hadc->Instance->RGLCHCFG3 |= ADC_RGLCHCFG3_RK(sConfig->Channel, sConfig->Rank);
	}    

	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Configures the analog watchdog.
  * @note Analog watchdog thresholds can be modified while ADC conversion
  * is on going.
  * In this case, some constraints must be taken into account:
  * The programmed threshold values are effective from the next
  * ADC EOC (end of unitary conversion).
  * Considering that registers write delay may happen due to
  * bus activity, this might cause an uncertainty on the
  * effective timing of the new programmed threshold values.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  AnalogWDGConfig  pointer to an ADC_AnalogWDGConfTypeDef structure 
  *         that contains the configuration information of ADC analog watchdog.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig)
{
	/* Process locked */
	__HAL_LOCK(hadc);
	
	if(AnalogWDGConfig->ITMode == ENABLE)
	{
		if(AnalogWDGConfig->WatchdogITMode == ADC_IT_WDG_RGL)
		{
			/* Enable the ADC Analog watchdog regular interrupt */
			__HAL_ADC_ENABLE_IT(hadc, ADC_IT_WDG_RGL);
		}
		if(AnalogWDGConfig->WatchdogITMode == ADC_IT_WDG_INJ)
		{
			/* Enable the ADC Analog watchdog injected interrupt */
			__HAL_ADC_ENABLE_IT(hadc, ADC_IT_WDG_INJ);
		}
		if(AnalogWDGConfig->WatchdogITMode == ADC_IT_WDG_RGL)
		{
			/* Enable the ADC Analog watchdog channel interrupt */
			__HAL_ADC_ENABLE_IT(hadc, ADC_IT_WDG_CH);
		}

	}
	else
	{
		/* Disable the ADC Analog watchdog interrupt */
		__HAL_ADC_DISABLE_IT(hadc, ADC_IT_WDG_RGL | ADC_IT_WDG_INJ | ADC_IT_WDG_CH);
	}

	/* Set the high threshold and the low threshold */
	hadc->Instance->WDGCOND = ADC_WDGCOND_HIGHTHRESHOLD(AnalogWDGConfig->HighThreshold) | AnalogWDGConfig->LowThreshold;
	
	/* Clear the Analog watchdog channel select bits */
	hadc->Instance->WDGEN &= ~ADC_WDGEN_FULL_Msk;
	
	/* Set the Analog watchdog channel */
	hadc->Instance->WDGEN |= (uint32_t)((uint16_t)(ADC_WDGCOND_CHANNEL(AnalogWDGConfig->Channel)));
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;
}

/** @defgroup ADC_Exported_Functions_Group4 ADC Peripheral State functions
 *  @brief   ADC Peripheral State functions 
 *
@verbatim   
 ===============================================================================
            ##### Peripheral State and errors functions #####
 ===============================================================================  
    [..]
    This subsection provides functions allowing to
      (+) Check the ADC state
      (+) Check the ADC Error
         
@endverbatim
  * @{
  */
  
/**
  * @brief  return the ADC state
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL state
  */
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef* hadc)
{
	/* Return ADC state */
	return hadc->State;
}

/**
  * @brief  Return the ADC error code
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval ADC Error Code
  */
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc)
{
	return hadc->ErrorCode;
}

/**
  * @}
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
  * @brief  selected ADC trig of   the regular mode.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  trig  the trig select of syscfg 
    *          This parameter can be one of the following values:
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_IT11_syscfg
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_IT10_syscfg
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_IT6_syscfg
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_lptim0out_syscfg 
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_lptim1out_syscfg
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_acmp0out_syscfg
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_acmp1out_syscfg
    *            @arg  ADC_RGLEXTERNALTRIGCONV_Ext_acmp2out_syscfg
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_EXT_RGL_TRIG_SYSCFG(ADC_HandleTypeDef* hadc,uint32_t trig)
{
  /* Process locked */
	__HAL_LOCK(hadc);

    MODIFY_REG(SYSCFG->ADCETSR, SYSCFG_ADCETSR_RGL_TRIG_SEL, trig);  
    
  	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;  
}


/**
  * @brief  selected ADC trig of   the inject mode.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  trig  the trig select of syscfg 
    *          This parameter can be one of the following values:
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_IT15_syscfg
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_IT9_syscfg
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_IT5_syscfg
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_lptim0out_syscfg 
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_lptim1out_syscfg
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_acmp0out_syscfg
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_acmp1out_syscfg
    *            @arg  ADC_INJEXTERNALTRIGCONV_Ext_acmp2out_syscfg
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_EXT_INJ_TRIG_SYSCFG(ADC_HandleTypeDef* hadc,uint32_t trig)
{
  /* Process locked */
	__HAL_LOCK(hadc);

    MODIFY_REG(SYSCFG->ADCETSR, SYSCFG_ADCETSR_INJ_TRIG_SEL, trig);  
    
  	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;  
}

/**
  * @brief  Enables the selected ADC software start conversion of the injected channels.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc)
{
	__IO uint32_t counter = 0U;
  
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Enable the ADC peripheral */
	/* Check if ADC peripheral is disabled in order to enable it and wait during 
	Tstab time the ADC's stabilization */
	if((hadc->Instance->DGCTRL & ADC_DGCTRL_ADCC_EN) != ADC_DGCTRL_ADCC_EN)
	{
		/* Enable the Peripheral */
		__HAL_ADC_ENABLE(hadc);
		
		/* Delay for ADC stabilization time */
		/* Compute number of CPU cycles to wait for */
		counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
		while(counter != 0U)
		{
			counter--;
		}
	}
	
	/* Start conversion if ADC is effectively enabled */
	if(HAL_IS_BIT_SET(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Set ADC state                                                          */
		/* - Clear state bitfield related to injected group conversion results    */
		/* - Set state bitfield related to injected operation                     */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC,
						HAL_ADC_STATE_INJ_BUSY);
	
		/* Process unlocked */
		/* Unlock before starting ADC conversions: in case of potential           */
		/* interruption, to let the process to ADC IRQ Handler.                   */
		__HAL_UNLOCK(hadc);

		/* Clear regular group conversion flag and overrun flag */
		/* (To ensure of no unknown state from potential previous ADC operations) */
		__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EOIC);
	
		/* Enable the selected ADC software conversion for regular group */
		hadc->Instance->ANCTRL |= (uint32_t)ADC_ANCTRL_INJ_START;
	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
	
		/* Set ADC error code to ADC IP internal error */
		SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
	}
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Stop conversion of injected channels. Disable ADC peripheral if
  *         no regular conversion is on going.
  * @note   If ADC must be disabled and if conversion is on going on 
  *         regular group, function HAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @param  hadc ADC handle
  * @retval None
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef tmp_hal_status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hadc);
		
	/* Stop potential conversion and disable ADC peripheral                     */
	/* Conditioned to:                                                          */
	/* - No conversion on the other group (regular group) is intended to        */
	/*   continue (injected and regular groups stop conversion and ADC disable  */
	/*   are common)                                                            */
	if((hadc->State & HAL_ADC_STATE_REG_BUSY) == RESET)
	{
		/* Stop potential conversion on going, on regular and injected groups */
		/* Disable ADC peripheral */
		__HAL_ADC_DISABLE(hadc);
		
		/* Check if ADC is effectively disabled */
		if(HAL_IS_BIT_CLR(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
		{
			/* Set ADC state */
			ADC_STATE_CLR_SET(hadc->State,
								HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
								HAL_ADC_STATE_READY);
		}
	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
		
		tmp_hal_status = HAL_ERROR;
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return tmp_hal_status;
}

/**
  * @brief  Poll for injected conversion complete
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  Timeout Timeout value in millisecond.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout)
{
	uint32_t tickstart = 0U;
	
	/* Get tick */ 
	tickstart = HAL_GetTick();
	
	/* Check End of conversion flag */
	while(!(__HAL_ADC_GET_IT_FLAG(hadc, ADC_IT_FLAG_EOIC)))
	{
		/* Check if timeout is disabled (set to infinite wait) */
		if(Timeout != HAL_MAX_DELAY)
		{
			if((Timeout == 0U) || ((HAL_GetTick() - tickstart ) > Timeout))
			{
				/* New check to avoid false timeout detection in case of preemption */
				if(!(__HAL_ADC_GET_IT_FLAG(hadc, ADC_IT_FLAG_EOIC)))
				{
					/* Update ADC state machine to timeout */
					SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);
					
					/* Process unlocked */
					__HAL_UNLOCK(hadc);
					
					return HAL_TIMEOUT;
				}
			}
		}
	}
	
	/* Clear injected group conversion flag */
	__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EOIC);
	
	/* Update ADC state machine */
	SET_BIT(hadc->State, HAL_ADC_STATE_INJ_EOC);
	CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
	
	/* Return ADC state */
	return HAL_OK;
}

/**
  * @brief  Enables the interrupt and starts ADC conversion of injected channels.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc)
{
	__IO uint32_t counter = 0U;
  
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Enable the ADC peripheral */
	/* Check if ADC peripheral is disabled in order to enable it and wait during 
	Tstab time the ADC's stabilization */
	if((hadc->Instance->DGCTRL & ADC_DGCTRL_ADCC_EN) != ADC_DGCTRL_ADCC_EN)
	{
		/* Enable the Peripheral */
		__HAL_ADC_ENABLE(hadc);
		
		/* Delay for ADC stabilization time */
		/* Compute number of CPU cycles to wait for */
		counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
		while(counter != 0U)
		{
			counter--;
		}
	}
	
	/* Start conversion if ADC is effectively enabled */
	if(HAL_IS_BIT_SET(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Set ADC state                                                          */
		/* - Clear state bitfield related to injected group conversion results    */
		/* - Set state bitfield related to injected operation                     */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC,
						HAL_ADC_STATE_INJ_BUSY);
	
		/* Process unlocked */
		/* Unlock before starting ADC conversions: in case of potential           */
		/* interruption, to let the process to ADC IRQ Handler.                   */
		__HAL_UNLOCK(hadc);
		
		/* Set the injected conversion mode */
		SET_BIT(hadc->Instance->DGCTRL, ADC_DGCTRL_INJ_MOD);
		
		/* Clear regular group conversion flag and overrun flag */
		/* (To ensure of no unknown state from potential previous ADC operations) */
		__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EOIC);
		
		/* Enable end of conversion interrupt for regular group */
		__HAL_ADC_ENABLE_IT(hadc, ADC_IT_EOIC);
	
		/* Enable the selected ADC software conversion for regular group */
		hadc->Instance->ANCTRL |= (uint32_t)ADC_ANCTRL_INJ_START;

	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
	
		/* Set ADC error code to ADC IP internal error */
		SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
	}
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Stop conversion of injected channels, disable interruption of 
  *         end-of-conversion. Disable ADC peripheral if no regular conversion
  *         is on going.
  * @note   If ADC must be disabled and if conversion is on going on 
  *         regular group, function HAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @param  hadc ADC handle
  * @retval None
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef tmp_hal_status = HAL_OK;

	/* Process locked */
	__HAL_LOCK(hadc);
		
	/* Stop potential conversion and disable ADC peripheral                     */ 
	if((hadc->State & HAL_ADC_STATE_REG_BUSY) == RESET)
	{
		/* Stop potential conversion on going, on regular and injected groups */
		/* Disable ADC peripheral */
		__HAL_ADC_DISABLE(hadc);
		
		/* Check if ADC is effectively disabled */
		if(HAL_IS_BIT_CLR(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
		{
			/* Disable ADC end of conversion interrupt for injected channels */
			__HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOIC);
			
			/* Set ADC state */
			ADC_STATE_CLR_SET(hadc->State,
								HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
								HAL_ADC_STATE_READY);
		}
	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
		
		tmp_hal_status = HAL_ERROR;
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return tmp_hal_status;
}

/**
  * @brief  Enables ADC DMA request after last transfer (Dual-ADC mode) and enables ADC peripheral
  * 
  * @note   Caution: This function must be used only with the ADC master.  
  *
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  pData   Pointer to buffer in which transferred from ADC peripheral to memory will be stored. 
  * @param  Length  The length of data to be transferred from ADC peripheral to memory.  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_DualModeStart_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
{
	__IO uint32_t counter = 0U;
	
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Enable the ADC peripheral */
	/* Check if ADC peripheral is disabled in order to enable it and wait during 
	Tstab time the ADC's stabilization */
	if((hadc->Instance->DGCTRL & ADC_DGCTRL_ADCC_EN) != ADC_DGCTRL_ADCC_EN)
	{
		/* Enable the Peripheral */
		__HAL_ADC_ENABLE(hadc);
		
		/* Delay for ADC stabilization time */
		/* Compute number of CPU cycles to wait for */
		counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
		while(counter != 0U)
		{
			counter--;
		}
	}
	
	/* Start conversion if ADC is effectively enabled */
	if(HAL_IS_BIT_SET(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Set ADC state                                                          */
		/* - Clear state bitfield related to regular group conversion results     */
		/* - Set state bitfield related to regular group operation                */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC,
						HAL_ADC_STATE_REG_BUSY);
		
		/* State machine update: Check if an injected conversion is ongoing */
		if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
		{
			/* Reset ADC error code fields related to conversions on group regular */
			CLEAR_BIT(hadc->ErrorCode,  HAL_ADC_ERROR_DMA);         
		}
		else
		{
			/* Reset ADC all error code fields */
			ADC_CLEAR_ERRORCODE(hadc);
		}
	
		/* Process unlocked */
		/* Unlock before starting ADC conversions: in case of potential           */
		/* interruption, to let the process to ADC IRQ Handler.                   */
		__HAL_UNLOCK(hadc);
	
		/* Set the DMA transfer complete callback */
		hadc->DMA_Handle->XferBlockCallback = ADC_DMAConvCplt;
		
		/* Set the DMA error callback */
		hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;
	
		/* Manage ADC and DMA start:  DMA start, ADC     						  */
		/* start (in case of SW start):                                           */
		/* Clear regular group conversion flag and overrun flag 				  */
		/* (To ensure of no unknown state from potential previous ADC operations) */
		__HAL_ADC_CLEAR_IT_FLAG(hadc, ADC_IT_FLAG_EORC);
		
		/* Enable ADC DMA mode1 */
		MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_DMA_MOD, ADC_DGCTRL_DMA_MOD_1);
		 
		/* Start the DMA channel */
		HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DUALDAT, (uint32_t)pData, Length);
		
		/* Enable the selected ADC software conversion for regular group */
		hadc->Instance->ANCTRL |= (uint32_t)ADC_ANCTRL_RGL_START;
	}
	else
	{
		/* Update ADC state machine to error */
		SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
	
		/* Set ADC error code to ADC IP internal error */
		SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
	}
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Disables ADC DMA (dual-ADC mode) and disables ADC peripheral    
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_DualModeStop_DMA(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef tmp_hal_status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* Stop potential conversion on going, on regular and injected groups */
	/* Disable ADC peripheral */
	__HAL_ADC_DISABLE(hadc);
	
	/* Check if ADC is effectively disabled */
	if(HAL_IS_BIT_CLR(hadc->Instance->DGCTRL, ADC_DGCTRL_ADCC_EN))
	{
		/* Disable the selected ADC DMA mode */
		CLEAR_BIT(hadc->Instance->DGCTRL, ADC_DGCTRL_DMA_MOD); 
		
		/* Disable the DMA channel */
		tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);
		
		/* Set ADC state */
		ADC_STATE_CLR_SET(hadc->State,
						HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
						HAL_ADC_STATE_READY);
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return tmp_hal_status;
}

/**
  * @brief  Configures for the selected ADC injected channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  sConfigInjected ADC configuration structure for injected channel. 
  * @retval None
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc, ADC_InjectionConfTypeDef* sConfigInjected)
{
	/* Process locked */
	__HAL_LOCK(hadc);
	
	/* if ADC_Channel_10 ... ADC_Channel_15 is selected */
	if (sConfigInjected->InjectedChannel > ADC_CHANNEL_9)
	{
		/* Clear the old average times */
		hadc->Instance->CHAVGCFG1 &= ~ADC_CHAVGCFG1(ADC_CHAVGCFG1_CH10_AVG_SEL, sConfigInjected->InjectedChannel);
		
		/* Set the new average times */
		hadc->Instance->CHAVGCFG1 |= ADC_CHAVGCFG1(sConfigInjected->AverageTimes, sConfigInjected->InjectedChannel);
	}
	else /* ADC_Channel include in ADC_Channel_[0..9] */
	{
		/* Clear the old average times */
		hadc->Instance->CHAVGCFG0 &= ~ADC_CHAVGCFG0(ADC_CHAVGCFG0_CH0_AVG_SEL, sConfigInjected->InjectedChannel);
		
		/* Set the new average times */
		hadc->Instance->CHAVGCFG0 |= ADC_CHAVGCFG0(sConfigInjected->AverageTimes, sConfigInjected->InjectedChannel);
	}
	
	/*---------------------------- ADCx INJCHCFG Configuration -----------------*/
	hadc->Instance->INJCHCFG &= ~(ADC_INJCHCFG_INJ_LENG);
	hadc->Instance->INJCHCFG |=  ADC_INJ_LENG(sConfigInjected->InjectedNbrOfConversion);
	
	/* Rank configuration */
	
	/* Clear the old INJ_CHx_SEL  bits for the selected rank */
	hadc->Instance->INJCHCFG &= ~ADC_INJCHCFG_RK(ADC_INJCHCFG_INJ_CH1_SEL, sConfigInjected->InjectedRank);
	
	/* Set the INJ_CHx_SEL bits for the selected rank */
	hadc->Instance->INJCHCFG |= ADC_INJCHCFG_RK(sConfigInjected->InjectedChannel, sConfigInjected->InjectedRank);
	
	/* Enable external trigger if trigger selection is different of software  */
	/* start.                                                                 */
	/* Note: This configuration keeps the hardware feature of parameter       */
	/*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
	/*       software start.                                                  */ 
	if(sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
	{  
		/* Select external trigger to start conversion */
		hadc->Instance->DGCTRL &= ~(ADC_DGCTRL_INJ_TRIG_SEL);
		hadc->Instance->DGCTRL |=  sConfigInjected->ExternalTrigInjecConv;
		
		/* Select external trigger polarity */
		hadc->Instance->DGCTRL &= ~(ADC_DGCTRL_INJ_TRIG_EDG);
		hadc->Instance->DGCTRL |= sConfigInjected->ExternalTrigInjecConvEdge;
	}
	else
	{
		/* Reset the external trigger */
		hadc->Instance->DGCTRL &= ~(ADC_DGCTRL_INJ_TRIG_SEL);
		hadc->Instance->DGCTRL &= ~(ADC_DGCTRL_INJ_TRIG_EDG);  
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Configures for the selected ADC injected channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  sConfigDifferential ADC configuration structure for Differential channel. 
  * @retval None
  */
HAL_StatusTypeDef HAL_ADCEx_DifferentialConfigChannel(ADC_HandleTypeDef* hadc, ADC_DifferentialConfTypeDef* sConfigDifferential)
{
	/* Process locked */
	__HAL_LOCK(hadc);
	
	if(sConfigDifferential->DifferentialConvMode == ENABLE)
	{
		/* Enable the ADC  Differential Channel*/
		SET_BIT(hadc->Instance->ANCTRL, sConfigDifferential->DifferentialChannel);
	}
	else
	{
		/* Disable the ADC Differential Channel*/
		CLEAR_BIT(hadc->Instance->ANCTRL, sConfigDifferential->DifferentialChannel);
	}
	
	/* Process unlocked */
	__HAL_UNLOCK(hadc);
	
	/* Return function status */
	return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup ADC_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters 
  *         in the ADC_InitStruct without initializing the ADC MSP.       
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval None
  */
static void ADC_Init(ADC_HandleTypeDef* hadc)
{
	/* Set the ADC power on */
	SET_BIT(hadc->Instance->ANCTRL, ADC_ANCTRL_POWER_ON);
	
	/* Set the ADC channel switching time 15adcclk */
	SET_BIT(hadc->Instance->CLKCTRL, ADC_CLKCTRL_CH_SWITCH);
	
	/* Set the ADC  pclk div */
	MODIFY_REG(hadc->Instance->CLKCTRL, ADC_CLKCTRL_PCLK_DIV, hadc->Init.ClockPrescaler - 1);
	
	/* Set the ADC regular sequence mode */
	if(hadc->Init.ContinuousConvMode == ENABLE)
	{
		/* ADC regular continuous conversion mode */
		MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_RGL_MOD, ADC_DGCTRL_RGL_MOD_1);
	}
	else if(hadc->Init.DiscontinuousConvMode == ENABLE)
	{
		/* ADC regular discontinuous conversion mode */
		MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_RGL_MOD, ADC_DGCTRL_RGL_MOD);
		
		/* Set the ADC regular discontinuous length */
		hadc->Instance->RGLCHCFG3 &= ~ADC_RGLCHCFG3_SHORT_LENG;
		hadc->Instance->RGLCHCFG3 |= ADC_RGLCHCFG3_SHORTLENG(hadc->Init.NbrOfDiscConversion);
	}
	else
	{
		/* ADC regular single conversion mode */
		MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_RGL_MOD, ADC_DGCTRL_RGL_MOD_0);
	}
	
	/* Set the ADC regular sequence conversion length */
	hadc->Instance->RGLCHCFG3 &= ~ADC_RGLCHCFG3_RGL_LENG;
	hadc->Instance->RGLCHCFG3 |= ADC_RGLCHCFG3_LENG(hadc->Init.NbrOfConversion);
	
	/* Set the ADC dual regular conversion mode */
	MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_DUAL_RGL_MOD, hadc->Init.DualRglConvMode);
	
	/* Enable the ADC channel data clearing after read */
	SET_BIT(hadc->Instance->DGCTRL, ADC_DGCTRL_DATA_R_CLR);
	
	/* Enable external trigger if trigger selection is different of software  */
	/* start.                                                                 */
	/* Note: This configuration keeps the hardware feature of parameter       */
	/*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
	/*       software start.                                                  */
	if(hadc->Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_NONE)
	{
		/* Select external trigger to start conversion */
		MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_RGL_TRIG_SEL, hadc->Init.ExternalTrigConv);
		
		/* Select external trigger polarity */
		MODIFY_REG(hadc->Instance->DGCTRL, ADC_DGCTRL_RGL_TRIG_EDG, hadc->Init.ExternalTrigConvEdge);
	}
	else
	{
		/* Reset the external trigger */
		hadc->Instance->DGCTRL &= ~ADC_DGCTRL_RGL_TRIG_SEL;
		hadc->Instance->DGCTRL &= ~ADC_DGCTRL_RGL_TRIG_EDG;
	}
	
	/* Set whether OPA is connect to ADC */
	if(hadc->Init.Opamp_en == ENABLE)
	{
		/* Enable OPA is connect to ADC */
		SET_BIT(hadc->Instance->ANCTRL, ADC_ANCTRL_OPAMP_EN);
	}
	else
	{
		/* Disable OPA is connect to ADC */
		CLEAR_BIT(hadc->Instance->ANCTRL, ADC_ANCTRL_OPAMP_EN);
	}
}

/**
  * @brief  DMA transfer complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma)   
{
	/* Retrieve ADC handle corresponding to current DMA handle */
	ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
	
	/* Update state machine on conversion status if not in error state */
	if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))
	{
		/* Update ADC state machine */
		SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);
		
		/* Set ADC state */
		CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);   
		
		if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
		{
			SET_BIT(hadc->State, HAL_ADC_STATE_READY);
		}
		
		/* Conversion complete callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
		hadc->ConvCpltCallback(hadc);
#else
		HAL_ADC_ConvCpltCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
	}
	else /* DMA and-or internal error occurred */
	{
		if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) != 0UL)
		{
		/* Call HAL ADC Error Callback function */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
		hadc->ErrorCallback(hadc);
#else
		HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
		}
		else
		{
		/* Call DMA error callback */
		hadc->DMA_Handle->XferErrorCallback(hdma);
		}
	}
}

/**
  * @brief  DMA error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void ADC_DMAError(DMA_HandleTypeDef *hdma)   
{
	ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
	hadc->State= HAL_ADC_STATE_ERROR_DMA;
	/* Set ADC error code to DMA error */
	hadc->ErrorCode |= HAL_ADC_ERROR_DMA;
	/* Error callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
	hadc->ErrorCallback(hadc);
#else
	HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
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

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
