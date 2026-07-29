/**
  ******************************************************************************
  * @file     um324xx_hal_lptim.c 
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-06-26  
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

/** @defgroup LPTIM_functions
  * @{
  */
#ifdef HAL_LPTIM_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup LPTIM_Exported_Functions LPTIM Exported Functions
  * @{
  */

/** @defgroup LPTIM_Exported_Functions_Group1 LPTIM Lptime Base functions
  *  @brief    Lptime Base functions
  *
@verbatim
  ==============================================================================
              ##### Lptime Base functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the LPTIM base.
    (+) De-initialize the LPTIM base.
    (+) Start the Lptime Base.
    (+) Stop the Lptime Base.
    (+) Start the Lptime Base and enable interrupt.
    (+) Stop the Lptime Base and disable interrupt.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the LPTIM Lptime base Unit according to the specified
  *         parameters in the LPTIM_HandleTypeDef and initialize the associated handle.
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Base_Init(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the LPTIM handle allocation */
    if (hlptim == NULL)
    {
        return HAL_ERROR;
    }

    if (hlptim->State == HAL_LPTIM_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hlptim->Lock = HAL_UNLOCKED;

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
        /* Reset interrupt callbacks to legacy weak callbacks */
        LPTIM_ResetCallback(hlptim);

        if (hlptim->Base_MspInitCallback == NULL)
        {
            hlptim->Base_MspInitCallback = HAL_LPTIM_Base_MspInit;
        }
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        hlptim->Base_MspInitCallback(hlptim);
#else
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        HAL_LPTIM_Base_MspInit(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Set the Lptime Base configuration */
    LPTIM_Base_SetConfig(hlptim);

    /* Initialize the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the LPTIM Base peripheral
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Base_DeInit(LPTIM_HandleTypeDef *hlptim)
{
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Disable the LPTIM Peripheral Clock */
	__HAL_LPTIM_DISABLE(hlptim);
	
#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
	if (hlptim->Base_MspDeInitCallback == NULL)
	{
		hlptim->Base_MspDeInitCallback = HAL_LPTIM_Base_MspDeInit;
	}
	/* DeInit the low level hardware */
	hlptim->Base_MspDeInitCallback(hlptim);
#else
	/* DeInit the low level hardware: GPIO, CLOCK, NVIC */
	HAL_LPTIM_Base_MspDeInit(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
	
	/* Change LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_RESET;

	/* Release Lock */
	__HAL_UNLOCK(hlptim);

	return HAL_OK;
}

/**
  * @brief  Starts the LPTIM Base generation.
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Base_Start(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM state */
	if (hlptim->State != HAL_LPTIM_STATE_READY)
	{
		return HAL_ERROR;
	}
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Enable LPTIM */ 
	__HAL_LPTIM_ENABLE(hlptim);
	
	return HAL_OK;
}

/**
  * @brief  Stop the LPTIM Base generation.
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Base_Stop(LPTIM_HandleTypeDef *hlptim)
{
	/* Disable LPTIM */ 
	__HAL_LPTIM_DISABLE(hlptim);
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_READY;
	
	return HAL_OK;
}

/**
  * @brief  Starts the LPTIM Base generation in interrupt mode.
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Base_Start_IT(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM state */
	if (hlptim->State != HAL_LPTIM_STATE_READY)
	{
		return HAL_ERROR;
	}
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Enable the LPTIM Over interrupt */
	__HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OVER);
	
	/* Enable LPTIM */ 
	__HAL_LPTIM_ENABLE(hlptim);
	
	return HAL_OK;
}

/**
  * @brief  Stop the LPTIM Base generation in interrupt mode.
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Base_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
	/* Disable the LPTIM Over interrupt */
	__HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OVER);
	
	/* Disable LPTIM */ 
	__HAL_LPTIM_DISABLE(hlptim);
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_READY;
	
	return HAL_OK;
}

/**
  * @}
  */

/** @defgroup LPTIM_Exported_Functions_Group2 LPTIM PWM functions
  *  @brief    LPTIM PWM functions
  *
@verbatim
  ==============================================================================
                          ##### LPTIM PWM functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the LPTIM PWM.
    (+) De-initialize the LPTIM PWM.
    (+) Start the LPTIM PWM.
    (+) Stop the LPTIM PWM.
    (+) Start the LPTIM PWM and enable interrupt.
    (+) Stop the LPTIM PWM and disable interrupt.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the LPTIM PWM Lptime Base according to the specified
  *         parameters in the LPTIM_HandleTypeDef and initializes the associated handle.
  * @param  hlptim LPTIM PWM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_PWM_Init(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM handle allocation */
    if (hlptim == NULL)
    {
        return HAL_ERROR;
    }

    if (hlptim->State == HAL_LPTIM_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hlptim->Lock = HAL_UNLOCKED;

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
        /* Reset interrupt callbacks to legacy weak callbacks */
        LPTIM_ResetCallback(hlptim);

        if (hlptim->PWM_MspInitCallback == NULL)
        {
            hlptim->PWM_MspInitCallback = HAL_LPTIM_PWM_MspInit;
        }
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        hlptim->PWM_MspInitCallback(hlptim);
#else
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        HAL_LPTIM_PWM_MspInit(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Set the Lptime Base configuration */
    LPTIM_Base_SetConfig(hlptim);

    /* Initialize the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the LPTIM PWM peripheral
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_PWM_DeInit(LPTIM_HandleTypeDef *hlptim)
{
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Disable the LPTIM Peripheral Clock */
	__HAL_LPTIM_DISABLE(hlptim);
	
#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
	if (hlptim->PWM_MspDeInitCallback == NULL)
	{
		hlptim->PWM_MspDeInitCallback = HAL_LPTIM_PWM_MspDeInit;
	}
	/* DeInit the low level hardware */
	hlptim->PWM_MspDeInitCallback(hlptim);
#else
	/* DeInit the low level hardware: GPIO, CLOCK, NVIC */
	HAL_LPTIM_PWM_MspDeInit(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
	
	/* Change LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_RESET;

	/* Release Lock */
	__HAL_UNLOCK(hlptim);

	return HAL_OK;
}

/**
  * @brief  Starts the PWM signal generation.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_PWM_Start(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM state */
	if (hlptim->State != HAL_LPTIM_STATE_READY)
	{
		return HAL_ERROR;
	}
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Enable LPTIM */ 
	__HAL_LPTIM_ENABLE(hlptim);
	
	return HAL_OK;
}

/**
  * @brief  Stop the LPTIM PWM signal generation.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop(LPTIM_HandleTypeDef *hlptim)
{
	/* Disable LPTIM */ 
	__HAL_LPTIM_DISABLE(hlptim);
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_READY;
	
	return HAL_OK;
}

/**
  * @brief  Starts the LPTIM PWM signal generation in interrupt mode.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_PWM_Start_IT(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM state */
	if (hlptim->State != HAL_LPTIM_STATE_READY)
	{
		return HAL_ERROR;
	}
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Enable the LPTIM Compare interrupt */
	__HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_COMP);
	
	/* Enable LPTIM */ 
	__HAL_LPTIM_ENABLE(hlptim);
	
	return HAL_OK;
}

/**
  * @brief  Stop the LPTIM PWM signal generation in interrupt mode.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
	/* Disable the LPTIM Compare interrupt */
	__HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_COMP);
	
	/* Disable LPTIM */ 
	__HAL_LPTIM_DISABLE(hlptim);
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_READY;
	
	return HAL_OK;
}

/** @defgroup LPTIM_Exported_Functions_Group3 LPTIM Trigger functions
  *  @brief    LPTIM Trigger functions
  *
@verbatim
  ==============================================================================
                          ##### LPTIM Trigger functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the LPTIM Trigger.
    (+) De-initialize the LPTIM Trigger.
    (+) Start the LPTIM Trigger.
    (+) Stop the LPTIM Trigger.
    (+) Start the LPTIM Trigger and enable interrupt.
    (+) Stop the LPTIM Trigger and disable interrupt.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the LPTIM Trigger according to the specified
  *         parameters in the LPTIM_HandleTypeDef and initializes the associated handle.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Trigger_Init(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM handle allocation */
    if (hlptim == NULL)
    {
        return HAL_ERROR;
    }

    if (hlptim->State == HAL_LPTIM_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hlptim->Lock = HAL_UNLOCKED;

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
        /* Reset interrupt callbacks to legacy weak callbacks */
        LPTIM_ResetCallback(hlptim);

        if (hlptim->Trigger_MspInitCallback == NULL)
        {
            hlptim->Trigger_MspInitCallback = HAL_LPTIM_Trigger_MspInit;
        }
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        hlptim->Trigger_MspInitCallback(hlptim);
#else
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        HAL_LPTIM_Trigger_MspInit(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
    }

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Set the Lptime Base configuration */
    LPTIM_Base_SetConfig(hlptim);

    /* Initialize the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the LPTIM Trigger peripheral
  * @param  hlptim LPTIM Base handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Trigger_DeInit(LPTIM_HandleTypeDef *hlptim)
{
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Disable the LPTIM Peripheral Clock */
	__HAL_LPTIM_DISABLE(hlptim);
	
#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
	if (hlptim->Trigger_MspDeInitCallback == NULL)
	{
		hlptim->Trigger_MspDeInitCallback = HAL_LPTIM_Trigger_MspDeInit;
	}
	/* DeInit the low level hardware */
	hlptim->Trigger_MspDeInitCallback(hlptim);
#else
	/* DeInit the low level hardware: GPIO, CLOCK, NVIC */
	HAL_LPTIM_Trigger_MspDeInit(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
	
	/* Change LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_RESET;

	/* Release Lock */
	__HAL_UNLOCK(hlptim);

	return HAL_OK;
}

/**
  * @brief  Starts the Trigger.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Trigger_Start(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM state */
	if (hlptim->State != HAL_LPTIM_STATE_READY)
	{
		return HAL_ERROR;
	}
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Enable LPTIM */ 
	__HAL_LPTIM_ENABLE(hlptim);
	
	return HAL_OK;
}

/**
  * @brief  Stop the LPTIM Trigger.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Trigger_Stop(LPTIM_HandleTypeDef *hlptim)
{
	/* Disable LPTIM */ 
	__HAL_LPTIM_DISABLE(hlptim);
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_READY;
	
	return HAL_OK;
}

/**
  * @brief  Starts the LPTIM Trigger in interrupt mode.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Trigger_Start_IT(LPTIM_HandleTypeDef *hlptim)
{
	/* Check the LPTIM state */
	if (hlptim->State != HAL_LPTIM_STATE_READY)
	{
		return HAL_ERROR;
	}

	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_BUSY;
	
	/* Enable the LPTIM Trigger interrupt */
	__HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_TRIG);
	
	/* Enable LPTIM */ 
	__HAL_LPTIM_ENABLE(hlptim);
	
	return HAL_OK;
}

/**
  * @brief  Stop the LPTIM Trigger in interrupt mode.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Trigger_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
	/* Disable the LPTIM Trigger interrupt */
	__HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_TRIG);
	
	/* Disable LPTIM */ 
	__HAL_LPTIM_DISABLE(hlptim);
	
	/* Set the LPTIM state */
	hlptim->State = HAL_LPTIM_STATE_READY;
	
	return HAL_OK;
}

/**
  * @}
  */

/** @defgroup LPTIM_Exported_Functions_Group5 LPTIM IRQ handler management
  *  @brief    LPTIM IRQ handler management
  *
@verbatim
  ==============================================================================
                        ##### IRQ handler management #####
  ==============================================================================
  [..]
    This section provides Lptimer IRQ handler function.

@endverbatim
  * @{
  */
/**
  * @brief  This function handles LPTIM interrupts requests.
  * @param  hlptim LPTIM  handle
  * @retval None
  */
void HAL_LPTIM_IRQHandler(LPTIM_HandleTypeDef *hlptim)
{
	/* LPTIM over event */
	if(__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_OVER) != RESET)
	{
		if(__HAL_LPTIM_GET_IT_SOURCE(hlptim, LPTIM_IT_OVER) != RESET)
		{
			__HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_FLAG_OVER);
		
#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
			hlptim->OverCallback(hlptim);
#else
			HAL_LPTIM_OverCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
		}
    }

	/* LPTIM Compare event */
	if(__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_COMP) != RESET)
	{
		if(__HAL_LPTIM_GET_IT_SOURCE(hlptim, LPTIM_IT_COMP) != RESET)
		{
			__HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_FLAG_COMP);
		
#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
			hlptim->CompCallback(hlptim);
#else
			HAL_LPTIM_CompCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
		}
    }

	/* LPTIM Trigger event */
	if(__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_TRIG) != RESET)
	{
		if(__HAL_LPTIM_GET_IT_SOURCE(hlptim, LPTIM_IT_TRIG) != RESET)
		{
			__HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_FLAG_TRIG);
		
#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
			hlptim->TriggerCallback(hlptim);
#else
			HAL_LPTIM_TriggerCallback(hlptim);
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */
		}
    }	
	
}
/**
  * @}
  */

/** @defgroup LPTIM_Exported_Functions_Group6 LPTIM Peripheral Control functions
  *  @brief    LPTIM Peripheral Control functions
  *
@verbatim
  ==============================================================================
                   ##### Peripheral Control functions #####
  ==============================================================================
 [..]
   This section provides functions allowing to:
      (+) Configure The PWM mode.
      (+) Configure External Clock source.
      (+) Configure Trigger mode.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the LPTIM PWM Channel according to the specified
  *         parameters in the LPTIM_PWM_InitTypeDef.
  * @param  hlptim LPTIM handle
  * @param  sConfig LPTIM PWM configuration structure
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_PWM_ConfigChannel(LPTIM_HandleTypeDef *hlptim, LPTIM_PWM_InitTypeDef *sConfig)
{
	/* Process Locked */
    __HAL_LOCK(hlptim);
	
	/* Set the PWM polarity according to the sConfig->PWMPolarity value */
	MODIFY_REG(hlptim->Instance->CFG, LPTIM_CFG_POLARITY, sConfig->PWMPolarity);
	
	/* Set the PWM Mode according to the sConfig->PWMMode value */
	MODIFY_REG(hlptim->Instance->CFG, LPTIM_CFG_PWM, sConfig->PWMMode);
	
	/* Set the compare value according to the sConfig->Pulse value */
	hlptim->Instance->CMP = sConfig->Pulse;
	
	__HAL_UNLOCK(hlptim);

    return HAL_OK;
}
		
/**
  * @brief  Initializes the LPTIM Trigger according to the specified
  *         parameters in the LPTIM_Trigger_InitTypeDef.
  * @param  hlptim LPTIM handle
  * @param  sConfig LPTIM Trigger configuration structure
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_Trigger_ConfigChannel(LPTIM_HandleTypeDef *hlptim, LPTIM_Trigger_InitTypeDef *sConfig)
{
	/* Process Locked */
    __HAL_LOCK(hlptim);
	
	/* Enable the external filter */
	SET_BIT(hlptim->Instance->CFG, LPTIM_CFG_FLTEN);
	
	/* Set the Trigger mode */
	SET_BIT(hlptim->Instance->CFG, LPTIM_CFG_TMODE_0);
	
	if(sConfig->TimeoutMode == ENABLE)
	{
		/* Set the Timeout mode */
		SET_BIT(hlptim->Instance->CFG, LPTIM_CFG_TMODE);
	}
	
	/* Set the Trigger Edge according to the sConfig->TriggerEdge value */
	MODIFY_REG(hlptim->Instance->CFG, LPTIM_CFG_TRIGCFG, sConfig->TriggerEdge);
	
	__HAL_UNLOCK(hlptim);

    return HAL_OK;
}

/**
  * @brief  Initializes the LPTIM External Clock according to the specified
  *         parameters in the LPTIM_ExtClock_InitTypeDef.
  * @param  hlptim LPTIM handle
  * @param  sConfig LPTIM External Clock configuration structure
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPTIM_ext_ClockConfig(LPTIM_HandleTypeDef *hlptim, LPTIM_ExtClock_InitTypeDef *sConfig)
{
	/* Process Locked */
    __HAL_LOCK(hlptim);
	
	/* Enable the external filter */
	SET_BIT(hlptim->Instance->CFG, LPTIM_CFG_FLTEN);
	
	/* Set the external clock count mode */
	SET_BIT(hlptim->Instance->CFG, LPTIM_CFG_TMODE_1);
	
	/* Set the LPTIN Clock */
	SET_BIT(hlptim->Instance->CFG, LPTIM_CFG_CLKSEL);
	
	/* Set the Count Edge according to the sConfig->CountEdge value */
	MODIFY_REG(hlptim->Instance->CFG, LPTIM_CFG_TRIGCFG, sConfig->CountEdge);
	
	__HAL_UNLOCK(hlptim);

    return HAL_OK;
}

/**
  * @}
  */

/**
  * @brief  Initializes the LPTIM mode according to the specified parameters in
  *         the LPTIM_InitTypeDef and create the associated handle.
  * @param  hlptim  Pointer to a LPTIM_HandleTypeDef structure that contains
  *                the configuration information for the specified LPTIM module.
  * @retval HAL status
  */
void LPTIM_Base_SetConfig(LPTIM_HandleTypeDef *hlptim)
{
	/*-------------------------- LPTIM CFG Configuration ------------------------*/
	if(hlptim->Init.ContinuousMode == ENABLE)
	{
		CLEAR_BIT(hlptim->Instance->CFG, LPTIM_CFG_MODE_Msk);
	}
	else
	{
		SET_BIT(hlptim->Instance->CFG, LPTIM_CFG_MODE_Msk);
	}
	
	MODIFY_REG(hlptim->Instance->CFG, LPTIM_CFG_CLKSEL_Msk | LPTIM_CFG_TMODE_Msk | \
									LPTIM_CFG_DIVSEL_Msk, hlptim->Init.ClockDivision | hlptim->Init.ClockSource);
	
	/* Set the target value */ 
	hlptim->Instance->TARGET = hlptim->Init.Period;
}

/**
  * @brief  Initializes the LPTIM Base MSP.
  * @param  hlptim LPTIM Base handle
  * @retval None
  */
__weak void HAL_LPTIM_Base_MspInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Base_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes LPTIM Base MSP.
  * @param  hlptim LPTIM Base handle
  * @retval None
  */
__weak void HAL_LPTIM_Base_MspDeInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Base_MspDeInit could be implemented in the user file
     */
}

/**
  * @brief  Initializes the LPTIM PWM MSP.
  * @param  hlptim LPTIM Base handle
  * @retval None
  */
__weak void HAL_LPTIM_PWM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Base_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes PWM Base MSP.
  * @param  hlptim LPTIM Base handle
  * @retval None
  */
__weak void HAL_LPTIM_PWM_MspDeInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Base_MspDeInit could be implemented in the user file
     */
}

/**
  * @brief  Initializes the LPTIM Trigger MSP.
  * @param  hlptim LPTIM Trigger handle
  * @retval None
  */
__weak void HAL_LPTIM_Trigger_MspInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Trigger_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes Trigger MSP.
  * @param  hlptim LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_Trigger_MspDeInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Trigger_MspDeInit could be implemented in the user file
     */
}

/**
  * @brief  Over callback in non-blocking mode
  * @param  hlptim LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_OverCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Over_Callback could be implemented in the user file
     */
}

/**
  * @brief  Compare callback in non-blocking mode
  * @param  hlptim LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_CompCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_CompCallback could be implemented in the user file
     */
}

/**
  * @brief  Trigger callback in non-blocking mode
  * @param  hlptim LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_TriggerCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_Triggercallback could be implemented in the user file
    */
}

/**
  * @brief  LPTIM Error callback in non-blocking mode
  * @param  hlptim LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_ErrorCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_LPTIM_ErrorCallback could be implemented in the user file
    */
}

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User LPTIM callback to be used instead of the weak predefined callback
  * @param hlptim lptim handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_LPTIM_BASE_MSPINIT_CB_ID Base MspInit Callback ID
  *          @arg @ref HAL_LPTIM_BASE_MSPDEINIT_CB_ID Base MspDeInit Callback ID
  *          @arg @ref HAL_LPTIM_TRIGGER_MSPINIT_CB_ID Trigger MspInit Callback ID
  *          @arg @ref HAL_LPTIM_TRIGGER_MSPDEINIT_CB_ID Trigger MspDeInit Callback ID
  *          @arg @ref HAL_LPTIM_PWM_MSPINIT_CB_ID PWM MspInit Callback ID
  *          @arg @ref HAL_LPTIM_PWM_MSPDEINIT_CB_ID PWM MspDeInit Callback ID
  *          @arg @ref HAL_LPTIM_TRIGGER_CB_ID Trigger Callback ID
  *          @arg @ref HAL_LPTIM_OVER_CB_ID Over Callback ID
  *          @arg @ref HAL_LPTIM_COMP_CB_ID Compare Callback ID
  *          @arg @ref HAL_LPTIM_ERROR_CB_ID Error Callback ID
  *          @param pCallback pointer to the callback function
  *          @retval status
  */
HAL_StatusTypeDef HAL_LPTIM_RegisterCallback(LPTIM_HandleTypeDef *hlptim, HAL_LPTIM_CallbackIDTypeDef CallbackID,
        pLPTIM_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hlptim);

    if (hlptim->State == HAL_LPTIM_STATE_READY)
    {
        switch (CallbackID)
        {
			case HAL_LPTIM_BASE_MSPINIT_CB_ID :
				hlptim->Base_MspInitCallback                 	= pCallback;
				break;

			case HAL_LPTIM_BASE_MSPDEINIT_CB_ID :
				hlptim->Base_MspDeInitCallback               	= pCallback;
				break;

			case HAL_LPTIM_TRIGGER_MSPINIT_CB_ID :
				hlptim->Trigger_MspInitCallback              	= pCallback;
				break;

			case HAL_LPTIM_TRIGGER_MSPDEINIT_CB_ID :
				hlptim->Trigger_MspDeInitCallback            	= pCallback;
				break;

			case HAL_LPTIM_PWM_MSPINIT_CB_ID :
				hlptim->PWM_MspInitCallback                  	= pCallback;
				break;

			case HAL_LPTIM_PWM_MSPDEINIT_CB_ID :
				hlptim->PWM_MspDeInitCallback                	= pCallback;
				break;

			case HAL_LPTIM_TRIGGER_CB_ID :
				hlptim->TriggerCallback                      	= pCallback;
				break;

			case HAL_LPTIM_OVER_CB_ID :
				hlptim->OverCallback         	             	= pCallback;
				break;

			case HAL_LPTIM_COMP_CB_ID :
				hlptim->CompCallback         			     	= pCallback;
				break;

			case HAL_LPTIM_ERROR_CB_ID :
				hlptim->ErrorCallback         					= pCallback;
				break;

			default :
				/* Return error status */
				status = HAL_ERROR;
				break;
        }
    }
    else if (hlptim->State == HAL_LPTIM_STATE_RESET)
    {
        switch (CallbackID)
        {
			case HAL_LPTIM_BASE_MSPINIT_CB_ID :
				hlptim->Base_MspInitCallback                 	= pCallback;
				break;

			case HAL_LPTIM_BASE_MSPDEINIT_CB_ID :
				hlptim->Base_MspDeInitCallback               	= pCallback;
				break;

			case HAL_LPTIM_TRIGGER_MSPINIT_CB_ID :
				hlptim->Trigger_MspInitCallback              	= pCallback;
				break;

			case HAL_LPTIM_TRIGGER_MSPDEINIT_CB_ID :
				hlptim->Trigger_MspDeInitCallback            	= pCallback;
				break;

			case HAL_LPTIM_PWM_MSPINIT_CB_ID :
				hlptim->PWM_MspInitCallback                  	= pCallback;
				break;

			case HAL_LPTIM_PWM_MSPDEINIT_CB_ID :
				hlptim->PWM_MspDeInitCallback                	= pCallback;
				break;

			case HAL_LPTIM_TRIGGER_CB_ID :
				hlptim->TriggerCallback                      	= pCallback;
				break;

			case HAL_LPTIM_OVER_CB_ID :
				hlptim->OverCallback         	             	= pCallback;
				break;

			case HAL_LPTIM_COMP_CB_ID :
				hlptim->CompCallback         			     	= pCallback;
				break;

			case HAL_LPTIM_ERROR_CB_ID :
				hlptim->ErrorCallback         					= pCallback;
				break;

			default :
				/* Return error status */
				status = HAL_ERROR;
				break;
        }
    }
    else
    {
        /* Return error status */
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hlptim);

    return status;
}

/**
  * @brief  Unregister a LPTIM callback
  *         LPTIM callback is redirected to the weak predefined callback
  * @param  hlptim lptim handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_LPTIM_BASE_MSPINIT_CB_ID Base MspInit Callback ID
  *          @arg @ref HAL_LPTIM_BASE_MSPDEINIT_CB_ID Base MspDeInit Callback ID
  *          @arg @ref HAL_LPTIM_TRIGGER_MSPINIT_CB_ID Trigger MspInit Callback ID
  *          @arg @ref HAL_LPTIM_TRIGGER_MSPDEINIT_CB_ID Trigger MspDeInit Callback ID
  *          @arg @ref HAL_LPTIM_PWM_MSPINIT_CB_ID PWM MspInit Callback ID
  *          @arg @ref HAL_LPTIM_PWM_MSPDEINIT_CB_ID PWM MspDeInit Callback ID
  *          @arg @ref HAL_LPTIM_TRIGGER_CB_ID Trigger Callback ID
  *          @arg @ref HAL_LPTIM_OVER_CB_ID Over Callback ID
  *          @arg @ref HAL_LPTIM_COMP_CB_ID Compare Callback ID
  *          @arg @ref HAL_LPTIM_ERROR_CB_ID Error Callback ID
  *          @retval status
  */
HAL_StatusTypeDef HAL_LPTIM_UnRegisterCallback(LPTIM_HandleTypeDef *hlptim, HAL_LPTIM_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hlptim);

    if (hlptim->State == HAL_LPTIM_STATE_READY)
    {
        switch (CallbackID)
        {
			case HAL_LPTIM_BASE_MSPINIT_CB_ID :
				/* Legacy weak Base MspInit Callback */
				hlptim->Base_MspInitCallback                 	= HAL_LPTIM_Base_MspInit;
				break;

			case HAL_LPTIM_BASE_MSPDEINIT_CB_ID :
				/* Legacy weak Base MspDeInit Callback */
				hlptim->Base_MspDeInitCallback               	= HAL_LPTIM_Base_MspDeInit;
				break;

			case HAL_LPTIM_TRIGGER_MSPINIT_CB_ID :
				/* Legacy weak Trigger MspInit Callback */
				hlptim->Trigger_MspInitCallback              	= HAL_LPTIM_Trigger_MspInit;
				break;

			case HAL_LPTIM_TRIGGER_MSPDEINIT_CB_ID :
				/* Legacy weak Trigger MspDeInit Callback */
				hlptim->Trigger_MspDeInitCallback            	= HAL_LPTIM_Trigger_MspDeInit;
				break;

			case HAL_LPTIM_PWM_MSPINIT_CB_ID :
				/* Legacy weak PWM MspInit Callback */
				hlptim->PWM_MspInitCallback                  	= HAL_LPTIM_PWM_MspInit;
				break;

			case HAL_LPTIM_PWM_MSPDEINIT_CB_ID :
				/* Legacy weak PWM MspDeInit Callback */
				hlptim->PWM_MspDeInitCallback                	= HAL_LPTIM_PWM_MspDeInit;
				break;

			case HAL_LPTIM_TRIGGER_CB_ID :
				/* Legacy weak Trigger Callback */
				hlptim->TriggerCallback                      	= HAL_LPTIM_TriggerCallback;
				break;

			case HAL_LPTIM_OVER_CB_ID :
				/* Legacy weak Over Callback */
				hlptim->OverCallback         	             	= HAL_LPTIM_OverCallback;
				break;

			case HAL_LPTIM_COMP_CB_ID :
				/* Legacy weak Compare Callback */
				hlptim->CompCallback         			     	= HAL_LPTIM_CompCallback;
				break;

			case HAL_LPTIM_ERROR_CB_ID :
				/* Legacy weak Error Callback */
				hlptim->ErrorCallback         					= HAL_LPTIM_ErrorCallback;
				break;
			
			default :
				/* Return error status */
				status = HAL_ERROR;
				break;
			
        }
    }
    else if (hlptim->State == HAL_LPTIM_STATE_RESET)
    {
        switch (CallbackID)
        {
			case HAL_LPTIM_BASE_MSPINIT_CB_ID :
				/* Legacy weak Base MspInit Callback */
				hlptim->Base_MspInitCallback                 	= HAL_LPTIM_Base_MspInit;
				break;

			case HAL_LPTIM_BASE_MSPDEINIT_CB_ID :
				/* Legacy weak Base MspDeInit Callback */
				hlptim->Base_MspDeInitCallback               	= HAL_LPTIM_Base_MspDeInit;
				break;

			case HAL_LPTIM_TRIGGER_MSPINIT_CB_ID :
				/* Legacy weak Trigger MspInit Callback */
				hlptim->Trigger_MspInitCallback              	= HAL_LPTIM_Trigger_MspInit;
				break;

			case HAL_LPTIM_TRIGGER_MSPDEINIT_CB_ID :
				/* Legacy weak Trigger MspDeInit Callback */
				hlptim->Trigger_MspDeInitCallback            	= HAL_LPTIM_Trigger_MspDeInit;
				break;

			case HAL_LPTIM_PWM_MSPINIT_CB_ID :
				/* Legacy weak PWM MspInit Callback */
				hlptim->PWM_MspInitCallback                  	= HAL_LPTIM_PWM_MspInit;
				break;

			case HAL_LPTIM_PWM_MSPDEINIT_CB_ID :
				/* Legacy weak PWM MspDeInit Callback */
				hlptim->PWM_MspDeInitCallback                	= HAL_LPTIM_PWM_MspDeInit;
				break;

			case HAL_LPTIM_TRIGGER_CB_ID :
				/* Legacy weak Trigger Callback */
				hlptim->TriggerCallback                      	= HAL_LPTIM_TriggerCallback;
				break;

			case HAL_LPTIM_OVER_CB_ID :
				/* Legacy weak Over Callback */
				hlptim->OverCallback         	             	= HAL_LPTIM_OverCallback;
				break;

			case HAL_LPTIM_COMP_CB_ID :
				/* Legacy weak Compare Callback */
				hlptim->CompCallback         			     	= HAL_LPTIM_CompCallback;
				break;

			case HAL_LPTIM_ERROR_CB_ID :
				/* Legacy weak Error Callback */
				hlptim->ErrorCallback         					= HAL_LPTIM_ErrorCallback;
				break;
			
			default :
				/* Return error status */
				status = HAL_ERROR;
				break;
        }
    }
    else
    {
        /* Return error status */
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hlptim);

    return status;
}
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

#if (USE_HAL_LPTIM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Reset interrupt callbacks to the legacy weak callbacks.
  * @param  hlptim pointer to a LPTIM_HandleTypeDef structure that contains
  *                the configuration information for LPTIM module.
  * @retval None
  */
void LPTIM_ResetCallback(LPTIM_HandleTypeDef *hlptim)
{
	/* Reset the LPTIM callback to the legacy weak callbacks */
	hlptim->OverCallback                      = HAL_LPTIM_OverCallback;
	hlptim->CompCallback     				  = HAL_LPTIM_CompCallback;
	hlptim->TriggerCallback                   = HAL_LPTIM_TriggerCallback;
	hlptim->ErrorCallback                     = HAL_LPTIM_ErrorCallback;
}
#endif /* USE_HAL_LPTIM_REGISTER_CALLBACKS */

/**
  * @}
  */
#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
