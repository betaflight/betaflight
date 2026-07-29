/**
  ******************************************************************************
  * @file     um324xx_hal_RNG.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-03-21
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


/** @defgroup RNG RNG
  * @brief HAL RNG module driver
  * @{
  */
#ifdef HAL_RNG_MODULE_ENABLED

/**
  * @brief  Initializes the RNG peripheral and creates the associated handle.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng)
{
    /* Check the RNG handle allocation */
    if (hrng == NULL)
    {
        return HAL_ERROR;
    }

    if (hrng->State == HAL_RNG_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hrng->Lock = HAL_UNLOCKED;
		
        /* Init the low level hardware */
        HAL_RNG_MspInit(hrng);
    }

    /* Change RNG peripheral state */
    hrng->State = HAL_RNG_STATE_BUSY;

    /* Enable the RNG Peripheral */
    __HAL_RNG_ENABLE(hrng);

    /* Initialize the RNG state */
    hrng->State = HAL_RNG_STATE_READY;

    /* Initialise the error code */
    hrng->ErrorCode = HAL_RNG_ERROR_NONE;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  DeInitializes the RNG peripheral.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_DeInit(RNG_HandleTypeDef *hrng)
{
    /* Check the RNG handle allocation */
    if (hrng == NULL)
    {
        return HAL_ERROR;
    }

    /* Disable the RNG Peripheral */
    CLEAR_BIT(hrng->Instance->CR, RNG_CR_EN);

    /* DeInit the low level hardware */
    HAL_RNG_MspDeInit(hrng);

    /* Update the RNG state */
    hrng->State = HAL_RNG_STATE_RESET;

    /* Initialise the error code */
    hrng->ErrorCode = HAL_RNG_ERROR_NONE;

    /* Release Lock */
    __HAL_UNLOCK(hrng);

    /* Return the function status */
    return HAL_OK;
}

/**
  * @brief  Set random number seed.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @param  rng_seed_value  pointer to a random number seed 
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_Seed(RNG_HandleTypeDef *hrng, uint32_t rng_seed_value)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/* Process Locked */
	__HAL_LOCK(hrng);
	
	/* Check the RNG handle allocation */
    if (hrng == NULL)
    {
        return HAL_ERROR;
    }

    /* Check RNG peripheral state */
	if (hrng->State == HAL_RNG_STATE_READY)
	{
		/* Change RNG peripheral state */
		hrng->State = HAL_RNG_STATE_BUSY;
		
		/* Set random number seed */
		hrng->Instance->SEED = rng_seed_value;
		
		 /* Initialize the RNG state */
		hrng->State = HAL_RNG_STATE_READY;

		/* Initialise the error code */
		hrng->ErrorCode = HAL_RNG_ERROR_NONE;
		
    }
	else
	{
		hrng->ErrorCode = HAL_RNG_ERROR_BUSY;
		status = HAL_ERROR;
	}
	
	/* Process Unlocked */
	__HAL_UNLOCK(hrng);
	
    /* Return function status */
    return status;
}

/**
  * @brief  Initializes the RNG MSP.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @retval None
  */
__weak void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrng);
    /* NOTE : This function should not be modified. When the callback is needed,
              function HAL_RNG_MspInit must be implemented in the user file.
     */
}

/**
  * @brief  DeInitializes the RNG MSP.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @retval None
  */
__weak void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrng);
    /* NOTE : This function should not be modified. When the callback is needed,
              function HAL_RNG_MspDeInit must be implemented in the user file.
     */
}

/**
  * @}
  */

/** @addtogroup RNG_Exported_Functions_Group2
  *  @brief   Peripheral Control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Get the 32 bit Random number
@endverbatim
  * @{
  */

/**
  * @brief  Generates a 32-bit random number.
  * @note   Each time the random number data is read the RNG_FLAG_DRDY flag
  *         is automatically cleared.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @param  random32bit pointer to generated random number variable if successful.
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hrng);

    /* Check RNG peripheral state */
    if (hrng->State == HAL_RNG_STATE_READY)
    {
        /* Change RNG peripheral state */
        hrng->State = HAL_RNG_STATE_BUSY;

        /* Get a 32bit Random number */
        hrng->RandomNumber = hrng->Instance->DATA;
        *random32bit = hrng->RandomNumber;
		
        hrng->State = HAL_RNG_STATE_READY;
    }
    else
    {
        hrng->ErrorCode = HAL_RNG_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hrng);

    return status;
}

/**
  * @brief  Returns generated random number in polling mode (Obsolete)
  *         Use HAL_RNG_GenerateRandomNumber() API instead.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @retval Random value
  */
uint32_t HAL_RNG_GetRandomNumber(RNG_HandleTypeDef *hrng)
{
    if (HAL_RNG_GenerateRandomNumber(hrng, &(hrng->RandomNumber)) == HAL_OK)
    {
        return hrng->RandomNumber;
    }
    else
    {
        return 0U;
    }
}

/**
  * @brief  Read latest generated random number.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @retval random value
  */
uint32_t HAL_RNG_ReadLastRandomNumber(RNG_HandleTypeDef *hrng)
{
    return (hrng->RandomNumber);
}

/** @addtogroup RNG_Exported_Functions_Group3
  *  @brief   Peripheral State functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the RNG state.
  * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
  *                the configuration information for RNG.
  * @retval HAL state
  */
HAL_RNG_StateTypeDef HAL_RNG_GetState(RNG_HandleTypeDef *hrng)
{
    return hrng->State;
}

/**
  * @brief  Return the RNG handle error code.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval RNG Error Code
  */
uint32_t HAL_RNG_GetError(RNG_HandleTypeDef *hrng)
{
    /* Return RNG Error Code */
    return hrng->ErrorCode;
}
/**
  * @}
  */

/**
  * @}
  */


#endif /* HAL_RNG_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/

