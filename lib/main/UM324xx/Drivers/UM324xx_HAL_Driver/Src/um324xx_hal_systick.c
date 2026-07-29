/**
  ******************************************************************************
  * @file     um324xx_hal_systick.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-11
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

/** @defgroup SYSTICK SYSTICK
  * @brief HAL SYSTICK module driver
  * @{
  */
#ifdef HAL_SYSTICK_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup SYSTICK_Exported_Functions SYSTICK Exported Functions
  * @{
  */

/**
  * @brief  Initializes the SYSTICK mode and create the associated handle.
  * @param  hsystick  Pointer to a SYSTICK_TypeDef structure that contains
  *                   the configuration information for the specified SYSTICK module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SYSTICK_Init(SYSTICK_HandleTypeDef *hsystick)
{
    /* Check the SYSTICK handle allocation */
    if (hsystick == NULL)
    {
        return HAL_ERROR;
    }

    /* Configure the SysTick */
    if (HAL_SYSTICK_Config(hsystick->load_value / (1000U)) > 0U)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  Sets Systick load value.
  * @param  hsystick pointer to a SYSTICK_HandleTypeDef structure that contains
  *         the configuration information for the specified SYSTICK.
  * @retval Systick load value
  */
uint32_t HAL_SYSTICK_GetLoad(SYSTICK_HandleTypeDef * hsystick)
{
    return hsystick->Instance->RVR;
}

/**
  * @brief  Sets Systick load value.
  * @param  hsystick pointer to a SYSTICK_HandleTypeDef structure that contains
  *         the configuration information for the specified SYSTICK.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SYSTICK_SetLoad(SYSTICK_HandleTypeDef * hsystick)
{
    if(hsystick == NULL)
    {
        return HAL_ERROR;
    }

    SYSTICK->RVR = hsystick->load_value;

    return HAL_OK;
}

/**
  * @brief  Gets the cnt value.
  * @param  hsystick pointer to a SYSTICK_HandleTypeDef structure that contains
  *         the configuration information for the specified SYSTICK.
  * @retval cnt value
  */
uint32_t HAL_SYSTICK_GetCntValue(SYSTICK_HandleTypeDef * hsystick)
{
    return hsystick->Instance->CVR;
}

/**
  * @}
  */

#endif /* HAL_SYSTICK_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/

