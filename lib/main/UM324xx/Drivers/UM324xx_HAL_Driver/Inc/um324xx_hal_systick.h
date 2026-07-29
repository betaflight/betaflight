/**
  ******************************************************************************
  * @file    um324xx_hal_systick.h
  * @author  MCU Team
  * @version V1.00 
  * @date    10-February-2023  
  * @brief   Header file of SYSTICK HAL module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023 Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_SYSTICK_H__
#define __UM324XX_HAL_SYSTICK_H__

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup SYTICK
  * @{
  */

/* Exported typedefs ---------------------------------------------------------*/

/**
  * @brief  SYSTICK handle Structure definition
  */
typedef struct __SYSTICK_HandleTypeDef
{
  SYSTICK_TypeDef               *Instance;      /*!< SYSTICK registers base address */

  uint32_t                      load_value;     /*!< SYSTICK load value             */

  uint32_t                      cnt_value;      /*!< SYSTICK current value          */


} SYSTICK_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup SYSTICK_Private_Functions UART Private Functions
  * @{
  */  
HAL_StatusTypeDef HAL_SYSTICK_Init(SYSTICK_HandleTypeDef *hsystick);
uint32_t HAL_SYSTICK_GetLoad(SYSTICK_HandleTypeDef * hsystick);
HAL_StatusTypeDef HAL_SYSTICK_SetLoad(SYSTICK_HandleTypeDef * hsystick);
uint32_t HAL_SYSTICK_GetCntValue(SYSTICK_HandleTypeDef * hsystick);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/



