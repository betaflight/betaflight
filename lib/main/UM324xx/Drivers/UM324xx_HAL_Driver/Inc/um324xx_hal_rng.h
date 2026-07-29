/**
  ******************************************************************************
  * @file     um324xx_hal_RNG.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_RNG_H__
#define __UM324XX_HAL_RNG_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx.h"
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup SYTICK
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/

/** @defgroup RNG_Exported_Types_Group2 RNG State Structure definition
  * @{
  */
typedef enum
{
  HAL_RNG_STATE_RESET     = 0x00U,  /*!< RNG not yet initialized or disabled */
  HAL_RNG_STATE_READY     = 0x01U,  /*!< RNG initialized and ready for use   */
  HAL_RNG_STATE_BUSY      = 0x02U,  /*!< RNG internal process is ongoing     */
  HAL_RNG_STATE_TIMEOUT   = 0x03U,  /*!< RNG timeout state                   */
  HAL_RNG_STATE_ERROR     = 0x04U   /*!< RNG error state                     */

} HAL_RNG_StateTypeDef;

/**
  * @}
  */

/** @defgroup RNG_Exported_Types_Group3 RNG Handle Structure definition
  * @{
  */
typedef struct
{
  RNG_TypeDef                 *Instance;    /*!< Register base address   */

  HAL_LockTypeDef             Lock;         /*!< RNG locking object      */

  __IO HAL_RNG_StateTypeDef   State;        /*!< RNG communication state */

  __IO  uint32_t              ErrorCode;    /*!< RNG Error code          */

  uint32_t                    RandomNumber; /*!< Last Generated RNG Data */

} RNG_HandleTypeDef;


/* Exported constants --------------------------------------------------------*/
/** @defgroup RNG_Exported_Constants RNG Exported Constants
  * @{
  */

/** @defgroup RNG_Error_Definition   RNG Error Definition
  * @{
  */
#define  HAL_RNG_ERROR_NONE             0x00000000U    /*!< No error          */
#define  HAL_RNG_ERROR_BUSY             0x00000004U    /*!< Busy error        */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup RNG_Exported_Macros RNG Exported Macros
  * @{
  */

/**
  * @brief  Enables the RNG peripheral.
  * @param  __HANDLE__ RNG Handle
  * @retval None
  */
#define __HAL_RNG_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CR |=  RNG_CR_EN)

/**
  * @brief  Disables the RNG peripheral.
  * @param  __HANDLE__ RNG Handle
  * @retval None
  */
#define __HAL_RNG_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CR &= ~RNG_CR_EN)

/* Exported functions --------------------------------------------------------*/
/** @defgroup RNG_Exported_Functions RNG Exported Functions
  * @{
  */

/** @defgroup RNG_Exported_Functions_Group1 Initialization and configuration functions
  * @{
  */
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng);
HAL_StatusTypeDef HAL_RNG_DeInit(RNG_HandleTypeDef *hrng);
HAL_StatusTypeDef HAL_RNG_Seed(RNG_HandleTypeDef *hrng, uint32_t rng_seed_value);
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng);

/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */
uint32_t HAL_RNG_GetRandomNumber(RNG_HandleTypeDef *hrng);    /* Obsolete, use HAL_RNG_GenerateRandomNumber() instead    */
HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
uint32_t HAL_RNG_ReadLastRandomNumber(RNG_HandleTypeDef *hrng);

/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group3 Peripheral State functions
  * @{
  */
HAL_RNG_StateTypeDef HAL_RNG_GetState(RNG_HandleTypeDef *hrng);
uint32_t             HAL_RNG_GetError(RNG_HandleTypeDef *hrng);

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

