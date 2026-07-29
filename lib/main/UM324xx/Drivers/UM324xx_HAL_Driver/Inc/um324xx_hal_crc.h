 /**
  ******************************************************************************
  * @file     um324xx_hal_crc.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-10  
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
#ifndef __UM324XX_HAL_CRC_H__
#define __UM324XX_HAL_CRC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup xxx
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup CRC_Exported_Types CRC Exported Types
  * @{
  */

/**
  * @brief  CRC HAL State Structure definition
  */
typedef enum
{
  HAL_CRC_STATE_RESET     = 0x00U,  /*!< CRC not yet initialized or disabled */
  HAL_CRC_STATE_READY     = 0x01U,  /*!< CRC initialized and ready for use   */
  HAL_CRC_STATE_BUSY      = 0x02U,  /*!< CRC internal process is ongoing     */
  HAL_CRC_STATE_TIMEOUT   = 0x03U,  /*!< CRC timeout state                   */
  HAL_CRC_STATE_ERROR     = 0x04U   /*!< CRC error state                     */
} HAL_CRC_StateTypeDef;


/**
  * @brief  CRC Handle Structure definition
  */
typedef struct
{
  CRC_TypeDef                 *Instance;   /*!< Register base address        */

  uint32_t					  polynomial;  /*!< CRC polynomial               */
																			 
  uint32_t					  width_din;   /*!< CRC data input width         */
																			 
  uint32_t					  din_rev;     /*!< CRC data input reverse       */
																			 
  uint32_t					  dout_rev;    /*!< CRC data output reverse      */
																			 
  uint32_t					  init_rev;    /*!< CRC initial reverse          */		
	
  HAL_LockTypeDef             Lock;        /*!< CRC Locking object           */

  __IO HAL_CRC_StateTypeDef   State;       /*!< CRC communication state      */

} CRC_HandleTypeDef;
/**
  * @}
  */  

/* Exported constants --------------------------------------------------------*/
/** @defgroup xxx_Exported_constants xxx Exported Constants
  * @{
  */ 

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup xxx_Exported_macro xxx Exported Macro
  * @{
  */ 

/** @defgroup CRC_POLYNOMIAL crc polynomial
  * @{
  */
#define POL_CRC16_1021     		((uint32_t)0x00000000)
#define POL_CRC16_8005     		((uint32_t)CRC_CFG_POL_0)
#define POL_CRC32_04C11DB7 		((uint32_t)CRC_CFG_POL_1)
/**
  * @}
  */

/** @defgroup CRC_WIDTH_DIN crc input data width
  * @{
  */
#define WIDTH_DIN_8BIT     		((uint32_t)0x00000000)
#define WIDTH_DIN_16BIT     	((uint32_t)CRC_CFG_WIDTH_DIN_0)
#define WIDTH_DIN_32BIT 		((uint32_t)CRC_CFG_WIDTH_DIN_1)
/**
  * @}
  */

/** @defgroup CRC_DIN_REV crc input data reverse
  * @{
  */
#define DATA_INPUT_REVERSE_NONE ((uint32_t)0x00000000)
#define DATA_INPUT_REVERSE     	((uint32_t)CRC_CFG_DIN_REV)
/**
  * @}
  */

/** @defgroup CRC_DOUT_REV crc output data reverse
  * @{
  */
#define DATA_OUTPUT_REVERSE_NONE ((uint32_t)0x00000000)
#define DATA_OUTPUT_REVERSE      ((uint32_t)CRC_CFG_DOUT_REV)
/**
  * @}
  */

/** @defgroup CRC_INIT_REV crc initial value reverse
  * @{
  */
#define INITIAL_VALUE_REVERSE_NONE       	((uint32_t)0x00000000)
#define INITIAL_VALUE_OUTPUT_REVERSE      	((uint32_t)CRC_CFG_INIT_REV)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup xxx_Exported_Functions
  * @{
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup xxx_Private_Macros 
  * @{
  */
  
/** @brief Reset CRC handle state.
  * @param  __HANDLE__ CRC handle.
  * @retval None
  */
#define __HAL_CRC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_CRC_STATE_RESET)
  
/**
  * @brief  Reset CRC Data Register.
  * @param  __HANDLE__ CRC handle
  * @retval None
  */
#define __HAL_CRC_DR_RESET(__HANDLE__) ((__HANDLE__)->Instance->CFG |= CRC_CFG_RESET)
  
/**
  * @}
  */  

  
/* Private functions ---------------------------------------------------------*/
/** @defgroup CRC_Private_Functions CRC Private Functions
  * @{
  */  
/* Initialization and de-initialization functions  ****************************/
/** @defgroup CRC_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);
/**
  * @}
  */  

/* Peripheral Control functions ***********************************************/
/** @defgroup CRC_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */
uint32_t HAL_CRC16_Calculate(CRC_HandleTypeDef *hcrc, uint16_t pBuffer[], uint32_t BufferLength, uint16_t InitValue, uint16_t XorOut);
uint32_t HAL_CRC32_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength, uint32_t InitValue, uint32_t XorOut);
/**
  * @}
  */

/* Peripheral State and Error functions ***************************************/
/** @defgroup CRC_Exported_Functions_Group3 Peripheral State functions
  * @{
  */
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);
/**
  * @}
  */
  
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
