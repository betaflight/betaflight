 /**
  ******************************************************************************
  * @file     um324xF_hal_vref.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-14  
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
#ifndef __UM324XF_HAL_TS_H__
#define __UM324XF_HAL_TS_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @addtogroup TS
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup TS_Exported_typedefs VREF Exported Typedefs
  * @{
  */ 

/**
  * @brief TS Init Structure definition
  */
typedef struct
{ 
  uint32_t      Mode;           /*!< TS enable signal mode.
                                This parameter can be a value of @ref TS_mode_define */

  uint32_t      WorkMode;       /*!< TS work mode.
                                This parameter can be a value of @ref TS_work_mode_define */
	
  uint32_t      ChopEn;         /*!< TS enable chopper clock.
                                This parameter can be a value of @ref TS_CHOP_CLK_EN_define */ 
    
  uint32_t      ChopDiv;        /*!< Chopper clock division value setting.
                                This parameter must be a number between Min_Data = 0x1 and Max_Data = 0x3FF */ 
	
  FunctionalState irq_en;		/*!< TS interrupt enable.
                                This parameter can be a value of ENABLE or DISABLE */
} TS_InitTypeDef;

/**
  * @brief  TS handle Structure definition
  */
typedef struct
{ 
    
  TS_TypeDef      *Instance;         /*!< TS registers base address */    

  TS_InitTypeDef    Init;           /*!< TS enable signal mode.
                                        This parameter can be a value of @ref TS_mode_define */
}TS_HandleTypeDef;

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup TS_Exported_constants TS Exported Constants
  * @{
  */ 

/** @defgroup TS_mode_define TS mode define
   * @{
   */ 
#define TS_NORMAL_MODE                   (0x00000000U)   
#define TS_PORDOWN_MODE                  TS_CFG_PD 
/**
  * @}
  */

/** @defgroup TS_work_mode_define TS work mode define
   * @{
   */ 
#define TS_LOW_SPEED_MODE                (0x00000000U)   
#define TS_HIGH_SPEED_MODE               TS_CFG_MODE 
/**
  * @}
  */

/** @defgroup TS_CHOP_CLK_EN_define TS CHOP CLK EN define
   * @{
   */ 

#define TS_CHOP_CLK_DISEN                 (0x00000000U) 
#define TS_CHOP_CLK_EN                    TS_VREFCFG_CHOP_CLK_EN 

/**
  * @}
  */

#define K   19.32    // The default value of K is 19.32, 
                     // when two points are actually used, 
                     // fill in the value of K according to the actual calculation result.

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TS_Exported_macro TS Exported Macro
  * @{
  */ 
/**
  * @brief  Unlock TS write enable.
  * @retval None
  */                                     
#define __HAL_TS_WRITE_UNLOCK()           WRITE_REG(TS->UNLOCK,0xA5A55A5A)

/**
  * @brief  Lock TS write enable.
  * @retval None
  */                                     
#define __HAL_TS_WRITE_LOCK()             WRITE_REG(TS->UNLOCK,0xFFFFFFFF)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup TS_Exported_Functions
  * @{
  */ 
HAL_StatusTypeDef HAL_TS_Init(TS_HandleTypeDef *hts);
void HAL_TS_IRQHandler(TS_HandleTypeDef* hts);
uint16_t HAL_TS_get_data(TS_HandleTypeDef *hts);
float calculate_temp(TS_HandleTypeDef *hts, uint16_t code);
void HAL_TS_ConvCpltCallback(TS_HandleTypeDef* hts);

/* Private macros ------------------------------------------------------------*/
/** @defgroup TS_Private_Macros 
  * @{
  */
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup TS_Private_Functions VREF Private Functions
  * @{
  */  
  
  
  
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
