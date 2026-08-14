 /**
  ******************************************************************************
  * @file     um324xx_hal_vref.h
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
#ifndef __UM324XX_HAL_VREF_H__
#define __UM324XX_HAL_VREF_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup VREF
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup VREF_Exported_typedefs VREF Exported Typedefs
  * @{
  */ 

/**
  * @brief VREF Init Structure definition
  */
typedef struct
{ 
  uint32_t      Mode;           /*!< VREF enable signal mode.
                                This parameter can be a value of @ref VREF_mode_define */

  uint32_t      VrefSel;        /*!< VREF voltage gear selection.
                                This parameter can be a value of @ref VREF_SEL_define */ 
    
  uint32_t      ChopEn;         /*!< VREF enable chopper clock.
                                This parameter can be a value of @ref VREF_CHOP_CLK_EN_define */ 
    
  uint32_t      ChopDiv;        /*!< Chopper clock division value setting.
                                This parameter must be a number between Min_Data = 0x1 and Max_Data = 0x3FF */ 
    
} VREF_InitTypeDef;

/**
  * @brief  VREF handle Structure definition
  */
typedef struct
{ 
    
  VREF_TypeDef      *Instance;         /*!< VREF registers base address */    

  VREF_InitTypeDef    Init;           /*!< VREF enable signal mode.
                                        This parameter can be a value of @ref VREF_mode_define */
}VREF_HandleTypeDef;

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup VREF_Exported_constants VREF Exported Constants
  * @{
  */ 

/** @defgroup VREF_mode_define VREF mode define
   * @{
   */ 

#define VREF_NORMAL_MODE                   (0x00000000U)   
#define VREF_PORDOWN_MODE                  TS_VREFCFG_PD 

/**
  * @}
  */

/** @defgroup VREF_SEL_define VREF SEL define
   * @{
   */ 

#define VREF_SEL_1V5                        (0x00000000U)               /* Output voltage is equal to 1.5V */
#define VREF_SEL_2V0                        TS_VREFCFG_VREF_SEL_0       /* Output voltage is equal to 2.0V */
#define VREF_SEL_2V5                        TS_VREFCFG_VREF_SEL_1       /* Output voltage is equal to 2.5V */
#define VREF_SEL_3V0                        TS_VREFCFG_VREF_SEL         /* Output voltage is equal to 3.0V */

/**
  * @}
  */


/** @defgroup VREF_CHOP_CLK_EN_define VREF CHOP CLK EN define
   * @{
   */ 

#define VREF_CHOP_CLK_DISEN                 (0x00000000U) 
#define VREF_CHOP_CLK_EN                    TS_VREFCFG_CHOP_CLK_EN 

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup VREF_Exported_macro xxx Exported Macro
  * @{
  */ 
/**
  * @brief  Unlock VREF write enable.
  * @retval None
  */                                     
#define __HAL_VREF_WRITE_UNLOCK()           WRITE_REG(VREF->UNLOCK,0xA5A55A5A)

/**
  * @brief  Lock VREF write enable.
  * @retval None
  */                                     
#define __HAL_VREF_WRITE_LOCK()             WRITE_REG(VREF->UNLOCK,0xFFFFFFFF)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup VREF_Exported_Functions
  * @{
  */ 
HAL_StatusTypeDef HAL_VREF_Init(VREF_HandleTypeDef *hvref);


/* Private macros ------------------------------------------------------------*/
/** @defgroup VREF_Private_Macros 
  * @{
  */
   
/* Private functions ---------------------------------------------------------*/
/** @defgroup VREF_Private_Functions VREF Private Functions
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
