/**
  ******************************************************************************
  * @file     um324xx_hal_vref.c 
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

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"


/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @defgroup VREF VREF
  * @brief HAL VREF module driver
  * @{
  */
#ifdef HAL_VREF_MODULE_ENABLED

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @defgroup VREF_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup VREF_Private_Functions  VREF Private Functions
  * @{
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup VREF_Exported_Functions VREF Exported Functions
  * @{
  */
/**
  * @brief Initialize the VREF according to the specified parameters
  *        in the VREF_InitTypeDef and initialize the associated handle.
  * @param hvref VREF handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_VREF_Init(VREF_HandleTypeDef *hvref)
{
    /* Check Null pointer */
    if((hvref == NULL)||(hvref->Instance == NULL))
    {
        return HAL_ERROR;
    }
    
     /* Unlock the VREF setting register*/
    __HAL_VREF_WRITE_UNLOCK();
	
       
    if((hvref->Init.Mode & TS_VREFCFG_PD) != TS_VREFCFG_PD)
    {
        if((hvref->Init.ChopEn & TS_VREFCFG_CHOP_CLK_EN) == TS_VREFCFG_CHOP_CLK_EN)
        {
            if(hvref->Init.ChopDiv == 0)
            {
                /* Return to the initial default value */
                hvref->Init.ChopDiv = 0xEF;  
            }

            MODIFY_REG(hvref->Instance->TSVREFCFG,TS_VREFCFG_CHOP_CLK_EN_Msk | TS_VREFCFG_CHOP_CLK_DIV_Msk, \
                hvref->Init.ChopEn | (hvref->Init.ChopDiv<<4) );
        }
        
        MODIFY_REG(hvref->Instance->TSVREFCFG, TS_VREFCFG_PD_Msk, \
                   hvref->Init.Mode | hvref->Init.VrefSel );
                
    }
    else
    {
        /* VREF power down*/
        SET_BIT(hvref->Instance->TSVREFCFG,TS_VREFCFG_PD);
    }
    
    /* Unlock the VREF setting register */
    __HAL_VREF_WRITE_LOCK();
    
     return HAL_OK;
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
#endif /* HAL_VREF_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
