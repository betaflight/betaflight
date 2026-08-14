/**
  ******************************************************************************
  * @file     um324xF_hal_vref.c 
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


/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @defgroup TS TS
  * @brief HAL TS module driver
  * @{
  */
#ifdef HAL_TS_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup TS_Private_Functions  TS Private Functions
  * @{
  */
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/** @defgroup TS_Exported_Functions TS Exported Functions
  * @{
  */

/**
  * @brief  Initializes the TS mode according to the specified parameters in
  *         the TS_InitTypeDef and create the associated handle.
  * @param  hts  Pointer to a TS_HandleTypeDef structure that contains
  *                the configuration information for the specified TS module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TS_Init(TS_HandleTypeDef *hts)
{
    /* Check Null pointer */
    if((hts == NULL)||(hts->Instance == NULL))
    {
        return HAL_ERROR;
    }
    
     /* Unlock the TS setting register*/
    __HAL_TS_WRITE_UNLOCK();

    if((hts->Init.Mode & TS_CFG_PD) != TS_CFG_PD)
    {
        if((hts->Init.ChopEn & TS_VREFCFG_CHOP_CLK_EN) == TS_VREFCFG_CHOP_CLK_EN)
        {
            if(hts->Init.ChopDiv == 0)
            {
                /* Return to the initial default value */
                hts->Init.ChopDiv = 0xEF;  
            }

			/* Configure the CHOP_CLK_EN BIT according to the hts->Init.ChopEn value
			Configure CHOP_CLK_DIV BIT according to the hts->Init.ChopDiv value */
			
            MODIFY_REG(hts->Instance->TSVREFCFG, TS_VREFCFG_CHOP_CLK_EN_Msk | TS_VREFCFG_CHOP_CLK_DIV_Msk, \
                hts->Init.ChopEn | (hts->Init.ChopDiv<<4) );
        }
        
		/* SET the TS to PowerDown mode */
		WRITE_REG(hts->Instance->CFG, TS_CFG_PD);
		
		/* Configure the MODE BIT according to the hts->Init.WorkMode value */ 
		MODIFY_REG(hts->Instance->CFG, TS_CFG_MODE, hts->Init.WorkMode);
		
		/* SET the TS to normal mode */
		CLEAR_BIT(hts->Instance->CFG, TS_CFG_PD);
		
		/* SET TS interrupt according to the hts->Init.irq_en */ 
		if(hts->Init.irq_en == ENABLE)
		{
			SET_BIT(hts->Instance->CFG, TS_CFG_IRQ_EN);
			NVIC_ClearPendingIRQ(TS_IRQn);
			NVIC_EnableIRQ(TS_IRQn);
		}
		else
		{
			CLEAR_BIT(hts->Instance->CFG, TS_CFG_IRQ_EN);
			NVIC_ClearPendingIRQ(TS_IRQn);
			NVIC_DisableIRQ(TS_IRQn);
		}
    }
    else
    {
        /* TS power down*/
        SET_BIT(hts->Instance->TSVREFCFG, TS_VREFCFG_PD);
    }
    
    /* Unlock the TS setting register */
    __HAL_TS_WRITE_LOCK();
    
     return HAL_OK;
}

/**
  * @brief  Handles TS interrupt request
  * @param  hts pointer to a TS_HandleTypeDef structure that contains
  *         the configuration information for the specified TS.
  * @retval None
  */
void HAL_TS_IRQHandler(TS_HandleTypeDef* hts)
{
	/* Conversion complete callback */
	HAL_TS_ConvCpltCallback(hts);
}

/**
  * @brief  TS get data.
  * @param  hts  Pointer to a TS_HandleTypeDef structure that contains
  *                the configuration information for the specified TS module.
  * @retval HAL status
  */
uint16_t HAL_TS_get_data(TS_HandleTypeDef *hts)
{
  static uint16_t rdata = 0;
	/* Unlock the TS setting register*/
    __HAL_TS_WRITE_UNLOCK();
	
	if((hts->Instance->DATA & TS_DATA_UPDATED) != TS_DATA_UPDATED){
    return rdata;
  }
	
	if(hts->Init.WorkMode == TS_HIGH_SPEED_MODE)
	{
		rdata = (hts->Instance->DATA & 0x3ff);
	}
	else
	{
		rdata = (hts->Instance->DATA & 0xfff);
	}

  return rdata;
}

/**
  * @brief  Calculated temperature sensor calculates temperature
  * @param  hts  Pointer to a TS_HandleTypeDef structure that contains
  *                the configuration information for the specified TS module.
  * @param  code     Real-time data acquired by temperature sensor
  * @return Temperature sensor converts temperature value.
  */
float calculate_temp(TS_HandleTypeDef *hts, uint16_t code)
{
	float t=0;
    uint16_t code1;

	/* Unlock the TS setting register*/
    __HAL_TS_WRITE_UNLOCK();
	
    code1 = ((*(volatile uint32_t *)(0x40080030))& 0xfff);
	
    if(code1 == 0xfff)
    {
        code1 =1785;
    }
	
	if((hts->Instance->CFG & TS_CFG_MODE_Msk) == TS_HIGH_SPEED_MODE)
	{
		t = ((code - (code1 / 4)) / (K / 4)) + 25;
	}
	else
	{
		t = ((code - code1) / K) + 25;
	}
	
	return t;
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hts pointer to a TS_HandleTypeDef structure that contains
  *         the configuration information for the specified TS.
  * @retval None
  */
__weak void HAL_TS_ConvCpltCallback(TS_HandleTypeDef* hts)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hts);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TS_ConvCpltCallback could be implemented in the user file
   */
}

/**
  * @}
  */
#endif /* HAL_TS_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
