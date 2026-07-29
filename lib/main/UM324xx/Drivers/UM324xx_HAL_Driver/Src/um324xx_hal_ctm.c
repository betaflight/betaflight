/**
  ******************************************************************************
  * @file     um324xx_hal_ctm.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2024-07-1
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

/** @addtogroup um324xx_HAL_Driver
  * @{
  */
  

/** @defgroup CTM CTM
  * @brief CTM HAL module driver
  * @{
  */
  
#ifdef HAL_CTM_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup CTM_Private_Constants CTM Private Constants
  * @{
  */


/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
  * @brief  This function handles CTM Initializes.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CTM_Init(CTM_HandleTypeDef *hctm)
{
    /* Check the hctm handle allocation */
    if (hctm == NULL)
    {
        return HAL_ERROR;
    }
    
    /*Frequency measurement disable*/
    __HAL_CTM_FRQ_MEASUREMENT_DISABLE(hctm);
    
#if (USE_HAL_CTM_REGISTER_CALLBACKS == 1)
    if (hctm->MspDeInitCallback == NULL)
    {
        hctm->MspDeInitCallback = HAL_CTM_MspInit;
    }
    /* DeInit the low level hardware */
    hctm->MspDeInitCallback(hctm);
#else
    /* DeInit the low level hardware */
    HAL_CTM_MspInit(hctm);
#endif /* (USE_HAL_CTM_REGISTER_CALLBACKS) */
    
    MODIFY_REG(CTM->CTRL1, 0XFFFF, (hctm->Init.Digital_Filter | hctm->Init.Ref_ClkDiv | hctm->Init.Ref_ClkSel |\
    hctm->Init.REF_SignalEdgeSel | hctm->Init.Object_ClkSel | hctm->Init.Object_ClkDiv));
    
    if(hctm->Init.Ref_SignalSourceSel == CTM_RPS_INTINPUT)
    {
        SET_BIT(hctm->Instance->CTRL1, CTM_RPS_INTINPUT);
        CLEAR_BIT(hctm->Instance->CTRL1,CTM_REFE_INPUT_ENABLE);
    }
    else
    {
        CLEAR_BIT(hctm->Instance->CTRL1, CTM_RPS_INTINPUT);
        SET_BIT(hctm->Instance->CTRL1, CTM_REFE_INPUT_ENABLE);
    }
    
    /*Write upper limit value*/
    WRITE_REG(CTM->ULVR,hctm->Init.UpLimit_Value);
    
    /*Write low limit value*/
    WRITE_REG(CTM->LLVR,hctm->Init.LowLimit_Value);
    
    /*Config ISR*/
    if(hctm->Init.Irq != CTM_IRQ_NONE)
    {
        WRITE_REG(CTM->IRCR,hctm->Init.Irq);
    }
    

    
    return HAL_OK;

}

/**
  * @brief  This function CTM start.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
void HAL_CTM_Start(CTM_HandleTypeDef *hctm)
{
    __HAL_CTM_FRQ_MEASUREMENT_ENABLE(hctm);
   
}


/**
  * @brief  This function CTM stop.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
void HAL_CTM_Stop(CTM_HandleTypeDef *hctm)
{
    __HAL_CTM_FRQ_MEASUREMENT_DISABLE(hctm);
   
}


/**
  * @brief  This function handles CTM interrupt request.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
void HAL_CTM_IRQHandler(CTM_HandleTypeDef *hctm)
{
    uint32_t isrflags  = READ_REG(hctm->Instance->ISR);
    uint32_t irqflags  = READ_REG(hctm->Instance->IRCR);
    
    if (((isrflags & CTM_ISR_MENDF) != RESET) &&  ((irqflags & CTM_IRQ_MEDNIE) != RESET))
    {
        __HAL_CTM_CLEAR_IRQ_FLAG(hctm,CTM_ISR_MENDF);       //Clear measurement completed flag
        
          /*  User through measurement completed callback */
#if (USE_HAL_CTM_REGISTER_CALLBACKS == 1)
            /*Measurement completed callback*/
            hctm->MeasureCpltCallback(hctm);
#else
            /*Measurement completed callback*/
            HAL_CTM_MeasureCpltCallback(hctm);
#endif /* USE_HAL_CTM_REGISTER_CALLBACKS */

    }
    
    if (((isrflags & CTM_ISR_OVFF) != RESET) &&  ((irqflags & CTM_IRQ_OVFIE) != RESET))
    {
        __HAL_CTM_CLEAR_IRQ_FLAG(hctm,CTM_ISR_OVFF);       
        
          /*  User through OverFlow callback */
#if (USE_HAL_CTM_REGISTER_CALLBACKS == 1)
            /*OverFlow callback*/
            hctm->HAL_CTM_OverFlowCallback(hctm);
#else
            /*OverFlow callback*/
            HAL_CTM_OverFlowCallback(hctm);
#endif /* USE_HAL_CTM_REGISTER_CALLBACKS */

    }
    
    if (((isrflags & CTM_ISR_FERRF) != RESET) &&  ((irqflags & CTM_IRQ_FERRIE) != RESET))
    {
        __HAL_CTM_CLEAR_IRQ_FLAG(hctm,CTM_ISR_FERRF);       
        
          /*  User through abnormal frequency callback */
#if (USE_HAL_CTM_REGISTER_CALLBACKS == 1)
            /*Measurement completed callback*/
            hctm->HAL_CTM_FrqAbnormalCallback(hctm);
#else
            /*Abnormal frequency callback*/
            HAL_CTM_FrqAbnormalCallback(hctm);
#endif /* USE_HAL_CTM_REGISTER_CALLBACKS */

    }
    
    /* Clear Flag*/
    __HAL_CTM_CLEAR_IRQ_FLAG(hctm,CTM_ISR_MSK);
    
}


/**
  * @brief  Measurement completed callbacks.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
__weak void HAL_CTM_MeasureCpltCallback(CTM_HandleTypeDef *hctm)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hctm);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_CTM_MeasureCpltCallback could be implemented in the user file
   */
}


/**
  * @brief  counter overflow callbacks.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
__weak void HAL_CTM_OverFlowCallback(CTM_HandleTypeDef *hctm)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hctm);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_CTM_MeasureCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Abnormal frequency callbacks.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
__weak void HAL_CTM_FrqAbnormalCallback(CTM_HandleTypeDef *hctm)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hctm);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_CTM_MeasureCpltCallback could be implemented in the user file
   */
}


/**
  * @brief  CTM MSP Init.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
__weak void HAL_CTM_MspInit(CTM_HandleTypeDef *hctm)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hctm);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_CTM_MspInit could be implemented in the user file
   */
}

/**
  * @brief  CTM MSP DeInit.
  * @param  hctm  Pointer to a CTM_HandleTypeDef structure that contains
  *                the configuration information for the specified CTM module.
  * @retval None
  */
__weak void HAL_CTM_MspDeInit(CTM_HandleTypeDef *hctm)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hctm);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_CTM_MspInit could be implemented in the user file
   */
}
#endif /* HAL_CTM_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

