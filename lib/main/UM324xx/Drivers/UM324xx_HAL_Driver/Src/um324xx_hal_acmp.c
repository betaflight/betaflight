/**
  ******************************************************************************
  * @file     um324xx_hal_acmp.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-17
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

/** @addtogroup UM324xx_HAL_Examples
  * @{
  */

/** @defgroup ACMP ACMP
  * @brief ACMP HAL module driver
  * @{
  */
#ifdef HAL_ACMP_MODULE_ENABLED

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @defgroup ACMP_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void ACMP_SetConfig(ACMP_HandleTypeDef *hacmp);

/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  DeInitialize the ACMP peripheral.
  * @param  hacmp Pointer to a ACMP_InitTypeDef structure that contains
  *         the configuration information for the specified ACMP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ACMP_DeInit(ACMP_HandleTypeDef *hacmp)
{
    /* Check the ACMP handle allocation */
    if (hacmp == NULL)
    {
        return HAL_ERROR;
    }

    hacmp->gState = HAL_ACMP_STATE_BUSY;

    /* Unlock ACMP register */
    HAL_ACMP_Unlock();

    /* Configure registers to default values. */
    __HAL_ACMP_DISABLE(hacmp->Acmpx);

    /* DeInit the low level hardware */
    HAL_ACMP_MspDeInit(hacmp);

    /* Initialize the ACMP state */
    hacmp->gState = HAL_ACMP_STATE_RESET;
    hacmp->Lock = HAL_UNLOCKED;

    /* Process Unlock */
    __HAL_UNLOCK(hacmp);

    return HAL_OK;
}

/**
  * @brief  Initializes the ACMP according to the specified parameters
  *         in the ACMP_HandleTypeDef and initialize the associated handle.
  * @param  hacmp Pointer to a ACMP_HandleTypeDef structure that contains
  *                the configuration information for the specified ACMP.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ACMP_Init(ACMP_HandleTypeDef *hacmp)
{
    /* Check the ACMP handle allocation */
    if (hacmp == NULL)
    {
        return HAL_ERROR;
    }

    if (hacmp->gState == HAL_ACMP_STATE_RESET)
    {
        /* About the configuration of ACMP */
        /* Allocate lock resource and initialize it */
        hacmp->Lock = HAL_UNLOCKED;

        /* Init the low level hardware : GPIO, NVIC */
        HAL_ACMP_MspInit(hacmp);
    }
    hacmp->gState = HAL_ACMP_STATE_BUSY;

    /* Disable the peripheral */
    __HAL_ACMP_DISABLE(hacmp->Acmpx);

    /* Set the ACMP Communication parameters */
    ACMP_SetConfig(hacmp);

    /* Enable the peripheral */
    __HAL_ACMP_ENABLE(hacmp->Acmpx);

    /* Initialize the ACMP state */
    hacmp->gState = HAL_ACMP_STATE_READY;

    return HAL_OK;

}

/**
 * @brief  Get the comparison result of acmp
 * @param  hacmp Pointer to a ACMP_HandleTypeDef structure that contains
 *                the configuration information for the specified ACMP.
 * @return the ouput status of the acmp
 */
uint8_t HAL_ACMP_GetOutput_Status(ACMP_HandleTypeDef *hacmp)
{
    uint8_t acmp_status = 0;

    /* Get ACMP0 output status */
    if(hacmp->Acmpx == ACMP0)
    {
        acmp_status = ((ACMP->CFG0>>ACMP_CFG0_OUTPUT_Pos)&0x01);
    }
    /* Get ACMP1 output status */
    else if(hacmp->Acmpx == ACMP1)
    {
        acmp_status = ((ACMP->CFG1>>ACMP_CFG1_OUTPUT_Pos)&0x01);
    }
    /* Get ACMP2 output status */
    else
    {
        acmp_status = ((ACMP->CFG2>>ACMP_CFG2_OUTPUT_Pos)&0x01);
    }
    return acmp_status;
}

/**
  * @brief This function handles ACMP interrupt request.
  * @param  hacmp Pointer to a ACMP_HandleTypeDef structure that contains
  *                the configuration information for the specified ACMP.
  * @retval None
  */
void HAL_ACMP_IRQHandler(ACMP_HandleTypeDef *hacmp)
{
    /* Unlock ACMP register */
    HAL_ACMP_Unlock();

    /* ACMP interrupt detected */
    if(__HAL_ACMP_GET_IT_FLAG(hacmp->Acmpx) != RESET)
    {
        __HAL_ACMP_CLEAR_IT_FLAG(hacmp->Acmpx);
        HAL_ACMP_Callback(hacmp);

    }
    /* Lock ACMP register */
    HAL_ACMP_Lock();

}

/**
  * @brief  ACMP detection callbacks.
  * @param  hacmp Pointer to a ACMP_HandleTypeDef structure that contains
  *                the configuration information for the specified ACMP.
  * @retval None
  */
__weak void HAL_ACMP_Callback(ACMP_HandleTypeDef *hacmp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hacmp);
    /* NOTE: This function Should not be modified, when the callback is needed,
             the HAL_ACMP_Callback could be implemented in the user file
     */
}

/**
  * @brief  Initialize the ACMP MSP.
  * @param  hacmp Pointer to a ACMP_HandleTypeDef structure that contains
  *                the configuration information for the specified ACMP.
  * @retval None
  */
__weak void HAL_ACMP_MspInit(ACMP_HandleTypeDef *hacmp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hacmp);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_ACMP_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitialize the ACMP MSP.
  * @param  hacmp Pointer to a ACMP_HandleTypeDef structure that contains
  *                the configuration information for the specified ACMP.
  * @retval None
  */
__weak void HAL_ACMP_MspDeInit(ACMP_HandleTypeDef *hacmp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hacmp);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_ACMP_MspDeInit could be implemented in the user file
     */
}


/**
  * @brief  Unlock the ACMP control register access
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_ACMP_Unlock(void)
{
    /* Unlock ACMP register */
    ACMP->UNLOCK = 0xA5A55A5A ;

    return HAL_OK;
}

/**
  * @brief  Locks the ACMP control register access
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_ACMP_Lock(void)
{
    /* Lock ACMP register */
    ACMP->UNLOCK =0xFFFFFFFF;

    return HAL_OK;
}

/**
  * @brief  Configures the ACMP peripheral.
  * @param  hacmp  Pointer to a ACMP_HandleTypeDef structure that contains
  *                the configuration information for the specified ACMP module.
  * @retval None
  */
static void ACMP_SetConfig(ACMP_HandleTypeDef *hacmp)
{
    __HAL_ACMP_DISABLE(hacmp->Acmpx);

    /* Unlock ACMP register */
    HAL_ACMP_Unlock();

    if(hacmp->Acmpx == ACMP0)
    {
        /* Selection of Negative input resistance voltage division */
        ACMP->CFG0 &=~ ACMP_CFG0_CRVINCTRL_Msk;
        ACMP->CFG0 |= (hacmp->Init.CrvinCtrl);

        /* Voltage dividing ratio of negative input resistance */
        ACMP->CFG0 &=~ ACMP_CFG0_CRVCTRL_Msk;
        ACMP->CFG0 |= (hacmp->Init.CrvCtrl);

        /* Channel selection of negative input */
        ACMP->CFG0 &=~ ACMP_CFG0_CNEGSEL_Msk;
        ACMP->CFG0 |= (hacmp->Init.CnegSel);

        /* Hysteresis voltage selection */
        ACMP->CFG0 &=~ ACMP_CFG0_CHYS_Msk;
        ACMP->CFG0 |= (hacmp->Init.ChySel);

        /* The low power mode switch */
        ACMP->CFG0 &=~ ACMP_CFG0_CLPM_Msk;
        ACMP->CFG0 |= (hacmp->Init.Clpm);

        /* Interrupt generation mode */
        ACMP->CFG0 &=~ ACMP_CFG0_INT_CFG_Msk;
        ACMP->CFG0 |= (hacmp->Init.Interrupt_Mode);

        /* Enable ACMP0 */
        ACMP->CFG0 &=~ ACMP_CFG0_EN_Msk;
        ACMP->CFG0 |= (ACMP_CFG0_EN);
    }

    if(hacmp->Acmpx == ACMP1)
    {

        /* Selection of Negative input resistance voltage division */
        ACMP->CFG1 &=~(ACMP_CFG1_CRVINCTRL_Msk);
        ACMP->CFG1 |= (hacmp->Init.CrvinCtrl);

        /* Voltage dividing ratio of negative input resistance */
        ACMP->CFG1 &=~(ACMP_CFG1_CRVCTRL_Msk);
        ACMP->CFG1 |= (hacmp->Init.CrvCtrl );

        /* Channel selection of negative input */
        ACMP->CFG1 &=~(ACMP_CFG1_CNEGSEL_Msk);
        ACMP->CFG1 |= (hacmp->Init.CnegSel);

        /* Hysteresis voltage selection */
        ACMP->CFG1 &=~(ACMP_CFG1_CHYS_Msk);
        ACMP->CFG1 |= (hacmp->Init.ChySel);

        /* The low power mode switch */
        ACMP->CFG1 &=~(ACMP_CFG1_CLPM_Msk);
        ACMP->CFG1 |= (hacmp->Init.Clpm);

        /* Interrupt generation mode */
        ACMP->CFG1 &=~(ACMP_CFG1_INT_CFG_Msk);
        ACMP->CFG1 |= (hacmp->Init.Interrupt_Mode);

        /* Enable ACMP1 */
        ACMP->CFG1 &=~(ACMP_CFG1_EN_Msk);
        ACMP->CFG1 |= (ACMP_CFG1_EN);

    }

    if(hacmp->Acmpx == ACMP2)
    {

        /* Selection of Negative input resistance voltage division */
        ACMP->CFG2 &=~(ACMP_CFG2_CRVINCTRL_Msk);
        ACMP->CFG2 |= (hacmp->Init.CrvinCtrl);

        /* Voltage dividing ratio of negative input resistance */
        ACMP->CFG2 &=~(ACMP_CFG2_CRVCTRL_Msk);
        ACMP->CFG2 |= (hacmp->Init.CrvCtrl);

        /* Channel selection of negative input */
        ACMP->CFG2 &=~(ACMP_CFG2_CNEGSEL_Msk);
        ACMP->CFG2 |= (hacmp->Init.CnegSel);

        /* Hysteresis voltage selection */
        ACMP->CFG2 &=~(ACMP_CFG2_CHYS_Msk);
        ACMP->CFG2 |= (hacmp->Init.ChySel);

        /* The low power mode switch */
        ACMP->CFG2 &=~(ACMP_CFG2_CLPM_Msk);
        ACMP->CFG2 |= (hacmp->Init.Clpm);

        /* Interrupt generation mode */
        ACMP->CFG2 &=~(ACMP_CFG2_INT_CFG_Msk);
        ACMP->CFG2 |= (hacmp->Init.Interrupt_Mode);

        /* Enable ACMP2 */
        ACMP->CFG2 &=~(ACMP_CFG2_EN_Msk);
        ACMP->CFG2 |= (ACMP_CFG2_EN);

    }
    /* Lock ACMP register */
    HAL_ACMP_Lock();

}


/**
  * @}
  */
#endif /* HAL_ACMP_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
