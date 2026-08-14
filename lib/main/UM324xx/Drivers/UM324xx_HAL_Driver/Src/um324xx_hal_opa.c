/**
  ******************************************************************************
  * @file     um324xx_hal_opa.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-20
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

/** @defgroup OPA OPA
  * @brief HAL OPA module driver
  * @{
  */

#ifdef HAL_OPA_MODULE_ENABLED

/** @defgroup OPA_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup OPA_Private_Constants
  * @{
  */
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup OPA_Private_Functions  OPA Private Functions
  * @{
  */
static void OPA_SetConfig(OPA_HandleTypeDef *hopa);

/**
  * @}
  */
/* Exported functions ---------------------------------------------------------*/
/** @defgroup OPA_Exported_Functions OPA Exported Functions
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
  * @brief  DeInitialize the OPA peripheral.
  * @param  hopa Pointer to a OPA_InitTypeDef structure that contains
  *         the configuration information for the specified OPA.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_OPA_DeInit(OPA_HandleTypeDef *hopa)
{
    /* Check the OPA handle allocation */
    if (hopa == NULL)
    {
        return HAL_ERROR;
    }

    /* Unlock OPA register */
    HAL_OPA_Unlock();

    /* Configure registers to default values. */
    __HAL_OPA_DISABLE(hopa->Opax);

    /* DeInit the low level hardware */
    HAL_OPA_MspDeInit(hopa);

    /* Initialize the OPA state */
    hopa->Lock = HAL_UNLOCKED;

    /* Process Unlock */
    __HAL_UNLOCK(hopa);

    return HAL_OK;
}

/**
  * @brief  Initializes the OPA according to the specified parameters
  *         in the OPA_HandleTypeDef and initialize the associated handle.
  * @param  hopa Pointer to a OPA_HandleTypeDef structure that contains
  *                the configuration information for the specified OPA.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_OPA_Init(OPA_HandleTypeDef *hopa)
{
    /* Check the OPA handle allocation */
    if (hopa == NULL)
    {
        return HAL_ERROR;
    }

    /* About the configuration of OPA */
    /* Allocate lock resource and initialize it */
    hopa->Lock = HAL_UNLOCKED;

    /* Init the low level hardware : GPIO, NVIC */
    HAL_OPA_MspInit(hopa);

    /* Disable the peripheral */
    __HAL_OPA_DISABLE(hopa->Opax);

    /* Set the OPA Communication parameters */
    OPA_SetConfig(hopa);

    /* Enable the peripheral */
    __HAL_OPA_ENABLE(hopa->Opax);

    return HAL_OK;
}

/**
 * @brief  Get the comparison result of OPA when OPA as a comparator.
 * @param  hopa Pointer to a OPA_HandleTypeDef structure that contains
 *                the configuration information for the specified OPA.
 * @return the ouput status of the acmp
 */
_Bool HAL_OPA_GetOutput_Status(OPA_HandleTypeDef *hopa)
{
    _Bool acmp_status = 0;

    /* Get OPA0 output status */
    if(hopa->Opax == OPA0)
    {
        acmp_status = ((OPA->CFG0>>OPA_CFG0_COMPOUT_Pos)&0x01);
    }
    /* Get OPA1 output status */
    if(hopa->Opax == OPA1)
    {
        acmp_status = ((OPA->CFG1>>OPA_CFG1_COMPOUT_Pos)&0x01);
    }
   #if defined(UM32x42x)
     /* Get OPA2 output status */
    if(hopa->Opax == OPA2)
    {
        acmp_status = ((OPA->CFG2>>OPA_CFG2_COMPOUT_Pos)&0x01);
    }
   #endif
    return acmp_status;
}

/**
  * @brief This function handles OPA interrupt request.
  * @param  hopa Pointer to a OPA_HandleTypeDef structure that contains
  *                the configuration information for the specified OPA.
  * @retval None
  */
void HAL_OPA_IRQHandler(OPA_HandleTypeDef *hopa)
{
    /* Unlock OPA register */
    HAL_OPA_Unlock();

    /* OPA interrupt detected */
    if(__HAL_OPA_GET_IT_FLAG(hopa->Opax) != RESET)
    {
        __HAL_OPA_CLEAR_IT_FLAG(hopa->Opax);
        HAL_OPA_Callback(hopa);

    }
    /* Lock OPA register */
    HAL_OPA_Lock();
}

/**
  * @brief  OPA detection callbacks.
  * @param  hopa Pointer to a OPA_HandleTypeDef structure that contains
  *                the configuration information for the specified OPA.
  * @retval None
  */
__weak void HAL_OPA_Callback(OPA_HandleTypeDef *hopa)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hopa);
    /* NOTE: This function Should not be modified, when the callback is needed,
             the HAL_OPA_Callback could be implemented in the user file
     */
}

/**
  * @brief  Initialize the OPA MSP.
  * @param  hopa Pointer to a OPA_HandleTypeDef structure that contains
  *                the configuration information for the specified OPA.
  * @retval None
  */
__weak void HAL_OPA_MspInit(OPA_HandleTypeDef *hopa)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hopa);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_OPA_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitialize the OPA MSP.
  * @param  hopa Pointer to a OPA_HandleTypeDef structure that contains
  *                the configuration information for the specified OPA.
  * @retval None
  */
__weak void HAL_OPA_MspDeInit(OPA_HandleTypeDef *hopa)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hopa);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_OPA_MspDeInit could be implemented in the user file
     */
}

/**
  * @brief  Unlock the OPA control register access
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_OPA_Unlock(void)
{
    /* Unlock OPA register */
    OPA->UNLOCK = 0xA5A55A5A ;

    return HAL_OK;
}

/**
  * @brief  Locks the OPA control register access
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_OPA_Lock(void)
{
    /* Lock OPA register */
    OPA->UNLOCK =0xFFFFFFFF;

    return HAL_OK;
}

/**
  * @brief  Configures the OPA peripheral.
  * @param  hopa  Pointer to a OPA_HandleTypeDef structure that contains
  *                the configuration information for the specified OPA module.
  * @retval None
  */
static void OPA_SetConfig(OPA_HandleTypeDef *hopa)
{

    /* Unlock OPA register */
    HAL_OPA_Unlock();

    if(hopa->Opax == OPA0)
    {
        /* Disable OPA0 peripheral*/
        OPA->CFG0 &=~ OPA_CFG0_EN_Msk;

        if(hopa->Opa_Mode != HAL_OPA_MODE_CMP)
        {
            /* Disable comparator function interrupt */
            OPA->CFG0 &=~ OPA_CFG0_INT_CFG_Msk;

            /* Comparator function enable signal */
            OPA->CFG0 &=~ OPA_CFG0_COMPEN_Msk;

            /* PGA internal feedback capacitor enable signal */
            OPA->CFG0 &=~ OPA_CFG0_CAPEN_Msk;
            OPA->CFG0 |= (hopa->Init.Capen);

            /* PGA internal feedback resistor enable signal */
            OPA->CFG0 &=~ OPA_CFG0_FBRESEN_Msk;
            OPA->CFG0 |= (hopa->Init.Fbresen);

            /* Signal of the output by OPA to IO pin */
            OPA->CFG0 &=~ OPA_CFG0_OTPEN_Msk;
            OPA->CFG0 |= (hopa->Init.Otpen );

        }
        if(hopa->Opa_Mode == HAL_OPA_MODE_UNITBUFF)
        {
            /* PGA gain selection signal */
            OPA->CFG0 &=~ OPA_CFG0_GAINSEL_Msk;
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_PGA)
        {
            /* PGA gain selection signal */
            OPA->CFG0 &=~ OPA_CFG0_GAINSEL_Msk;
            OPA->CFG0 |= (hopa->Init.GainSel);
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_CMP)
        {
            /* PGA internal feedback resistor enable signal */
            OPA->CFG0 &=~ OPA_CFG0_FBRESEN_Msk;

            /* PGA internal feedback capacitor enable signal */
            OPA->CFG0 &=~ OPA_CFG0_CAPEN_Msk;

            /* Signal of the output by OPA to IO pin */
            OPA->CFG0 &=~ OPA_CFG0_OTPEN_Msk;

            /* PGA gain selection signal */
            OPA->CFG0 &=~ OPA_CFG0_GAINSEL_Msk;
            OPA->CFG0 |= (0x7<<OPA_CFG1_GAINSEL_Pos);

            /* Comparator function enable signal */
            OPA->CFG0 &=~ OPA_CFG0_COMPEN_Msk;
            OPA->CFG0 |= (hopa->Init.Compen);

            if(hopa->Init.Interrupt_Mode !=OPA_INTERRUPT_CLOSE)
            {
                /* Interrupt generation mode */
                OPA->CFG0 &=~ OPA_CFG0_INT_CFG_Msk;
                OPA->CFG0 |= (hopa->Init.Interrupt_Mode);
                __HAL_OPA_CLEAR_IT_FLAG(hopa->Opax);
            }
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_OPA)
        {
            /* PGA gain selection signal */
            OPA->CFG0 &=~ OPA_CFG0_GAINSEL_Msk;
            OPA->CFG0 |= (0x7<<OPA_CFG1_GAINSEL_Pos);
        }

        /* OPA negative channel selection signal */
        OPA->CFG0 &=~ OPA_CFG0_SELN_Msk;
        OPA->CFG0 |= (hopa->Init.Seln );

        /* OPA positive channel selection signal */
        OPA->CFG0 &=~ OPA_CFG0_SELP_Msk;
        OPA->CFG0 |= (hopa->Init.Selp);

        /* Enable OPA0 peripheral*/
        OPA->CFG0 |= (OPA_CFG0_EN);

    }

    if(hopa->Opax == OPA1)
    {
        /* Disable OPA1 peripheral*/
        OPA->CFG1 &=~ OPA_CFG1_EN_Msk;

        if(hopa->Opa_Mode != HAL_OPA_MODE_CMP)
        {
            /* Disable comparator function interrupt */
            OPA->CFG1 &=~ OPA_CFG1_INT_CFG_Msk;

            /* Comparator function enable signal */
            OPA->CFG1 &=~ OPA_CFG1_COMPEN_Msk;

            /* PGA internal feedback capacitor enable signal */
            OPA->CFG1 &=~ OPA_CFG1_CAPEN_Msk;
            OPA->CFG1 |= (hopa->Init.Capen);

            /* PGA internal feedback resistor enable signal */
            OPA->CFG1 &=~ OPA_CFG1_FBRESEN_Msk;
            OPA->CFG1 |= (hopa->Init.Fbresen);

            /* Signal of the output by OPA to IO pin */
            OPA->CFG1 &=~ OPA_CFG1_OTPEN_Msk;
            OPA->CFG1 |= (hopa->Init.Otpen );

        }
        if(hopa->Opa_Mode == HAL_OPA_MODE_UNITBUFF)
        {
            /* PGA gain selection signal */
            OPA->CFG1 &=~ OPA_CFG1_GAINSEL_Msk;
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_PGA)
        {
            /* PGA gain selection signal */
            OPA->CFG1 &=~ OPA_CFG1_GAINSEL_Msk;
            OPA->CFG1 |= (hopa->Init.GainSel);
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_CMP)
        {
            /* PGA internal feedback resistor enable signal */
            OPA->CFG1 &=~ OPA_CFG1_FBRESEN_Msk;

            /* PGA internal feedback capacitor enable signal */
            OPA->CFG1 &=~ OPA_CFG1_CAPEN_Msk;

            /* Signal of the output by OPA to IO pin */
            OPA->CFG1 &=~ OPA_CFG1_OTPEN_Msk;

            /* PGA gain selection signal */
            OPA->CFG1 &=~ OPA_CFG1_GAINSEL_Msk;
            OPA->CFG1 |= (0x7<<OPA_CFG1_GAINSEL_Pos);

            /* Comparator function enable signal */
            OPA->CFG1 &=~ OPA_CFG1_COMPEN_Msk;
            OPA->CFG1 |= (hopa->Init.Compen);

            if(hopa->Init.Interrupt_Mode !=OPA_INTERRUPT_CLOSE)
            {
                /* Interrupt generation mode */
                OPA->CFG1 &=~ OPA_CFG1_INT_CFG_Msk;
                OPA->CFG1 |= (hopa->Init.Interrupt_Mode);
                __HAL_OPA_CLEAR_IT_FLAG(hopa->Opax);
            }
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_OPA)
        {
            /* PGA gain selection signal */
            OPA->CFG1 &=~ OPA_CFG1_GAINSEL_Msk;
            OPA->CFG1 |= (0x7<<OPA_CFG1_GAINSEL_Pos);
        }

        /* OPA negative channel selection signal */
        OPA->CFG1 &=~ OPA_CFG1_SELN_Msk;
        OPA->CFG1 |= (hopa->Init.Seln);

        /* OPA positive channel selection signal */
        OPA->CFG1 &=~ OPA_CFG1_SELP_Msk;
        OPA->CFG1 |= (hopa->Init.Selp);

        /* Enable OPA1 peripheral*/
        OPA->CFG1 |= (OPA_CFG1_EN);

    }
     #if defined(UM32x42x)
    if(hopa->Opax == OPA2)
    {
        /* Disable OPA1 peripheral*/
        OPA->CFG2 &=~ OPA_CFG2_EN_Msk;

        if(hopa->Opa_Mode != HAL_OPA_MODE_CMP)
        {
            /* Disable comparator function interrupt */
            OPA->CFG2 &=~ OPA_CFG2_INT_CFG_Msk;

            /* Comparator function enable signal */
            OPA->CFG2 &=~ OPA_CFG2_COMPEN_Msk;

            /* PGA internal feedback capacitor enable signal */
            OPA->CFG2 &=~ OPA_CFG2_CAPEN_Msk;
            OPA->CFG2 |= (hopa->Init.Capen);

            /* PGA internal feedback resistor enable signal */
            OPA->CFG2 &=~ OPA_CFG2_FBRESEN_Msk;
            OPA->CFG2 |= (hopa->Init.Fbresen);

            /* Signal of the output by OPA to IO pin */
            OPA->CFG2 &=~ OPA_CFG2_OTPEN_Msk;
            OPA->CFG2 |= (hopa->Init.Otpen );

        }
        if(hopa->Opa_Mode == HAL_OPA_MODE_UNITBUFF)
        {
            /* PGA gain selection signal */
            OPA->CFG2 &=~ OPA_CFG2_GAINSEL_Msk;
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_PGA)
        {
            /* PGA gain selection signal */
            OPA->CFG2 &=~ OPA_CFG2_GAINSEL_Msk;
            OPA->CFG2 |= (hopa->Init.GainSel);
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_CMP)
        {
            /* PGA internal feedback resistor enable signal */
            OPA->CFG2 &=~ OPA_CFG1_FBRESEN_Msk;

            /* PGA internal feedback capacitor enable signal */
            OPA->CFG2 &=~ OPA_CFG2_CAPEN_Msk;

            /* Signal of the output by OPA to IO pin */
            OPA->CFG2 &=~ OPA_CFG2_OTPEN_Msk;

            /* PGA gain selection signal */
            OPA->CFG2 &=~ OPA_CFG2_GAINSEL_Msk;
            OPA->CFG2 |= (0x7<<OPA_CFG2_GAINSEL_Pos);

            /* Comparator function enable signal */
            OPA->CFG2 &=~ OPA_CFG2_COMPEN_Msk;
            OPA->CFG2 |= (hopa->Init.Compen);

            if(hopa->Init.Interrupt_Mode !=OPA_INTERRUPT_CLOSE)
            {
                /* Interrupt generation mode */
                OPA->CFG2 &=~ OPA_CFG2_INT_CFG_Msk;
                OPA->CFG2 |= (hopa->Init.Interrupt_Mode);
                __HAL_OPA_CLEAR_IT_FLAG(hopa->Opax);
            }
        }

        if(hopa->Opa_Mode == HAL_OPA_MODE_OPA)
        {
            /* PGA gain selection signal */
            OPA->CFG2 &=~ OPA_CFG1_GAINSEL_Msk;
            OPA->CFG2 |= (0x7<<OPA_CFG2_GAINSEL_Pos);
        }

        /* OPA negative channel selection signal */
        OPA->CFG2 &=~ OPA_CFG2_SELN_Msk;
        OPA->CFG2 |= (hopa->Init.Seln);

        /* OPA positive channel selection signal */
        OPA->CFG2 &=~ OPA_CFG2_SELP_Msk;
        OPA->CFG2 |= (hopa->Init.Selp);

        /* Enable OPA2 peripheral*/
        OPA->CFG2 |= (OPA_CFG2_EN);

    }
    #endif
    /* Lock OPA register */
    HAL_OPA_Lock();
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_OPA_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
