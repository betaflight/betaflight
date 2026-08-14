/**
  ******************************************************************************
  * @file     um324xx_hal_crc.c
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

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */
#ifdef HAL_CRC_MODULE_ENABLED

/** @defgroup CRC_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup CRC_Exported_Functions CRC Exported Functions
  * @{
  */

/**
  * @brief  Initialize the CRC according to the specified
  *         parameters in the CRC_InitTypeDef and create the associated handle.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc)
{
    /* Check the CRC handle allocation */
    if (hcrc == NULL)
    {
        return HAL_ERROR;
    }

    if (hcrc->State == HAL_CRC_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hcrc->Lock = HAL_UNLOCKED;
        /* Init the low level hardware */
        HAL_CRC_MspInit(hcrc);
    }

    MODIFY_REG(hcrc->Instance->CFG, CRC_CFG_POL | CRC_CFG_WIDTH_DIN | CRC_CFG_DIN_REV | CRC_CFG_DOUT_REV | CRC_CFG_INIT_REV,
               hcrc->polynomial | hcrc->width_din | hcrc->din_rev | hcrc->dout_rev | hcrc->init_rev);

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  DeInitialize the CRC peripheral.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc)
{
    /* Check the CRC handle allocation */
    if (hcrc == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the CRC peripheral state */
    if (hcrc->State == HAL_CRC_STATE_BUSY)
    {
        return HAL_BUSY;
    }

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_BUSY;

    /* Reset CRC calculation unit */
    __HAL_CRC_DR_RESET(hcrc);

    /* Clart CRC cfg register */
    CLEAR_BIT(hcrc->Instance->CFG, CRC_CFG_POL | CRC_CFG_WIDTH_DIN | CRC_CFG_DIN_REV | CRC_CFG_DOUT_REV | CRC_CFG_INIT_REV);

    /* DeInit the low level hardware */
    HAL_CRC_MspDeInit(hcrc);

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_RESET;

    /* Process unlocked */
    __HAL_UNLOCK(hcrc);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Initializes the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcrc);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_CRC_MspInit can be implemented in the user file
     */
}

/**
  * @brief  DeInitialize the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcrc);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_CRC_MspDeInit can be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group2 Peripheral Control functions
  *  @brief    management functions.
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) compute the 32-bit CRC value of a 32-bit data buffer
          using combination of the previous CRC value and the new one.

       [..]  or

      (+) compute the 32-bit CRC value of a 32-bit data buffer
          independently of the previous CRC value.

@endverbatim
  * @{
  */

/**
  * @brief  Compute the 16-bit CRC value of a 16-bit data buffer
  *         starting with hcrc->Instance->INIT as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer.
  * @param  BufferLength input data buffer length (number of uint32_t words)
  * @param  InitValue  initialization value
  * @param  XorOut  crc result XOR value
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t HAL_CRC16_Calculate(CRC_HandleTypeDef *hcrc, uint16_t pBuffer[], uint32_t BufferLength, uint16_t InitValue, uint16_t XorOut)
{
    uint32_t index;      /* CRC input data buffer index */
    uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DR register) */

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_BUSY;

    /* Set CRC initialization value */
    hcrc->Instance->INIT = InitValue;

    /* Clear the calculation result and load the initial value */
    hcrc->Instance->INIT = InitValue;

    /* Enter Data to the CRC calculator */
    for (index = 0U; index < BufferLength; index++)
    {
        hcrc->Instance->DATA = pBuffer[index];
    }
    temp = hcrc->Instance->DATA^XorOut;

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_READY;

    /* Return the CRC computed value */
    return temp;
}

/**
  * @brief  Compute the 32-bit CRC value of a 32-bit data buffer
  *         starting with hcrc->Instance->INIT as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer.
  * @param  BufferLength input data buffer length (number of uint32_t words)
  * @param  InitValue  initialization value
  * @param  XorOut  crc result XOR value
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t HAL_CRC32_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength, uint32_t InitValue, uint32_t XorOut)
{
    uint32_t index;      /* CRC input data buffer index */
    uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DR register) */

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_BUSY;

    /* Clear the calculation result and load the initial value */
    hcrc->Instance->INIT = InitValue;

    /* Set CRC initialization value */
    SET_BIT(hcrc->Instance->CFG, CRC_CFG_RESET);

    /* Enter Data to the CRC calculator */
    for (index = 0U; index < BufferLength; index++)
    {
        hcrc->Instance->DATA = pBuffer[index];
    }
    temp = hcrc->Instance->DATA^XorOut;

    /* Change CRC peripheral state */
    hcrc->State = HAL_CRC_STATE_READY;

    /* Return the CRC computed value */
    return temp;
}

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group3 Peripheral State functions
  *  @brief    Peripheral State functions.
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Return the CRC handle state.
  * @param  hcrc CRC handle
  * @retval HAL state
  */
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc)
{
    /* Return CRC handle state */
    return hcrc->State;
}

/**
  * @}
  */


#endif /* HAL_CRC_MODULE_ENABLED */
/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
