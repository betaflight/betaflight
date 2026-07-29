/**
  ******************************************************************************
  * @file     um324xx_hal_gpio.c
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

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */


/** @defgroup GPIO GPIO
  * @brief GPIO HAL module driver
  * @{
  */

#ifdef HAL_GPIO_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup GPIO_Private_Constants GPIO Private Constants
  * @{
  */

#define GPIO_NUMBER           16U
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32x42x device
  * @param  GPIO_Init pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    uint32_t position = 0;
    uint32_t ioposition = 0x00U;
    uint32_t iocurrent = 0x00U;
    uint32_t temp = 0x00U;

    /* Configure the port pins */
    for(position = 0; position < GPIO_NUMBER; position++)
    {
        /* Get the IO position */
        ioposition = 0x01U << position;
        /* Get the current IO position */
        iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

        /*--------------------- GPIO Mode Configuration ------------------------*/
        if(iocurrent == ioposition)
        {

            if(((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT) || \
                    (GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
            {
                /* Configure the IO Speed */
                temp = GPIOx->SR;
                temp &= ~(GPIO_SR_SR_0 << position);
                temp |= (GPIO_Init->Speed << position);
                GPIOx->SR = temp;

                /* Configure the IO drive capability */
                temp = GPIOx->DS;
                temp &= ~(GPIO_DS_20MA << (position*2U));
                temp |= (GPIO_Init->Driving_Ability << (position*2U));
                GPIOx->DS = temp;

            }

            if((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG)
            {
                if(((GPIOx == GPIOA) && (ioposition == GPIO_PIN_14)) || \
                        ((GPIOx == GPIOB) && (ioposition == GPIO_PIN_4)))
                {
                    temp = GPIOx->PULL;
                    temp &= ~((GPIO_PULL_PS_0|GPIO_PULL_PE_0) << position);

                    if(GPIO_Init->Pull == (uint32_t)(GPIO_PULLUP))
                    {
                        temp |= (GPIO_PULL_PE_0 << position);
                    }
                    else if(GPIO_Init->Pull == (uint32_t)(GPIO_PULLDOWN))
                    {
                        temp |= ((GPIO_PULL_PS_0|GPIO_PULL_PE_0) << position);
                    }
                }
                else
                {
                    /* Activate the Pull-up or Pull down resistor for the current IO */
                    temp = GPIOx->PULL;
                    temp &= ~((GPIO_PULL_PS_0|GPIO_PULL_PE_0) << position);
                    temp |= (GPIO_Init->Pull << position);
                }
                GPIOx->PULL = temp;
            }

            /*GPIO AF config*/
            if((GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
            {
                if(iocurrent < GPIO_PIN_8)
                {
                    temp = GPIOx->AFL;
                    temp &= ~(0xf << (position * 4U));
                    temp |= (GPIO_Init->Alternate << (position * 4U));
                    GPIOx->AFL = temp;
                }
                else
                {
                    temp = GPIOx->AFH;
                    temp &= ~(0xf << ((position-8) * 4U));
                    temp |= (GPIO_Init->Alternate << ((position-8) * 4U));
                    GPIOx->AFH = temp;

                }
            }

            /* Configure GPIO filtering function */
            if(GPIO_Init->Gpio_DbEn == GPIO_DB_EN)
            {
                GPIOx->DBL = GPIO_Init->Gpio_Dbl;

                temp = GPIOx->DBEN;
                temp &= ~(GPIO_DB_EN << (position));
                temp |= (GPIO_DB_EN << (position));
                GPIOx->DBEN = temp;

            }

            /* Configure IO input type for the selected pins */
            temp = GPIOx->IM;
            temp &= ~(GPIO_IM_IM_0 << (position));
            temp |= (GPIO_Init->Gpio_Im << (position));
            GPIOx->IM = temp;

            /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
            temp = GPIOx->MODE;
            temp &= ~(GPIO_MODE << (position * 2U));
            temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
            GPIOx->MODE = temp;

            /*--------------------- EXTI Mode Configuration ------------------------*/
            /* Configure the External Interrupt for the current IO */
            if((GPIO_Init->Mode & EXTI_MODE_Mask) != 0x00U)
            {
                temp = GPIOx->IS;
                temp &= ~(GPIO_IS_IS_0 << (position));
                if((GPIO_Init->Mode & EXTI_MODE) == TRIGGER_MODE_LEVEL)
                {
                    temp |= (GPIO_IS_IS_0 << (position));
                }
                GPIOx->IS = temp;

                /* Configure the interrupt trigger edge for the current IO */
                temp = GPIOx->IBE;
                temp &= ~(GPIO_IBE_IBE_0 << (position));
                if((GPIO_Init->Mode & EXTI_EGDE_MODE) == TRIGGER_EDGE_DOUBLE)
                {
                    temp |= (GPIO_IBE_IBE_0 << (position));
                }
                GPIOx->IBE = temp;

                /* Configure the interrupt trigger level for the current IO */
                temp = GPIOx->IEV;
                temp &= ~(GPIO_IEV_IEV_0 << (position));
                if(((GPIO_Init->Mode & EXTI_TRIGGER_MODE) == TRIGGER_LEVEL_HIGH) || \
                        ((GPIO_Init->Mode & EXTI_TRIGGER_MODE) == TRIGGER_EDGE_RISING) )
                {
                    temp |= (GPIO_IEV_IEV_0 << (position));
                }
                GPIOx->IEV = temp;

                /* Configure the EXTI_CR resigster for the current IO */
                temp = SYSCFG->EXTICR[position >> 2U];
                temp &= ~(0x0FU << (4U * (position & 0x03U)));
                temp |= ((uint32_t)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U)));
                SYSCFG->EXTICR[position >> 2U] = temp;

                /* Enable  IO pin interrupt */
                if((GPIO_Init->Mode & EXTI_MODE) != 0x00U)
                {
                    temp = GPIOx->IEN;
                    temp &= ~(GPIO_IEN_IEN_0 << position);
                    temp |= (GPIO_IEN_IEN_0 << position);
                    GPIOx->IEN = temp;
                }

                /*  Clear IO pin interrupt */
                temp = GPIOx->IC;
                temp |= ((GPIO_IC_IC_0 << (position)));
                GPIOx->IC = temp;
            }
        }
    }
}

/**
  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device
  * @param  GPIO_Pin specifies the port bit to be written.
  *         This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
    uint32_t position;
    uint32_t ioposition = 0x00U;
    uint32_t iocurrent = 0x00U;

    /* Configure the port pins */
    for(position = 0U; position < GPIO_NUMBER; position++)
    {
        /* Get the IO position */
        ioposition = 0x01U << position;
        /* Get the current IO position */
        iocurrent = GPIO_Pin & ioposition;

        /*--------------------- GPIO Mode Configuration ------------------------*/
        if(iocurrent == ioposition)
        {
            GPIOx->PULL &= ~(0x10001 << position);

            GPIOx->MODE &= ~(0x3 << (position * 2U));

            GPIOx->DS &= ~(0x3 << (position * 2U));

            GPIOx->IM &= ~(0x1 << (position));

            GPIOx->SR &= ~(0x1 << (position));

            if(GPIO_Pin < GPIO_PIN_8)
            {
                GPIOx->AFL &= ~(0xF << (position * 4U));
            }
            else
            {
                GPIOx->AFH &= ~(0xF << (position * 4U));
            }
        }
    }
}

/**
 * @brief  Fills the GPIO_InitStruct member with its default value.
 * @param  GPIO_InitStruct pointer to a GPIO_InitType structure which will
 *         be initialized.
 */
void GPIO_InitStructFunc(GPIO_InitTypeDef* GPIO_InitStruct)
{
    /* Reset GPIO init structure parameters values */
    GPIO_InitStruct->Pin                = GPIO_PIN_All;
    GPIO_InitStruct->Mode               = GPIO_MODE_INPUT;
    GPIO_InitStruct->Gpio_Dbl           = GPIO_DBL_INIT;
    GPIO_InitStruct->Gpio_Im            = GPIO_IM_CMOS;
    GPIO_InitStruct->Pull               = GPIO_NOPULL;
    GPIO_InitStruct->Speed              = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct->Driving_Ability    = GPIO_DS_6MA;
    GPIO_InitStruct->Alternate          = GPIO_AFL_AFL;
}

/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device
  * @param  GPIO_Pin specifies the port bit to read.
  *         This parameter can be GPIO_PIN_x where x can be (0..15).
  * @retval The input port pin value.
  */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_PinState bitstatus;

    if((GPIOx->IDATA & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
    {
        bitstatus = GPIO_PIN_SET;
    }
    else
    {
        bitstatus = GPIO_PIN_RESET;
    }
    return bitstatus;
}

/**
  * @brief  Sets or clears the selected data port bit.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  PinState specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if(PinState != GPIO_PIN_RESET)
    {
        GPIOx->SET = GPIO_Pin;
    }
    else
    {
        GPIOx->CLR = GPIO_Pin;
    }

}

/**
  * @brief  Toggles the specified GPIO pins.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device.
  * @param  GPIO_Pin Specifies the pins to be toggled.
  * @retval None
  */
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    /* Set selected pins that were at low level, and reset ones that were high */
    GPIOx->ODATA ^= GPIO_Pin;
}

/**
  * @brief  Locks GPIO Pins configuration registers.
  * @note   The locked registers are GPIOx_MODE, GPIOx_IM, GPIOx_PULL,
  *         GPIOx_IPE, GPIOx_SR,GPIOx_AFH,GPIOx_AFL and GPIOx_DS.
  * @note   The configuration of the locked GPIO pins can no longer be modified
  *         until the next reset.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device.
  * @param  GPIO_Pin specifies the port bit to be locked.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint32_t temp = 0x00U;

    /* Configure IO locking */
    temp = GPIOx->LOCK;
    temp |= GPIO_Pin;
    GPIOx->LOCK = temp;

    /* Read again in order to confirm lock is active */
    if((GPIOx->LOCK & GPIO_Pin) != RESET)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
  * @brief  This function handles EXTI interrupt request.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_IRQHandler(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    /* EXTI line interrupt detected */
    if(__HAL_GPIO_EXTI_GET_IT(GPIOx,GPIO_Pin) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIOx,GPIO_Pin);

        HAL_GPIO_EXTI_Callback(GPIOx,GPIO_Pin);
    }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
__weak void HAL_GPIO_EXTI_Callback(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(GPIOx);
    UNUSED(GPIO_Pin);
    /* NOTE: This function Should not be modified, when the callback is needed,
             the HAL_GPIO_EXTI_Callback could be implemented in the user file
     */
}


#endif /* HAL_GPIO_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


