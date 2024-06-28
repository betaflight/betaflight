/**
  *
  * @file    apm32f4xx_dal_gpio.c
  * @brief   GPIO DAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the General Purpose Input/Output (GPIO) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  @verbatim
  ==============================================================================
                    ##### GPIO Peripheral features #####
  ==============================================================================
  [..] 
  Subject to the specific hardware characteristics of each I/O port listed in the datasheet, each
  port bit of the General Purpose IO (GPIO) Ports, can be individually configured by software
  in several modes:
  (+) Input mode 
  (+) Analog mode
  (+) Output mode
  (+) Alternate function mode
  (+) External interrupt/event lines

  [..]  
  During and just after reset, the alternate functions and external interrupt  
  lines are not active and the I/O ports are configured in input floating mode.
  
  [..]   
  All GPIO pins have weak internal pull-up and pull-down resistors, which can be 
  activated or not.

  [..]
  In Output or Alternate mode, each IO can be configured on open-drain or push-pull
  type and the IO speed can be selected depending on the VDD value.

  [..]  
  All ports have external interrupt/event capability. To use external interrupt 
  lines, the port must be configured in input mode. All available GPIO pins are 
  connected to the 16 external interrupt/event lines from EINT0 to EINT15.
  
  [..]
  The external interrupt/event controller consists of up to 23 edge detectors 
  (16 lines are connected to GPIO) for generating event/interrupt requests (each 
  input line can be independently configured to select the type (interrupt or event) 
  and the corresponding trigger event (rising or falling or both). Each line can 
  also be masked independently. 

                     ##### How to use this driver #####
  ==============================================================================  
  [..]
    (#) Enable the GPIO AHB clock using the following function: __DAL_RCM_GPIOx_CLK_ENABLE(). 

    (#) Configure the GPIO pin(s) using DAL_GPIO_Init().
        (++) Configure the IO mode using "Mode" member from GPIO_InitTypeDef structure
        (++) Activate Pull-up, Pull-down resistor using "Pull" member from GPIO_InitTypeDef 
             structure.
        (++) In case of Output or alternate function mode selection: the speed is 
             configured through "Speed" member from GPIO_InitTypeDef structure.
        (++) In alternate mode is selection, the alternate function connected to the IO
             is configured through "Alternate" member from GPIO_InitTypeDef structure.
        (++) Analog mode is required when a pin is to be used as ADC channel 
             or DAC output.
        (++) In case of external interrupt/event selection the "Mode" member from 
             GPIO_InitTypeDef structure select the type (interrupt or event) and 
             the corresponding trigger event (rising or falling or both).

    (#) In case of external interrupt/event mode selection, configure NVIC IRQ priority 
        mapped to the EINT line using DAL_NVIC_SetPriority() and enable it using
        DAL_NVIC_EnableIRQ().
         
    (#) To get the level of a pin configured in input mode use DAL_GPIO_ReadPin().
            
    (#) To set/reset the level of a pin configured in output mode use 
        DAL_GPIO_WritePin()/DAL_GPIO_TogglePin().
    
    (#) To lock pin configuration until next reset use DAL_GPIO_LockPin().

                 
    (#) During and just after reset, the alternate functions are not 
        active and the GPIO pins are configured in input floating mode (except JTAG
        pins).
  
    (#) The LSE oscillator pins OSC32_IN and OSC32_OUT can be used as general purpose 
        (PC14 and PC15, respectively) when the LSE oscillator is off. The LSE has 
        priority over the GPIO function.
  
    (#) The HSE oscillator pins OSC_IN/OSC_OUT can be used as 
        general purpose PH0 and PH1, respectively, when the HSE oscillator is off. 
        The HSE has priority over the GPIO function.
  
  @endverbatim
  *
  */ 

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup GPIO GPIO
  * @brief GPIO DAL module driver
  * @{
  */

#ifdef DAL_GPIO_MODULE_ENABLED

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
/** @defgroup GPIO_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @defgroup GPIO_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim    
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
  [..]
    This section provides functions allowing to initialize and de-initialize the GPIOs
    to be ready for use.
 
@endverbatim
  * @{
  */


/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx where x can be (A..I) to select the GPIO peripheral for APM32F40XX.
  * @param  GPIO_Init pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void DAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position;
  uint32_t ioposition = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;

  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_ALL_INSTANCE(GPIOx));
  ASSERT_PARAM(IS_GPIO_PIN(GPIO_Init->Pin));
  ASSERT_PARAM(IS_GPIO_MODE(GPIO_Init->Mode));

  /* Configure the port pins */
  for(position = 0U; position < GPIO_NUMBER; position++)
  {
    /* Get the IO position */
    ioposition = 0x01U << position;
    /* Get the current IO position */
    iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Output or Alternate function mode selection */
      if(((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT) || \
          (GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
      {
        /* Check the Speed parameter */
        ASSERT_PARAM(IS_GPIO_SPEED(GPIO_Init->Speed));
        /* Configure the IO Speed */
        temp = GPIOx->OSSEL; 
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
        temp |= (GPIO_Init->Speed << (position * 2U));
        GPIOx->OSSEL = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OMODE;
        temp &= ~(GPIO_OMODE_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
        GPIOx->OMODE = temp;
       }

      if((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG)
      {
        /* Check the parameters */
        ASSERT_PARAM(IS_GPIO_PULL(GPIO_Init->Pull));
        
        /* Activate the Pull-up or Pull down resistor for the current IO */
        temp = GPIOx->PUPD;
        temp &= ~(GPIO_PUPD_PUPDR0 << (position * 2U));
        temp |= ((GPIO_Init->Pull) << (position * 2U));
        GPIOx->PUPD = temp;
      }

      /* In case of Alternate function mode selection */
      if((GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
      {
        /* Check the Alternate function parameter */
        ASSERT_PARAM(IS_GPIO_AF(GPIO_Init->Alternate));
        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->ALF[position >> 3U];
        temp &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & 0x07U) * 4U));
        GPIOx->ALF[position >> 3U] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODE;
      temp &= ~(GPIO_MODE_MODE0 << (position * 2U));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
      GPIOx->MODE = temp;

      /*--------------------- EINT Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if((GPIO_Init->Mode & EINT_MODE) != 0x00U)
      {
        /* Enable SYSCFG Clock */
        __DAL_RCM_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EINTCFG[position >> 2U];
        temp &= ~(0x0FU << (4U * (position & 0x03U)));
        temp |= ((uint32_t)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U)));
        SYSCFG->EINTCFG[position >> 2U] = temp;

        /* Clear Rising Falling edge configuration */
        temp = EINT->RTEN;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & TRIGGER_RISING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EINT->RTEN = temp;

        temp = EINT->FTEN;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & TRIGGER_FALLING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EINT->FTEN = temp;

        temp = EINT->EMASK;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & EINT_EVT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EINT->EMASK = temp;

        /* Clear EINT line configuration */
        temp = EINT->IMASK;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & EINT_IT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EINT->IMASK = temp;
      }
    }
  }
}

/**
  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx where x can be (A..I) to select the GPIO peripheral for APM32F40XX.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
void DAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
  uint32_t position;
  uint32_t ioposition = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t tmp = 0x00U;

  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_ALL_INSTANCE(GPIOx));
  
  /* Configure the port pins */
  for(position = 0U; position < GPIO_NUMBER; position++)
  {
    /* Get the IO position */
    ioposition = 0x01U << position;
    /* Get the current IO position */
    iocurrent = (GPIO_Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*------------------------- EINT Mode Configuration --------------------*/
      tmp = SYSCFG->EINTCFG[position >> 2U];
      tmp &= (0x0FU << (4U * (position & 0x03U)));
      if(tmp == ((uint32_t)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U))))
      {
        /* Clear EINT line configuration */
        EINT->IMASK &= ~((uint32_t)iocurrent);
        EINT->EMASK &= ~((uint32_t)iocurrent);
        
        /* Clear Rising Falling edge configuration */
        EINT->FTEN &= ~((uint32_t)iocurrent);
        EINT->RTEN &= ~((uint32_t)iocurrent);

        /* Configure the External Interrupt or event for the current IO */
        tmp = 0x0FU << (4U * (position & 0x03U));
        SYSCFG->EINTCFG[position >> 2U] &= ~tmp;
      }

      /*------------------------- GPIO Mode Configuration --------------------*/
      /* Configure IO Direction in Input Floating Mode */
      GPIOx->MODE &= ~(GPIO_MODE_MODE0 << (position * 2U));

      /* Configure the default Alternate Function in current IO */
      GPIOx->ALF[position >> 3U] &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;

      /* Deactivate the Pull-up and Pull-down resistor for the current IO */
      GPIOx->PUPD &= ~(GPIO_PUPD_PUPDR0 << (position * 2U));

      /* Configure the default value IO Output Type */
      GPIOx->OMODE  &= ~(GPIO_OMODE_OT_0 << position) ;

      /* Configure the default value for IO Speed */
      GPIOx->OSSEL &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
    }
  }
}

/**
  * @}
  */

/** @defgroup GPIO_Exported_Functions_Group2 IO operation functions 
  *  @brief   GPIO Read and Write
  *
@verbatim
 ===============================================================================
                       ##### IO operation functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx where x can be (A..I) to select the GPIO peripheral for APM32F40XX.
  * @param  GPIO_Pin specifies the port bit to read.
  *         This parameter can be GPIO_PIN_x where x can be (0..15).
  * @retval The input port pin value.
  */
GPIO_PinState DAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState bitstatus;

  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_PIN(GPIO_Pin));

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
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  GPIOx where x can be (A..I) to select the GPIO peripheral for APM32F40XX.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  PinState specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */
void DAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_PIN(GPIO_Pin));
  ASSERT_PARAM(IS_GPIO_PIN_ACTION(PinState));

  if(PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSC = GPIO_Pin;
  }
  else
  {
    GPIOx->BSC = (uint32_t)GPIO_Pin << 16U;
  }
}

/**
  * @brief  Toggles the specified GPIO pins.
  * @param  GPIOx Where x can be (A..I) to select the GPIO peripheral for APM32F40XX.
  * @param  GPIO_Pin Specifies the pins to be toggled.
  * @retval None
  */
void DAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint32_t odr;

  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_PIN(GPIO_Pin));

  /* get current Output Data Register value */
  odr = GPIOx->ODATA;

  /* Set selected pins that were at low level, and reset ones that were high */
  GPIOx->BSC = ((odr & GPIO_Pin) << GPIO_NUMBER) | (~odr & GPIO_Pin);
}

/**
  * @brief  Locks GPIO Pins configuration registers.
  * @note   The locked registers are GPIOx_MODE, GPIOx_OMODE, GPIOx_OSSEL,
  *         GPIOx_PUPD, GPIOx_ALFL and GPIOx_ALFH.
  * @note   The configuration of the locked GPIO pins can no longer be modified
  *         until the next reset.
  * @param  GPIOx where x can be (A..F) to select the GPIO peripheral for APM32F4 family
  * @param  GPIO_Pin specifies the port bit to be locked.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
DAL_StatusTypeDef DAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  __IO uint32_t tmp = GPIO_LOCK_LOCKKEY;

  /* Check the parameters */
  ASSERT_PARAM(IS_GPIO_PIN(GPIO_Pin));

  /* Apply lock key write sequence */
  tmp |= GPIO_Pin;
  /* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
  GPIOx->LOCK = tmp;
  /* Reset LCKx bit(s): LCKK='0' + LCK[15-0] */
  GPIOx->LOCK = GPIO_Pin;
  /* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
  GPIOx->LOCK = tmp;
  /* Read LCKR register. This read is mandatory to complete key lock sequence */
  tmp = GPIOx->LOCK;

  /* Read again in order to confirm lock is active */
 if((GPIOx->LOCK & GPIO_LOCK_LOCKKEY) != RESET)
  {
    return DAL_OK;
  }
  else
  {
    return DAL_ERROR;
  }
}

/**
  * @brief  This function handles EINT interrupt request.
  * @param  GPIO_Pin Specifies the pins connected EINT line
  * @retval None
  */
void DAL_GPIO_EINT_IRQHandler(uint16_t GPIO_Pin)
{
  /* EINT line interrupt detected */
  if(__DAL_GPIO_EINT_GET_IT(GPIO_Pin) != RESET)
  {
    __DAL_GPIO_EINT_CLEAR_IT(GPIO_Pin);
    DAL_GPIO_EINT_Callback(GPIO_Pin);
  }
}

/**
  * @brief  EINT line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EINT line
  * @retval None
  */
__weak void DAL_GPIO_EINT_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the DAL_GPIO_EINT_Callback could be implemented in the user file
   */
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* DAL_GPIO_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

