/**
  ******************************************************************************
  * @file    um324xx_hal_iwdt.c
  * @author  MCU Team
  * @version V1.00
  * @date    2023-4-21
  * @brief   IWDT HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Watchdog Timer (IWDT) peripheral:
  *           + Initialization and Configuration functions
  *           + IO operation functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 - 2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                      ##### IWDT Specific features #####
  ==============================================================================
  [..]
    (+) The IWDT can be started by either software.

    (+) The IWDT is clocked by the RCL or XTL and thus stays active even if the 
        main clock fails.

    (+) Once the IWDT is started, the LSCLK is forced ON and both cannot be
        disabled. The counter starts counting down from the load value (default is 
        0x0000 3FFF).When it reaches the end of count value (0x0000 0000) a reset signal is
        generated (IWDT reset).

    (+) Whenever the load value is written in the IWDT_LOAD register,
        the IWDT_LOAD value is reloaded into the counter and the watchdog reset
        is prevented.

    (+) IWDTRSTF flag in RCM_RFR register can be used to inform when an IWDT
        reset occurs.

    (+) The IWDT is implemented in the VDD voltage domain that is still functional
        in Deepsleep mode (IWDT Interrupt can wake up the CPU from Deepsleep).
        IWDT interrupt flag in IWDT_INTRAW register can be used to inform when an IWDT
        interrupt occurs whether the interrupt is enabled or not.

    (+) Debug mode: When the microcontroller enters debug mode (core halted),
        the IWDT counter either continues to work normally or stops, depending
        on STALL configuration bit in the IWDT_STALL register, accessible through
        __HAL_IWDT_ENABLE_STALL() macros.

    [..] Min-max timeout value @32KHz (LSCLK): ~122us / ~36 hours
         The IWDT timeout may vary due to LSCLK dispersion.
         The LSCLK measured value can be used to have an IWDT timeout with an
         acceptable accuracy.

    [..] Default timeout value (necessary for WRC status flags in the IWDT_CTRL 
         register update):
         Constant LSCLK_VALUE is defined based on the nominal LSCLK clock frequency.
         This frequency being subject to variations as mentioned above, the
         default timeout value (defined through constant HAL_IWDT_DEFAULT_TIMEOUT
         below) may become too short or too long.
         In such cases, this default timeout value can be tuned by redefining
         the constant LSCLK_VALUE at user-application level (based, for instance,
         on the measured LSCLK clock frequency as explained above).

                     ##### How to use this driver #####
  ==============================================================================
  [..]
    (#) Use IWDT using HAL_IWDT_Init() function to :
      (++) Enable instance by writing IWDT_LOAD register, accessible through 
           __HAL_IWDT_START macros. LSCLK is forced ON and IWDT counter starts 
           counting down.
           
      (++) Enable write access by writting value 0x1ACCE551 in the IWDT_LOCK register
           to configuration registers except itself.
           
      (++) Configure the CLK_DIV in the IWDT_STALL register and LOAD value in the 
           IWDT_LOAD register. This load value will be loaded in the IWDT_CNT 
           each time the watchdog is reloaded or recleared, then the IWDT will 
           start counting down from this value.
           
      (++) Wait for WRC status flag in the IWDT_CTRL register to be reset.
      
      (++) Configure the RSTEN or INTEN in the IWDT_CTRL regidter to enable MCU reset 
           or IWDT interrupt to meet the user requirement.

    (#) Then the application program must refresh the IWDT counter at regular
        intervals during normal operation to prevent an MCU reset, using
        HAL_IWDT_Refresh() function or __HAL_IWDT_CLEAR_FLAG_IT macros under 
        interrupt mode.

     *** IWDT HAL driver macros list ***
     ====================================
     [..]
       Below the list of most used macros in IWDT HAL driver:
      (+) __HAL_IWDT_START: the IWDT start to work
      (+) __HAL_IWDT_CLEAR_FLAG_IT: clear the IWDT overflow status
  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

#ifdef HAL_IWDT_MODULE_ENABLED
/** @defgroup IWDT IWDT
  * @brief IWDT HAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup IWDT_Private_Constants IWDT Private Constants
  * @{
  */

/* WRC Status flag needs up to 3~4 LSCLK clock periods which undivided by the CLK_DIV
   to be updated. The number of LSCLK clock periods is upper-rounded to
   4 for the timeout value calculation.
   The timeout value is calculated using the highest prescaler (65536) and
   the LSCLK_VALUE(32000UL) constant. The value of this constant can be changed by the user
   to take into account possible LSCLK clock period variations.
   The timeout value is multiplied by 1000 to be converted in milliseconds.
   LSCLK startup time is also considered here by adding LSCLK_STARTUP_TIME
   converted in milliseconds.
   (((4UL * 65536UL * 1000UL) / LSCLK_VALUE) + ((LSI_STARTUP_TIME / 1000UL) + 1UL)) */
   
#define HAL_IWDT_DEFAULT_TIMEOUT        (10000)
#define IWDT_KERNEL_UPDATE_FLAG         IWDT_CTRL_WRC

/**
  * @brief  IWDT Lock Register Value
  */
#define IWDT_LOCK_WRITE_ACCESS_ENABLE    0x1ACCE551u  /*!< IWDT Lock Write Access Enable  */
#define IWDT_LOCK_WRITE_ACCESS_DISABLE   0x00000000u  /*!< IWDT Lock Write Access Disable */

/**
  * @brief  IWDT Prescaler Max Value 
  */
#define IWDT_CLKDIV_MAX                  0xFFFFu                    /*!< IWDT Clock Divide Max Value  */

/**
  * @brief  IWDT Clear Value 
  */
#define IWDT_CLR_CARRY                   0xCCCCCCCCu                /*!< Clear Overflow, Reset and Interrupt flag */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup IWDT_Private_Macros IWDT Private Macros
  * @{
  */

/**
  * @brief  Enable write access to IWDT registers(exclude WDT_LOCK register).
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define IWDT_ENABLE_WRITE_ACCESS(__HANDLE__)        WRITE_REG((__HANDLE__)->Instance->LOCK, IWDT_LOCK_WRITE_ACCESS_ENABLE)

/**
  * @brief  Disable write access to IWDT registers(exclude WDT_LOCK register).
  * @param  __HANDLE__  IWDT handle
  * @retval None
  */
#define IWDT_DISABLE_WRITE_ACCESS(__HANDLE__)       WRITE_REG((__HANDLE__)->Instance->LOCK, IWDT_LOCK_WRITE_ACCESS_DISABLE)
  

#define IS_IWDT_RST_TIMES_MODE(__MODE__)         (((__MODE__) == IWDT_RST_2TIMES)     || \
                                                  ((__MODE__) == IWDT_RST_1TIMES))
                                                  
#define IS_IWDT_RST_MODE(__MODE__)               (((__MODE__) == IWDT_RST_ENABLE)        || \
                                                  ((__MODE__) == IWDT_RST_DISABLE))

#define IS_IWDT_IE_MODE(__MODE__)                (((__MODE__) == IWDT_IE_ENABLE)         || \
                                                  ((__MODE__) == IWDT_IE_DISABLE))

#define IS_IWDT_STALL_MODE(__MODE__)             (((__MODE__) == IWDT_STALL_ENABLE)      || \
                                                  ((__MODE__) == IWDT_STALL_DISABLE))

/**
  * @brief  Check IWDT reload value.
  * @param  __LOAD__  IWDT reload value
  * @retval None
  */
#define IS_IWDT_LOAD(__LOAD__)                ((__LOAD__) <= IWDT_LOAD_LOAD)

/**
  * @brief  Check IWDT clock divide value.
  * @param  __CLKDIV__  IWDT clock divide value
  * @retval None
  */
#define IS_IWDT_CLKDIV(__CLKDIV__)            ((__CLKDIV__) <= IWDT_CLKDIV_MAX)


/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup IWDT_Exported_Functions IWDT Exported Functions
  * @{
  */

/** @defgroup IWDT_Exported_Functions_Group1 Initialization and Configuration functions
  *  @brief    Initialization and Configuration functions.
  *
@verbatim
  ==============================================================================
          ##### Initialization and Configuration functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Initialize and start the IWDT according to the specified parameters
          in the IWDT_InitTypeDef of associated handle.
      (+) Initialize the IWDT MSP.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the IWDT according to the specified.
  *         parameters in the IWDT_InitTypeDef of  associated handle.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_IWDT_Init(IWDT_HandleTypeDef *hiwdt)
{
	uint32_t tickstart;

	/* Check the IWDT handle allocation */
	if (hiwdt == NULL)
	{
	return HAL_ERROR;
	}

	

#if (USE_HAL_IWDT_REGISTER_CALLBACKS == 1)
	/* Reset Callback pointers */
	if (hiwdt->IntCallback == NULL)
	{
		hiwdt->IntCallback = HAL_IWDT_IntCallback;
	}

	if (hiwdt->MspInitCallback == NULL)
	{
		hiwdt->MspInitCallback = HAL_IWDT_MspInit;
	}

    /* Init the low level hardware */
	hiwdt->MspInitCallback(hiwdt);
#else
	/* Init the low level hardware */
	HAL_IWDT_MspInit(hiwdt);
#endif /* USE_HAL_IWDT_REGISTER_CALLBACKS */

    IWDT_ENABLE_WRITE_ACCESS(hiwdt);
  
    /* Set IWDT Ctrl register */
    WRITE_REG(hiwdt->Instance->CTRL, hiwdt->Init.RstTimes | hiwdt->Init.Rsten \
                                   | hiwdt->Init.Inten);

    /* Check wrc flag, if previous update not done, return timeout */
    tickstart = HAL_GetTick();

    /* Wait for wrc flag to be updated */
    while ((hiwdt->Instance->CTRL & IWDT_KERNEL_UPDATE_FLAG) != IWDT_CTRL_WRC)
    {
        if ((HAL_GetTick() - tickstart) > HAL_IWDT_DEFAULT_TIMEOUT)
        {
            if ((hiwdt->Instance->CTRL & IWDT_KERNEL_UPDATE_FLAG) != IWDT_CTRL_WRC)
            {
                return HAL_TIMEOUT;
            }
        }
    }
  
    /* Set IWDT Stall register */
    WRITE_REG(hiwdt->Instance->STALL,  hiwdt->Init.Stallen | (hiwdt->Init.Clkdiv << 16)); 

    IWDT_DISABLE_WRITE_ACCESS(hiwdt);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Start the IWDT to work.
  *         parameters in the IWDT_InitTypeDef of  associated handle.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_IWDT_Start(IWDT_HandleTypeDef *hiwdt)
{
	uint32_t tickstart;

	IWDT_ENABLE_WRITE_ACCESS(hiwdt);

	/* Set IWDT Load register*/
	__HAL_IWDT_START(hiwdt, hiwdt->Init.Load);

	/* Check wrc flag, if previous update not done, return timeout */
	tickstart = HAL_GetTick();

	/* Wait for wrc flag to be updated */
	while ((hiwdt->Instance->CTRL & IWDT_KERNEL_UPDATE_FLAG) != IWDT_CTRL_WRC)
	{
		if ((HAL_GetTick() - tickstart) > HAL_IWDT_DEFAULT_TIMEOUT)
		{
			if ((hiwdt->Instance->CTRL & IWDT_KERNEL_UPDATE_FLAG) != IWDT_CTRL_WRC)
			{
				return HAL_TIMEOUT;
			}
		}
	}

	IWDT_DISABLE_WRITE_ACCESS(hiwdt);

	/* Return function status */
	return HAL_OK;
}

/**
  * @brief  Initialize the IWDT MSP.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @note   When rewriting this function in user file, mechanism may be added
  *         to avoid multiple initialize when HAL_IWDT_Init function is called
  *         again to change parameters.
  * @retval None
  */
__weak void HAL_IWDT_MspInit(IWDT_HandleTypeDef *hiwdt)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hiwdt);

	/* NOTE: This function should not be modified, when the callback is needed,
		   the HAL_IWDT_MspInit could be implemented in the user file
	*/
}


#if (USE_HAL_IWDT_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User IWDT Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param  hiwdt IWDT handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_IWDT_INT_CB_ID Overflow Interrupt Callback ID
  *           @arg @ref HAL_IWDT_MSPINIT_CB_ID MspInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval status
  */
HAL_StatusTypeDef HAL_IWDT_RegisterCallback(IWDT_HandleTypeDef *hiwdt, HAL_IWDT_CallbackIDTypeDef CallbackID,
                                            pIWDT_CallbackTypeDef pCallback)
{
	HAL_StatusTypeDef status = HAL_OK;

	if (pCallback == NULL)
	{
		status = HAL_ERROR;
	}
	else
	{
		switch (CallbackID)
		{
			case HAL_IWDT_INT_CB_ID:
		      hiwdt->IntCallback = pCallback;
		      break;

	        case HAL_IWDT_MSPINIT_CB_ID:
		      hiwdt->MspInitCallback = pCallback;
		      break;

	        default:
			  status = HAL_ERROR;
		      break;
		}
	}
	return status;
}


/**
  * @brief  Unregister a IWDT Callback
  *         IWDT Callback is redirected to the weak (surcharged) predefined callback
  * @param  hiwdt IWDT handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_IWDT_INT_CB_ID Overflow Interrupt Callback ID
  *           @arg @ref HAL_IWDT_MSPINIT_CB_ID MspInit callback ID
  * @retval status
  */
HAL_StatusTypeDef HAL_IWDT_UnRegisterCallback(IWDT_HandleTypeDef *hiwdt, HAL_IWDT_CallbackIDTypeDef CallbackID)
{
	HAL_StatusTypeDef status = HAL_OK;

	switch (CallbackID)
	{
		case HAL_IWDT_INT_CB_ID:
		  hiwdt->IntCallback = HAL_IWDT_IntCallback;
		  break;

		case HAL_IWDT_MSPINIT_CB_ID:
		  hiwdt->MspInitCallback = HAL_IWDT_MspInit;
		  break;

		default:
		  status = HAL_ERROR;
		  break;
	}

	return status;
}
#endif /* USE_HAL_IWDT_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup IWDT_Exported_Functions_Group2 IO operation functions
  *  @brief    IO operation functions
  *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Refresh the IWDT.
    (+) Handle IWDT interrupt request and associated function callback.

@endverbatim
  * @{
  */

/**
  * @brief  Refresh the IWDT.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_IWDT_Refresh(IWDT_HandleTypeDef *hiwdt)
{
	IWDT_ENABLE_WRITE_ACCESS(hiwdt);

	/* Reload the value to IWDT_CNT register */
	__HAL_IWDT_RELOAD(hiwdt);

	IWDT_DISABLE_WRITE_ACCESS(hiwdt);

	/* Return function status */
	return HAL_OK;
}


/**
  * @brief  Handle IWDT interrupt request.
  * @note   The Overflow Interrupt can be used if specific safety operations
  *         or data logging must be performed before the reset is generated.
  *         The Overflow interrupt is enabled by calling HAL_IWDT_Init function with
  *         IE_Mode set to IWDT_IE_ENABLE.
  *         When the downcounter reaches the value 0x0000 0000, and Overflow interrupt is
  *         generated and the corresponding Interrupt Service Routine (ISR) can
  *         be used to trigger specific actions (such as communications or data
  *         logging or clear the overflow state,interrupt flag and reset flag), 
  *         before resetting the device.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @retval None
  */
void HAL_IWDT_IRQHandler(IWDT_HandleTypeDef *hiwdt)
{
	/* Check if IWDT Interrupt is enable */
	if (__HAL_IWDT_GET_IT_SOURCE(hiwdt, IWDT_IT_IE) != RESET)
	{
	/* Check if IWDT Interrupt occurred */
	if (__HAL_IWDT_GET_FLAG_IT(hiwdt, IWDT_FLAG_IF) != RESET)
	{
		IWDT_ENABLE_WRITE_ACCESS(hiwdt);

		/* Clear the IWDT Interrupt flag */
	    __HAL_IWDT_CLEAR_FLAG_IT(hiwdt);

		IWDT_DISABLE_WRITE_ACCESS(hiwdt);

#if (USE_HAL_IWDT_REGISTER_CALLBACKS == 1)
		/* Interrupt registered callback */
		hiwdt->IntCallback(hiwdt);
#else
		/* Interrupt callback */
		HAL_IWDT_IntCallback(hiwdt);
#endif /* USE_HAL_IWDT_REGISTER_CALLBACKS */
    }
  }
}


/**
  * @brief  IWDT  interrupt callback.
  * @param  hiwdt  pointer to a IWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified IWDT module.
  * @retval None
  */
__weak void HAL_IWDT_IntCallback(IWDT_HandleTypeDef *hiwdt)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hiwdt);

	/* NOTE: This function should not be modified, when the callback is needed,
		   the HAL_IWDT_IntCallback could be implemented in the user file
	*/
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_IWDT_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
