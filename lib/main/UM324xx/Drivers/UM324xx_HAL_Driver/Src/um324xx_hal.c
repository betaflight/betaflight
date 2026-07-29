/**
  ******************************************************************************
  * @file     um324xx_hal.c
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

/** @defgroup xxx_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup HAL_Private_Variables
  * @{
  */
__IO uint32_t uwTick;
uint32_t uwTickPrio   = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  This function is used to initialize the HAL Library; it must be the first
  *         instruction to be executed in the main program (before to call any other
  *         HAL function), it performs the following:
  *           Configure the Flash prefetch, instruction and Data caches.
  *           Configures the SysTick to generate an interrupt each 1 millisecond,
  *           which is clocked by the HSI (at this stage, the clock is not yet
  *           configured and thus the system is running from the internal HSI at 16 MHz).
  *           Set NVIC Group Priority to 4.
  *           Calls the HAL_MspInit() callback function defined in user file
  *           "um324xx_hal_msp.c" to do the global low level hardware initialization
  *
  *         SysTick is used as time base for the HAL_Delay() function, the application
  *         need to ensure that the SysTick time base is always set to 1 millisecond
  *         to have correct HAL operation.
  * @param  Nones
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_Init(void)
{
    /* Configure Flash prefetch, Instruction cache, Data cache */
#if (INSTRUCTION_CACHE_ENABLE != 0U)
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
#endif /* INSTRUCTION_CACHE_ENABLE */

#ifdef UM324xF
    
#if (DATA_CACHE_ENABLE != 0U)
    __HAL_FLASH_DATA_CACHE_ENABLE();
    
    while (__HAL_FLASH_DATA_CACHE_IS_ENABLE() == 0);
#endif /* DATA_CACHE_ENABLE */

#if (PREFETCH_ENABLE != 0U)
    __HAL_FLASH_INSTRUCTION_CACHE_PREFETCH_ENABLE();
    __HAL_FLASH_DATA_CACHE_PREFETCH_ENABLE();
    
      while (__HAL_FLASH_INSTRUCTION_CACHE_PREFETCH_IS_ENABLE() == 0);
      while (__HAL_FLASH_DATA_CACHE_PREFETCH_IS_ENABLE() == 0);
#endif /* PREFETCH_ENABLE */

#endif
    
    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
    HAL_InitTick(TICK_INT_PRIORITY);

    /* Init the low level hardware */
    HAL_MspInit();
    
    /* Set ext filter enabled*/
    __HAL_RCM_EXT_FILTER_ENABLE();

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Initialize the MSP.
  * @retval None
  */
__weak void HAL_MspInit(void)
{
    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_MspInit could be implemented in the user file
     */
}

/**
  * @brief This function configures the source of the time base.
  *        The time source is configured  to have 1ms time base with a dedicated
  *        Tick interrupt priority.
  *
  *        This function is called  automatically at the beginning of program after
  *        reset by HAL_Init() or at any time when clock is reconfigured  by HAL_RCC_ClockConfig().
  *
  *        In the default implementation, SysTick timer is the source of time base.
  *        It is used to generate interrupts at regular time intervals.
  *        Care must be taken if HAL_Delay() is called from a peripheral ISR process,
  *        The SysTick interrupt must have higher priority (numerically lower)
  *        than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *        The function is declared as __weak  to be overwritten  in case of other
  *        implementation  in user file.
  * @param      TickPriority Tick interrupt priority.
  *     @arg    None
  * @return     HAL status
  *     @retval HAL status
  */
__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    /* Configure the SysTick to have interrupt in 1ms time basis*/
    if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
    {
        return HAL_ERROR;
    }

    /* Configure the SysTick IRQ priority */
    if (TickPriority < (1UL << __NVIC_PRIO_BITS))
    {
        HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
    }
    else
    {
        return HAL_ERROR;
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in SysTick ISR.
  * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
__weak void HAL_IncTick(void)
{
    uwTick += uwTickFreq;
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
__weak uint32_t HAL_GetTick(void)
{
    return uwTick;
}

/**
  * @brief This function returns a tick priority.
  * @retval tick priority
  */
uint32_t HAL_GetTickPrio(void)
{
    return uwTickPrio;
}

/**
  * @brief Set new tick Freq.
  * @param Freq  Tick Freq.
  * @retval Status
  */
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq)
{
    HAL_StatusTypeDef status  = HAL_OK;
    HAL_TickFreqTypeDef prevTickFreq;

    if (uwTickFreq != Freq)
    {
        /* Back up uwTickFreq frequency */
        prevTickFreq = uwTickFreq;

        /* Update uwTickFreq global variable used by HAL_InitTick() */
        uwTickFreq = Freq;

        /* Apply the new tick Freq  */
        status = HAL_InitTick(uwTickPrio);

        if (status != HAL_OK)
        {
            /* Restore previous tick frequency */
            uwTickFreq = prevTickFreq;
        }
    }

    return status;
}

/**
  * @brief Return tick frequency.
  * @retval tick period in Hz
  */
HAL_TickFreqTypeDef HAL_GetTickFreq(void)
{
    return uwTickFreq;
}

/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
__weak void HAL_Delay(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    /* Add a freq to guarantee minimum wait */
    if (wait < HAL_MAX_DELAY)
    {
        wait += (uint32_t)(uwTickFreq);
    }

    while((HAL_GetTick() - tickstart) < wait)
    {
    }
}

/**
  * @brief Suspends Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once HAL_SuspendTick()
  *       is called, the SysTick interrupt will be disabled and so Tick increment
  *       is suspended.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
__weak void HAL_SuspendTick(void)
{
  /* Disable SysTick Interrupt */
  CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief Resume Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once HAL_ResumeTick()
  *       is called, the SysTick interrupt will be enabled and so Tick increment
  *       is resumed.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
__weak void HAL_ResumeTick(void)
{
  /* Enable SysTick Interrupt */
  SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}


/**
  * @}
  */

/**
  * @}
  */

