 /**
  ******************************************************************************
  * @file     um324xx_hal.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-11
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_H_
#define __UM324XX_HAL_H_


#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Exported_macros
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_conf.h"
//#include "common.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup xxx
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup xxx_Exported_typedefs xxx Exported Typedefs
  * @{
  */ 

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup HAL_Exported_Constants HAL Exported Constants
  * @{
  */ 
/** @defgroup HAL_TICK_FREQ Tick Frequency
  * @{
  */
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup xxx_Exported_macro xxx Exported Macro
  * @{
  */ 
/** @brief  stop exti irq
  */
#define __HAL_SYSCFG_EXTIWR_IESEL_STOP()             (SYSCFG->EXTIWR |= (SYSCFG_EXTIWR_IESEL))

/** @brief  run exti irq
  */
#define __HAL_SYSCFG_EXTIWR_IESEL_RUN()             (SYSCFG->EXTIWR &= (~SYSCFG_EXTIWR_IESEL))

/**
  * @}
  */
/* Exported variables --------------------------------------------------------*/

/** @addtogroup HAL_Exported_Variables
  * @{
  */

extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_Exported_Functions
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
  *           "um32x42x_hal_msp.c" to do the global low level hardware initialization
  *
  *         SysTick is used as time base for the HAL_Delay() function, the application
  *         need to ensure that the SysTick time base is always set to 1 millisecond
  *         to have correct HAL operation.
  * @param      None
  *     @arg    None
  * @return     HAL status
  *     @retval HAL status
  */

HAL_StatusTypeDef HAL_Init(void);

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

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

/**
  * @brief  Initialize the MSP.
  * @retval None
  */
void HAL_MspInit(void);

/**
  * @brief  DeInitializes the MSP.
  * @retval None
  */
void HAL_MspDeInit(void);

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in SysTick ISR.
 * @note This function is declared as __weak to be overwritten in case of other 
  *      implementations in user file.
  * @retval None
  */
void HAL_IncTick(void);
/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other 
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void);
/**
  * @brief This function returns a tick priority.
  * @retval tick priority
  */
uint32_t HAL_GetTickPrio(void);
/**
  * @brief Set new tick Freq.
  * @retval Status
  */
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
/**
  * @brief Return tick frequency.
  * @retval tick period in Hz
  */
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
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
void HAL_Delay(uint32_t Delay);

/* Private macros ------------------------------------------------------------*/
/** @defgroup xxx_Private_Macros 
  * @{
  */
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);   
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup xxx_Private_Functions xxx Private Functions
  * @{
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


