/**
  ******************************************************************************
  * @file    system_um324xx.h
  * @author  MCD Application Team
  * @brief   
  ******************************************************************************
**/
#pragma once

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32g4xx_system
  * @{
  */

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup STM32G4xx_System_Includes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
extern uint32_t SystemCoreClock;            /*!< System Clock Frequency (Core Clock) */

extern const uint8_t  AHBPrescTable[16];    /*!< AHB prescalers table values */
extern const uint8_t  APBPrescTable[16];     /*!< APB prescalers table values */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32G4xx_System_Exported_Functions
  * @{
  */

extern void SystemInit(void);
extern void SystemClock_Config(void);
extern void SystemCoreClockUpdate(void);
extern void Error_Handler(void);



/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */

