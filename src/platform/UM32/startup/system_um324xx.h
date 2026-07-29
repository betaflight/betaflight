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

#define AHB_Div1                     ((uint16_t)7)
#define AHB_Div2                     ((uint16_t)8)
#define AHB_Div4                     ((uint16_t)9)
#define AHB_Div8                     ((uint16_t)10)
#define AHB_Div16                    ((uint16_t)11)
#define AHB_Div32                    ((uint16_t)12)
#define AHB_Div64                    ((uint16_t)13)
#define AHB_Div128                   ((uint16_t)14)
#define AHB_Div256                   ((uint16_t)15)

#define USB_SDIO_Div1                ((uint16_t)0)
#define USB_SDIO_Div2                ((uint16_t)1)
#define USB_SDIO_Div3                ((uint16_t)2)
#define USB_SDIO_Div4                ((uint16_t)3)
#define USB_SDIO_Div5                ((uint16_t)4)
#define USB_SDIO_Div6                ((uint16_t)5)
#define USB_SDIO_Div7                ((uint16_t)6)
#define USB_SDIO_Div8                ((uint16_t)7)



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
extern void SystemCoreClockUpdate(void);

extern void systemClockSetHSEValue(uint32_t frequency);
extern void OverclockRebootIfNecessary(uint32_t targetMhz);
extern int SystemSYSCLKSource(void);
extern int SystemPLLSource(void);

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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
