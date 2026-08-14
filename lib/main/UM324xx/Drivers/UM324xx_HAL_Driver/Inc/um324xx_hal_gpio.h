 /**
  ******************************************************************************
  * @file     um324xx_hal_gpio.h
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
#ifndef __UM324XX_HAL_GPIO_H__
#define __UM324XX_HAL_GPIO_H__


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
 * @{
 */

/** @addtogroup GPIO
 * @{
 */

typedef struct
{
  uint32_t Pin;            /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */
                   
  uint32_t Mode;           /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Gpio_DbEn;      /*!< Specifies the GPIO filter switch.
                           This parameter can be a value of @ref GPIO_DB_EN_define */      
    
  uint32_t Gpio_Dbl;      /*!< Specifies the number of filtering cycles for the selected pins.
                              This parameter can be a value of a 32-bit number */    
                                    
  uint32_t Gpio_Im;       /*!< Specifies the IO input type for the selected pins.
                              This parameter can be a value of @ref GPIO_im_define */ 
    
  uint32_t Pull;           /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */
                     
  uint32_t Speed;          /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Driving_Ability; /*!< Specifies the IO output drive capability for the selected pins.
                            This parameter can be a value of @ref GPIO_ds_define*/
    
  uint32_t Alternate;      /*!< Peripheral to be connected to the selected pins. 
                           This parameter can be a value of @ref GPIO_Alternate_function_selection */
    
} GPIO_InitTypeDef;

typedef enum
{
  GPIO_PIN_RESET = 0u,
  GPIO_PIN_SET
} GPIO_PinState;

/** @defgroup GPIO_mode_define GPIO MODE define
   * @brief GPIO mode switch
   * @{
   */ 
#define  GPIO_MODE_INPUT                        MODE_INPUT
#define  GPIO_MODE_OUTPUT                       MODE_OUTPUT
#define  GPIO_MODE_AF                           MODE_AF
#define  GPIO_MODE_ANALOG                       MODE_ANALOG

/**
  * @}
  */

#define GPIO_MODE_IT_LEVEL_LOW                  (MODE_INPUT  | TRIGGER_MODE_LEVEL | TRIGGER_LEVEL_LOW) 
#define GPIO_MODE_IT_LEVEL_HIGH                 (MODE_INPUT  | TRIGGER_MODE_LEVEL | TRIGGER_LEVEL_HIGH) 

#define GPIO_MODE_IT_EDGE_FALL                  (MODE_INPUT  | TRIGGER_MODE_EDGE | TRIGGER_EDGE_SINGLE | TRIGGER_EDGE_FALLING) 
#define GPIO_MODE_IT_EDGE_RISE                  (MODE_INPUT  | TRIGGER_MODE_EDGE | TRIGGER_EDGE_SINGLE | TRIGGER_EDGE_RISING) 
#define GPIO_MODE_IT_EDGE_FALL_RISE             (MODE_INPUT  | TRIGGER_MODE_EDGE | TRIGGER_EDGE_DOUBLE )  


/** @defgroup GPIO_DB_EN_define GPIO DE EN define
   * @brief GPIO filter switch
   * @{
   */  
#define GPIO_DB_EN                      ((uint32_t)0x00000001U)
#define GPIO_DB_DISEN                   ((uint32_t)0x00000000U)
/**
  * @}
  */


/** @defgroup GPIO_dbl_define GPIO dbl define
   * @brief Number of GPIO filtering cycles
   * @{
   */ 
#define GPIO_DBL_MAK                    ((uint32_t)0xFFFFFFFFU)   
#define GPIO_DBL_INIT                   ((uint32_t)0x00000000U)   
/**
  * @}
  */


/** @defgroup GPIO_im_define GPIO im define
   * @brief GPIO CMOS  or GPIO SCHMITT_TRIGGER Activation
   * @{
   */  
#define GPIO_IM_CMOS                    ((uint32_t)0x00000000U)
#define GPIO_IM_SCHMITT_TRIGGER         ((uint32_t)0x00000001U)
/**
  * @}
  */


/** @defgroup GPIO_pull_define GPIO pull define
   * @{
   */  
#define  GPIO_NOPULL                      0x00000000U
#define  GPIO_PULLUP                      GPIO_PULL_PS_0|GPIO_PULL_PE_0
#define  GPIO_PULLDOWN                    0x00000000U|GPIO_PULL_PE_0

/**
  * @}
  */

/** @defgroup GPIO_speed_define GPIO speed define
   * @{
   */  
#define  GPIO_SPEED_FREQ_LOW              GPIO_SR_SR_0
#define  GPIO_SPEED_FREQ_HIGH             0x00000000U
/**
  * @}
  */


/** @defgroup GPIO_ds_define  GPIO Driving Strength define
  * @brief The default driving capability of the pin is 4mA.
  * @{
  */  
#define GPIO_DS_2MA       				((uint32_t)0x00000000U)
#define GPIO_DS_6MA       				((uint32_t)0x00000001U)
#define GPIO_DS_14MA       				((uint32_t)0x00000002U)
#define GPIO_DS_20MA       				((uint32_t)0x00000003U)
/**
  * @}
  */


/** @defgroup GPIO_pins_define GPIO pins define
  * @{
  */
#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */


/**
  * @}
  */


/** @defgroup GPIO_Alternate_function_selection  GPIO pins Alternate define
  * @{
  */
  
/** 
  * @brief   AF 0 selection  
  */
#if defined(UM32x42x) 
#define GPIO_AF0_LDO2               ((uint8_t)0x00)  /* LDO2 Alternate Function mapping         */
#define GPIO_AF0_ACMP               ((uint8_t)0x00)  /* ACMP2 Alternate Function mapping        */
#define GPIO_AF0_MCO                ((uint8_t)0x00)  /* MCO0 Alternate Function mapping         */
#define GPIO_AF0_SWDIO              ((uint8_t)0x00)  /* SWDIO Alternate Function mapping        */
#define GPIO_AF0_SWCLK              ((uint8_t)0x00)  /* SWCLK Alternate Function mapping        */
#define GPIO_AF0_JTDI               ((uint8_t)0x00)  /* JTDI Alternate Function mapping         */
#define GPIO_AF0_JTDO               ((uint8_t)0x00)  /* JTDO Alternate Function mapping         */
#define GPIO_AF0_NJTRST             ((uint8_t)0x00)  /* NJTRST Alternate Function mapping       */
#define GPIO_AF0_OPA                ((uint8_t)0x00)  /* OPA0 Alternate Function mapping         */
#define GPIO_AF0_CLK1HZ             ((uint8_t)0x00)  /* CLK1HZ Alternate Function mapping       */
#define GPIO_AF0_RTC                ((uint8_t)0x00)  /* RTC Alternate Function mapping          */
#define GPIO_AF0_RTCOUT             ((uint8_t)0x00)  /* RTCOUT Alternate Function mapping       */
#define GPIO_AF0_RESETN             ((uint8_t)0x00)  /* RESETN Alternate Function mapping       */
 #endif
 #if defined(UM324xF)
#define GPIO_AF0                ((uint8_t)0x00U  )
#define GPIO_AF0_MCO            (GPIO_AF0  )/* MCO (MCO1 and MCO0) Alternate Function mapping */  
#define GPIO_AF0_SWJ            (GPIO_AF0  )/* SWJ (SWD and JTAG) Alternate Function mapping */   
#define GPIO_AF0_CLK1HZ         (GPIO_AF0  )/* CLK1HZ  Alternate Function mapping */  
#define GPIO_AF0_RTC_VLD_ON     (GPIO_AF0  )/* RTC_VLD_ON Alternate Function mapping */    
#define GPIO_AF0_TRACE          (GPIO_AF0  )/* TRACE Alternate Function mapping */  
 #endif
#if defined(UM32x41x) 
#define GPIO_AF0_ACMP               ((uint8_t)0x00)  /* ACMP2 Alternate Function mapping        */
#define GPIO_AF0_MCO                ((uint8_t)0x00)  /* MCO0 Alternate Function mapping         */
#define GPIO_AF0_SWDIO              ((uint8_t)0x00)  /* SWDIO Alternate Function mapping        */
#define GPIO_AF0_SWCLK              ((uint8_t)0x00)  /* SWCLK Alternate Function mapping        */
#define GPIO_AF0_JTDI               ((uint8_t)0x00)  /* JTDI Alternate Function mapping         */
#define GPIO_AF0_JTDO               ((uint8_t)0x00)  /* JTDO Alternate Function mapping         */
#define GPIO_AF0_NJTRST             ((uint8_t)0x00)  /* NJTRST Alternate Function mapping       */
#define GPIO_AF0_OPA                ((uint8_t)0x00)  /* OPA0 Alternate Function mapping         */
#define GPIO_AF0_REFIN              ((uint8_t)0x00)  /* REFIN Alternate Function mapping       */
#endif
#if defined(UM324xH) 
#define GPIO_AF0_LDO2               ((uint8_t)0x00)  /* LDO2 Alternate Function mapping         */
#define GPIO_AF0_ACMP               ((uint8_t)0x00)  /* ACMP2 Alternate Function mapping        */
#define GPIO_AF0_MCO                ((uint8_t)0x00)  /* MCO0 Alternate Function mapping         */
#define GPIO_AF0_SWDIO              ((uint8_t)0x00)  /* SWDIO Alternate Function mapping        */
#define GPIO_AF0_SWCLK              ((uint8_t)0x00)  /* SWCLK Alternate Function mapping        */
#define GPIO_AF0_JTDI               ((uint8_t)0x00)  /* JTDI Alternate Function mapping         */
#define GPIO_AF0_JTDO               ((uint8_t)0x00)  /* JTDO Alternate Function mapping         */
#define GPIO_AF0_NJTRST             ((uint8_t)0x00)  /* NJTRST Alternate Function mapping       */
#define GPIO_AF0_OPA                ((uint8_t)0x00)  /* OPA0 Alternate Function mapping         */
#define GPIO_AF0_CLK1HZ             ((uint8_t)0x00)  /* CLK1HZ Alternate Function mapping       */
#define GPIO_AF0_RTC                ((uint8_t)0x00)  /* RTC Alternate Function mapping          */
#define GPIO_AF0_RTCOUT             ((uint8_t)0x00)  /* RTCOUT Alternate Function mapping       */
#define GPIO_AF0_RESETN             ((uint8_t)0x00)  /* RESETN Alternate Function mapping       */
 #endif
/** 
  * @brief   AF 1 selection  
  */
#if defined(UM32x42x)   
#define GPIO_AF1_TIM1               ((uint8_t)0x01)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF1_LPTIM0             ((uint8_t)0x01)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF1_TIM0               ((uint8_t)0x01)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF1_LPTIM1             ((uint8_t)0x01)  /* LPTIM1 Alternate Function mapping       */
 #endif
#if defined(UM324xF)
#define GPIO_AF1                ((uint8_t)0x01U  )
#define GPIO_AF1_TIM2           (GPIO_AF1  )/* TIM2 Alternate Function mapping */  
#define GPIO_AF1_LPUART         (GPIO_AF1  )/* LPUART Alternate Function mapping */   
#define GPIO_AF1_TIM1           (GPIO_AF1 )/* TIM1 Alternate Function mapping */    
#define GPIO_AF1_LPTIM0         (GPIO_AF1  )/* LPTIME0 Alternate Function mapping */ 
#define GPIO_AF1_LPTIM1         (GPIO_AF1  )/* LPTIME1 Alternate Function mapping */ 
  
 #endif  
 #if defined(UM32x41x) 
#define GPIO_AF1_TIM1               ((uint8_t)0x01)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF1_LPTIM0             ((uint8_t)0x01)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF1_TIM0               ((uint8_t)0x01)  /* TIM0 Alternate Function mapping         */
 
#endif  
#if defined(UM324xH)   
#define GPIO_AF1_TIM1               ((uint8_t)0x01)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF1_LPTIM0             ((uint8_t)0x01)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF1_TIM0               ((uint8_t)0x01)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF1_LPTIM1             ((uint8_t)0x01)  /* LPTIM1 Alternate Function mapping       */
 #endif
/** 
  * @brief   AF 2 selection  
  */
 #if defined(UM32x42x) 
#define GPIO_AF2_TIM4               ((uint8_t)0x02)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF2_TIM2               ((uint8_t)0x02)  /* TIM2 Alternate Function mapping         */
#define GPIO_AF2_TIM0               ((uint8_t)0x02)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF2_TIM7               ((uint8_t)0x02)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF2_TIM3               ((uint8_t)0x02)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF2_TIM9               ((uint8_t)0x02)  /* TIM9 Alternate Function mapping         */
#define GPIO_AF2_TIM1               ((uint8_t)0x02)  /* TIM1 Alternate Function mapping         */
 #endif 
#if defined(UM324xF)
#define GPIO_AF2                ((uint8_t)0x02U )
#define GPIO_AF2_TIM5           (GPIO_AF2  )/* TIM5 Alternate Function mapping */ 
#define GPIO_AF2_TIM1           (GPIO_AF2  )/* TIM1 Alternate Function mapping */ 
#define GPIO_AF2_TIM4           (GPIO_AF2  )/* TIM4 Alternate Function mapping */ 
#define GPIO_AF2_LPUART         (GPIO_AF2  )/* LPUART Alternate Function mapping */ 
#define GPIO_AF2_TIM3           (GPIO_AF2  )/* TIM3 Alternate Function mapping */ 

 #endif
 #if defined(UM32x41x) 
#define GPIO_AF2_TIM4               ((uint8_t)0x02)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF2_TIM2               ((uint8_t)0x02)  /* TIM2 Alternate Function mapping         */
#define GPIO_AF2_TIM0               ((uint8_t)0x02)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF2_TIM7               ((uint8_t)0x02)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF2_TIM3               ((uint8_t)0x02)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF2_TIM1               ((uint8_t)0x02)  /* TIM1 Alternate Function mapping         */
 
 #endif
 #if defined(UM324xH) 
#define GPIO_AF2_TIM4               ((uint8_t)0x02)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF2_TIM2               ((uint8_t)0x02)  /* TIM2 Alternate Function mapping         */
#define GPIO_AF2_TIM0               ((uint8_t)0x02)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF2_TIM7               ((uint8_t)0x02)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF2_TIM3               ((uint8_t)0x02)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF2_TIM9               ((uint8_t)0x02)  /* TIM9 Alternate Function mapping         */
#define GPIO_AF2_TIM1               ((uint8_t)0x02)  /* TIM1 Alternate Function mapping         */
 #endif  
/** 
  * @brief   AF 3 selection  
  */
 #if defined(UM32x42x)  
#define GPIO_AF3_TIM7               ((uint8_t)0x03)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF3_TIM4               ((uint8_t)0x03)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF3_TIM14              ((uint8_t)0x03)  /* TIM14 Alternate Function mapping        */
#define GPIO_AF3_TIM16              ((uint8_t)0x03)  /* TIM16 Alternate Function mapping        */
#define GPIO_AF3_TIM15              ((uint8_t)0x03)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF3_LPTIM0             ((uint8_t)0x03)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF3_TIM0               ((uint8_t)0x03)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF3_TIM9               ((uint8_t)0x03)  /* TIM9 Alternate Function mapping         */
 #endif
#if defined(UM324xF)
#define GPIO_AF3                ((uint8_t)0x03U  )
#define GPIO_AF3_TIM9           (GPIO_AF3  )/* TIM9 Alternate Function mapping */ 
#define GPIO_AF3_TIM11          (GPIO_AF3  )/* TIM11 Alternate Function mapping */ 
#define GPIO_AF3_TIM8           (GPIO_AF3  )/* TIM8 Alternate Function mapping */ 
#define GPIO_AF3_TIM10           (GPIO_AF3  )/* TIM10 Alternate Function mapping */ 

 #endif
#if defined(UM32x41x) 
#define GPIO_AF3_TIM7               ((uint8_t)0x03)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF3_TIM8               ((uint8_t)0x03)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF3_TIM4               ((uint8_t)0x03)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF3_TIM14              ((uint8_t)0x03)  /* TIM14 Alternate Function mapping        */
#define GPIO_AF3_LPTIM0             ((uint8_t)0x03)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF3_TIM0               ((uint8_t)0x03)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF3_TIM9               ((uint8_t)0x03)  /* TIM9 Alternate Function mapping         */  
 #endif
 #if defined(UM324xH)  
#define GPIO_AF3_TIM7               ((uint8_t)0x03)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF3_TIM4               ((uint8_t)0x03)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF3_TIM14              ((uint8_t)0x03)  /* TIM14 Alternate Function mapping        */
#define GPIO_AF3_TIM16              ((uint8_t)0x03)  /* TIM16 Alternate Function mapping        */
#define GPIO_AF3_TIM15              ((uint8_t)0x03)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF3_LPTIM0             ((uint8_t)0x03)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF3_TIM0               ((uint8_t)0x03)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF3_TIM9               ((uint8_t)0x03)  /* TIM9 Alternate Function mapping         */
 #endif 
/** 
  * @brief   AF 4 selection  
  */
#if defined(UM32x42x)   
#define GPIO_AF4_I2C2               ((uint8_t)0x04)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF4_I2C2               ((uint8_t)0x04)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF4_I2C1               ((uint8_t)0x04)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF4_I2C1               ((uint8_t)0x04)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF4_I2C0               ((uint8_t)0x04)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF4_I2C0               ((uint8_t)0x04)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF4_TIM9               ((uint8_t)0x04)  /* TIM9 Alternate Function mapping         */
 #endif
#if defined(UM324xF)
#define GPIO_AF4                ((uint8_t)0x04U  )
#define GPIO_AF4_I2C3           (GPIO_AF4  )/* I2C3 Alternate Function mapping */  
#define GPIO_AF4_I2C1           (GPIO_AF4  )/* I2C1 Alternate Function mapping */ 
#define GPIO_AF4_I2C2           (GPIO_AF4  )/* I2C2 Alternate Function mapping */ 

#endif
#if defined(UM32x41x) 
#define GPIO_AF4_I2C2               ((uint8_t)0x04)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF4_I2C1               ((uint8_t)0x04)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF4_I2C0               ((uint8_t)0x04)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF4_TIM9               ((uint8_t)0x04)  /* TIM9 Alternate Function mapping         */
 
#endif
#if defined(UM324xH)   
#define GPIO_AF4_I2C2               ((uint8_t)0x04)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF4_I2C2               ((uint8_t)0x04)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF4_I2C1               ((uint8_t)0x04)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF4_I2C1               ((uint8_t)0x04)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF4_I2C0               ((uint8_t)0x04)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF4_I2C0               ((uint8_t)0x04)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF4_TIM9               ((uint8_t)0x04)  /* TIM9 Alternate Function mapping         */
 #endif
/** 
  * @brief   AF 5 selection  
  */
#if defined(UM32x42x)     
#define GPIO_AF5_SPI0               ((uint8_t)0x05)  /* SPI0 Alternate Function mapping         */
#define GPIO_AF5_SPI1               ((uint8_t)0x05)  /* SPI1 Alternate Function mapping         */
#define GPIO_AF5_I2S0               ((uint8_t)0x05)  /* I2S0 Alternate Function mapping         */
#define GPIO_AF5_SPI2               ((uint8_t)0x05)  /* SPI2 Alternate Function mapping         */
#endif
#if defined(UM324xF)
#define GPIO_AF5                ((uint8_t)0x05U  )
#define GPIO_AF5_SPI1           (GPIO_AF5  )/* SPI0 Alternate Function mapping  */ 
#define GPIO_AF5_SPI2           (GPIO_AF5  )/* SPI1 Alternate Function mapping  */ 
#define GPIO_AF5_I2S0           (GPIO_AF5  )/* I2S0 Alternate Function mapping  */ 
#define GPIO_AF5_I2S1           (GPIO_AF5 )/* I2S1 Alternate Function mapping */ 

#endif
#if defined(UM32x41x)
#define GPIO_AF5_SPI2               ((uint8_t)0x05)  /* SPI2 Alternate Function mapping         */
#endif
#if defined(UM324xH)     
#define GPIO_AF5_SPI0               ((uint8_t)0x05)  /* SPI0 Alternate Function mapping         */
#define GPIO_AF5_SPI1               ((uint8_t)0x05)  /* SPI1 Alternate Function mapping         */
#define GPIO_AF5_I2S0               ((uint8_t)0x05)  /* I2S0 Alternate Function mapping         */
#define GPIO_AF5_SPI2               ((uint8_t)0x05)  /* SPI2 Alternate Function mapping         */
#endif
/** 
  * @brief   AF 6 selection  
  */
#if defined(UM32x42x)   
#define GPIO_AF6_I2S0               ((uint8_t)0x06)  /* I2S0 Alternate Function mapping         */
#define GPIO_AF6_SPI2               ((uint8_t)0x06)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF6_SPI1               ((uint8_t)0x06)  /* SPI1 Alternate Function mapping         */
#endif
#if defined(UM324xF)
#define GPIO_AF6                ((uint8_t)0x06U  )
#define GPIO_AF6_SPI4           (GPIO_AF6  )/* SPI3 Alternate Function mapping  */ 
#define GPIO_AF6_SPI3           (GPIO_AF6  )/* SPI2 Alternate Function mapping */ 
#define GPIO_AF6_I2S1           (GPIO_AF6  )/* I2S1 Alternate Function mapping */ 
#define GPIO_AF6_I2S0           (GPIO_AF6  )/* I2S0 Alternate Function mapping */ 

#endif
#if defined(UM32x41x)
#define GPIO_AF6_SPI2               ((uint8_t)0x06)  /* SPI2 Alternate Function mapping         */
#endif

#if defined(UM324xH)   
#define GPIO_AF6_I2S0               ((uint8_t)0x06)  /* I2S0 Alternate Function mapping         */
#define GPIO_AF6_SPI2               ((uint8_t)0x06)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF6_SPI1               ((uint8_t)0x06)  /* SPI1 Alternate Function mapping         */
#endif

/** 
  * @brief   AF 7 selection  
  */
#if defined(UM32x42x)    
#define GPIO_AF7_UART1              ((uint8_t)0x07)  /* UART1 Alternate Function mapping        */
#define GPIO_AF7_UART0              ((uint8_t)0x07)  /* UART0 Alternate Function mapping        */
#define GPIO_AF7_UART3              ((uint8_t)0x07)  /* UART3 Alternate Function mapping        */
#define GPIO_AF7_UART2              ((uint8_t)0x07)  /* UART2 Alternate Function mapping        */
#endif
#if defined(UM324xF)
#define GPIO_AF7                ((uint8_t)0x07U  )
#define GPIO_AF7_UART2          (GPIO_AF7  )/* UART2 Alternate Function mapping */  
#define GPIO_AF7_UART1          (GPIO_AF7  )/* UART1 Alternate Function mapping */  
#define GPIO_AF7_I2S1           (GPIO_AF7  )/* I2S1 Alternate Function mapping */  
#define GPIO_AF7_UART3          (GPIO_AF7  )/* UART3 Alternate Function mapping */  

#endif
#if defined(UM32x41x)
#define GPIO_AF7_UART1              ((uint8_t)0x07)  /* UART1 Alternate Function mapping        */
#define GPIO_AF7_UART0              ((uint8_t)0x07)  /* UART0 Alternate Function mapping        */

#endif

#if defined(UM324xH)    
#define GPIO_AF7_UART1              ((uint8_t)0x07)  /* UART1 Alternate Function mapping        */
#define GPIO_AF7_UART0              ((uint8_t)0x07)  /* UART0 Alternate Function mapping        */
#define GPIO_AF7_UART3              ((uint8_t)0x07)  /* UART3 Alternate Function mapping        */
#define GPIO_AF7_UART2              ((uint8_t)0x07)  /* UART2 Alternate Function mapping        */
#endif

/** 
  * @brief   AF 8 selection  
  */
#if defined(UM32x42x)   
#define GPIO_AF8_UART3              ((uint8_t)0x08)  /* UART3 Alternate Function mapping        */
#define GPIO_AF8_UART1              ((uint8_t)0x08)  /* UART1 Alternate Function mapping        */
#define GPIO_AF8_LPUART0            ((uint8_t)0x08)  /* LPUART0 Alternate Function mapping      */
#define GPIO_AF8_USART7             ((uint8_t)0x08)  /* USART7 Alternate Function mapping       */
#define GPIO_AF8_USART6             ((uint8_t)0x08)  /* USART6 Alternate Function mapping       */
#endif
#if defined(UM324xF)
#define GPIO_AF8                ((uint8_t)0x08U  )
#define GPIO_AF8_UART4          (GPIO_AF8  )/* UART4 Alternate Function mapping */ 
#define GPIO_AF8_I2S1           (GPIO_AF8  )/* I2S1 Alternate Function mapping */ 
#define GPIO_AF8_UART6          (GPIO_AF8  )/* UART6 Alternate Function mapping */ 
#define GPIO_AF8_UART5          (GPIO_AF8  )/* UART5 Alternate Function mapping */ 
#define GPIO_AF8_SCI            (GPIO_AF8  )/* SCI Alternate Function mapping */ 
#define GPIO_AF8_USART8         (GPIO_AF8  )/* USART8 Alternate Function mapping */ 

#endif
#if defined(UM32x41x)
#define GPIO_AF8_UART0              ((uint8_t)0x08)  /* UART0 Alternate Function mapping        */
#define GPIO_AF8_UART1              ((uint8_t)0x08)  /* UART1 Alternate Function mapping        */
#define GPIO_AF8_TIM7               ((uint8_t)0x08)  /* TIM7 Alternate Function mapping        */
#define GPIO_AF8_USART6             ((uint8_t)0x08)  /* USART6 Alternate Function mapping       */
#endif

#if defined(UM324xH)   
#define GPIO_AF8_UART3              ((uint8_t)0x08)  /* UART3 Alternate Function mapping        */
#define GPIO_AF8_UART1              ((uint8_t)0x08)  /* UART1 Alternate Function mapping        */
#define GPIO_AF8_LPUART0            ((uint8_t)0x08)  /* LPUART0 Alternate Function mapping      */
#define GPIO_AF8_USART7             ((uint8_t)0x08)  /* USART7 Alternate Function mapping       */
#define GPIO_AF8_USART6             ((uint8_t)0x08)  /* USART6 Alternate Function mapping       */
#endif

/** 
  * @brief   AF 9 selection  
  */
 #if defined(UM32x42x) 
#define GPIO_AF9_USART6             ((uint8_t)0x09)  /* USART6 Alternate Function mapping       */
#define GPIO_AF9_CANFD                ((uint8_t)0x09)  /* CANFD Alternate Function mapping          */
#define GPIO_AF9_USART7             ((uint8_t)0x09)  /* USART7 Alternate Function mapping       */
#define GPIO_AF9_TIM8               ((uint8_t)0x09)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF9_UART1              ((uint8_t)0x09)  /* UART1 Alternate Function mapping        */
#endif
#if defined(UM324xF)
#define GPIO_AF9                ((uint8_t)0x09U  )
#define GPIO_AF9_TIM14          (GPIO_AF9  )/* TIM14 Alternate Function mapping  */ 
#define GPIO_AF9_TIM13          (GPIO_AF9  )/* TIM13 Alternate Function mapping */  
#define GPIO_AF9_CAN0           (GPIO_AF9  )/* CAN0 Alternate Function mapping */ 
#define GPIO_AF9_USART8         (GPIO_AF9  )/* USART8 Alternate Function mapping */ 
#define GPIO_AF9_USART7         (GPIO_AF9  )/* USART7 Alternate Function mapping */ 
#define GPIO_AF9_CAN1           (GPIO_AF9  )/* CAN1 Alternate Function mapping */ 
#define GPIO_AF9_TIM12          (GPIO_AF9  )/* TIM12 Alternate Function mapping */ 

#endif
#if defined(UM32x41x)
#define GPIO_AF9_USART6             ((uint8_t)0x09)  /* USART6 Alternate Function mapping       */
#define GPIO_AF9_UART1              ((uint8_t)0x09)  /* UART1 Alternate Function mapping        */
#define GPIO_AF9_TIM7               ((uint8_t)0x09)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF9_TIM4               ((uint8_t)0x09)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF9_TIM8               ((uint8_t)0x09)  /* TIM8 Alternate Function mapping         */
#endif

#if defined(UM324xH) 
#define GPIO_AF9_USART6             ((uint8_t)0x09)  /* USART6 Alternate Function mapping       */
#define GPIO_AF9_CANFD                ((uint8_t)0x09)  /* CANFD Alternate Function mapping          */
#define GPIO_AF9_USART7             ((uint8_t)0x09)  /* USART7 Alternate Function mapping       */
#define GPIO_AF9_TIM8               ((uint8_t)0x09)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF9_UART1              ((uint8_t)0x09)  /* UART1 Alternate Function mapping        */
#endif

/** 
  * @brief   AF 10 selection  
  */
  #if defined(UM32x42x)  
#define GPIO_AF10_USART6            ((uint8_t)0x0A)  /* USART6 Alternate Function mapping       */
#define GPIO_AF10_LPUART1           ((uint8_t)0x0A)  /* LPUART1 Alternate Function mapping      */
#define GPIO_AF10_SPI1              ((uint8_t)0x0A)  /* SPI1 Alternate Function mapping         */
#define GPIO_AF10_LPTIM0            ((uint8_t)0x0A)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF10_TIM8              ((uint8_t)0x0A)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF10_I2C0              ((uint8_t)0x0A)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF10_USB0              ((uint8_t)0x0A)  /* USB0 Alternate Function mapping         */
#define GPIO_AF10_I2C2              ((uint8_t)0x0A)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF10_I2S               ((uint8_t)0x0A)  /* I2S Alternate Function mapping          */
#define GPIO_AF10_USART2            ((uint8_t)0x0A)  /* USART2 Alternate Function mapping       */
#define GPIO_AF10_UART1             ((uint8_t)0x0A)  /* UART1 TS Alternate Function mapping     */
#define GPIO_AF10_SPI2              ((uint8_t)0x0A)  /* SPI2 TS Alternate Function mapping      */
#define GPIO_AF10_I2C1              ((uint8_t)0x0A)  /* I2C1 TS Alternate Function mapping      */
#define GPIO_AF10_CANFD               ((uint8_t)0x0A)  /* CANFD TS Alternate Function mapping       */
#define GPIO_AF10_UART2             ((uint8_t)0x0A)  /* UART2 Alternate Function mapping        */
#endif
#if defined(UM324xF)
#define GPIO_AF10               ((uint8_t)0x0AU  )
#define GPIO_AF10_TIM12         (GPIO_AF10  )/* TIM12 Alternate Function mapping */  
#define GPIO_AF10_QSPI          (GPIO_AF10  )/* QSPI Alternate Function mapping */  

#endif
#if defined(UM32x41x)
#define GPIO_AF10_USART6            ((uint8_t)0x0A)  /* USART6 Alternate Function mapping       */
#define GPIO_AF10_LPTIM0            ((uint8_t)0x0A)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF10_TIM8              ((uint8_t)0x0A)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF10_I2C0              ((uint8_t)0x0A)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF10_USB              ((uint8_t)0x0A)  /* USB Alternate Function mapping         */
#define GPIO_AF10_I2C2              ((uint8_t)0x0A)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF10_SPI2              ((uint8_t)0x0A)  /* SPI2 TS Alternate Function mapping      */
#define GPIO_AF10_I2C1              ((uint8_t)0x0A)  /* I2C1 TS Alternate Function mapping      */
#define GPIO_AF10_TIM7              ((uint8_t)0x0A)  /* TIM7 Alternate Function mapping        */
#define GPIO_AF10_UART0             ((uint8_t)0x0A)  /* UART0 Alternate Function mapping        */
#endif

#if defined(UM324xH)  
#define GPIO_AF10_USART6            ((uint8_t)0x0A)  /* USART6 Alternate Function mapping       */
#define GPIO_AF10_LPUART1           ((uint8_t)0x0A)  /* LPUART1 Alternate Function mapping      */
#define GPIO_AF10_SPI1              ((uint8_t)0x0A)  /* SPI1 Alternate Function mapping         */
#define GPIO_AF10_LPTIM0            ((uint8_t)0x0A)  /* LPTIM0 Alternate Function mapping       */
#define GPIO_AF10_TIM8              ((uint8_t)0x0A)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF10_I2C0              ((uint8_t)0x0A)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF10_USB0              ((uint8_t)0x0A)  /* USB0 Alternate Function mapping         */
#define GPIO_AF10_I2C2              ((uint8_t)0x0A)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF10_I2S               ((uint8_t)0x0A)  /* I2S Alternate Function mapping          */
#define GPIO_AF10_USART2            ((uint8_t)0x0A)  /* USART2 Alternate Function mapping       */
#define GPIO_AF10_UART1             ((uint8_t)0x0A)  /* UART1 TS Alternate Function mapping     */
#define GPIO_AF10_SPI2              ((uint8_t)0x0A)  /* SPI2 TS Alternate Function mapping      */
#define GPIO_AF10_I2C1              ((uint8_t)0x0A)  /* I2C1 TS Alternate Function mapping      */
#define GPIO_AF10_CANFD               ((uint8_t)0x0A)  /* CANFD TS Alternate Function mapping       */
#define GPIO_AF10_UART2             ((uint8_t)0x0A)  /* UART2 Alternate Function mapping        */
#endif

/** 
  * @brief   AF 11 selection
  */
  #if defined(UM32x42x)   
#define GPIO_AF11_LPUART0           ((uint8_t)0x0B)  /* LPUART0 Alternate Function mapping      */
#define GPIO_AF11_TIM8              ((uint8_t)0x0B)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF11_CANFD               ((uint8_t)0x0B)  /* CANFD Alternate Function mapping          */
#define GPIO_AF11_TIM3              ((uint8_t)0x0B)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF11_TIM4              ((uint8_t)0x0B)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF11_LPUART1           ((uint8_t)0x0B)  /* LPUART1 Alternate Function mapping      */
#define GPIO_AF11_SPI2              ((uint8_t)0x0B)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF11_SPI1              ((uint8_t)0x0B)  /* SPI1 Alternate Function mapping         */
#define GPIO_AF11_UART3             ((uint8_t)0x0B)  /* UART3 Alternate Function mapping        */
#define GPIO_AF11_I2C0              ((uint8_t)0x0B)  /* I2C0 Alternate Function mapping         */
#endif
#if defined(UM324xF)
#define GPIO_AF11               ((uint8_t)0x0BU  )
#define GPIO_AF11_ETH           (GPIO_AF11  )/* ETH Alternate Function mapping */  
  
#endif

#if defined(UM32x41x)
#define GPIO_AF11_TIM8              ((uint8_t)0x0B)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF11_TIM3              ((uint8_t)0x0B)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF11_TIM4              ((uint8_t)0x0B)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF11_SPI2              ((uint8_t)0x0B)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF11_I2C0              ((uint8_t)0x0B)  /* I2C0 Alternate Function mapping         */
#define GPIO_AF11_TIM7              ((uint8_t)0x0B)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF11_TIM0              ((uint8_t)0x0B)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF11_UART0             ((uint8_t)0x0B)  /* UART0 Alternate Function mapping        */
#endif

#if defined(UM324xH)   
#define GPIO_AF11_LPUART0           ((uint8_t)0x0B)  /* LPUART0 Alternate Function mapping      */
#define GPIO_AF11_TIM8              ((uint8_t)0x0B)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF11_CANFD               ((uint8_t)0x0B)  /* CANFD Alternate Function mapping          */
#define GPIO_AF11_TIM3              ((uint8_t)0x0B)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF11_TIM4              ((uint8_t)0x0B)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF11_LPUART1           ((uint8_t)0x0B)  /* LPUART1 Alternate Function mapping      */
#define GPIO_AF11_SPI2              ((uint8_t)0x0B)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF11_SPI1              ((uint8_t)0x0B)  /* SPI1 Alternate Function mapping         */
#define GPIO_AF11_UART3             ((uint8_t)0x0B)  /* UART3 Alternate Function mapping        */
#define GPIO_AF11_I2C0              ((uint8_t)0x0B)  /* I2C0 Alternate Function mapping         */
#endif

/** 
  * @brief   AF 12 selection  
  */
  #if defined(UM32x42x)   
#define GPIO_AF12_TIM0              ((uint8_t)0x0C)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF12_TIM8              ((uint8_t)0x0C)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF12_CANFD               ((uint8_t)0x0C)  /* CANFD Alternate Function mapping          */
#define GPIO_AF12_TIM1              ((uint8_t)0x0C)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF12_TIM15             ((uint8_t)0x0C)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF12_TIM16             ((uint8_t)0x0C)  /* TIM16 Alternate Function mapping        */
#define GPIO_AF12_I2C1              ((uint8_t)0x0C)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF12_SPI0              ((uint8_t)0x0C)  /* SPI0 Alternate Function mapping         */
#define GPIO_AF12_I2S0              ((uint8_t)0x0C)  /* I2S0 Alternate Function mapping         */
#define GPIO_AF12_I2C2              ((uint8_t)0x0C)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF12_SPI2              ((uint8_t)0x0C)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF12_TIM7              ((uint8_t)0x0C)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF12_TIM9              ((uint8_t)0x0C)  /* TIM9 Alternate Function mapping         */
#define GPIO_AF12_TIM14             ((uint8_t)0x0C)  /* TIM14 Alternate Function mapping        */
#endif
#if defined(UM324xF)
#define GPIO_AF12               ((uint8_t)0x0CU  )
#define GPIO_AF12_SDIO          (GPIO_AF12  )/* SDIO Alternate Function mapping */ 
#define GPIO_AF12_EMC           (GPIO_AF12  )/* EMC Alternate Function mapping */ 

#endif

#if defined(UM32x41x)
#define GPIO_AF12_TIM0              ((uint8_t)0x0C)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF12_TIM8              ((uint8_t)0x0C)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF12_TIM1              ((uint8_t)0x0C)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF12_I2C1              ((uint8_t)0x0C)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF12_I2C2              ((uint8_t)0x0C)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF12_SPI2              ((uint8_t)0x0C)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF12_TIM7              ((uint8_t)0x0C)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF12_TIM9              ((uint8_t)0x0C)  /* TIM9 Alternate Function mapping         */
#endif

  #if defined(UM324xH)   
#define GPIO_AF12_TIM0              ((uint8_t)0x0C)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF12_TIM8              ((uint8_t)0x0C)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF12_CANFD               ((uint8_t)0x0C)  /* CANFD Alternate Function mapping          */
#define GPIO_AF12_TIM1              ((uint8_t)0x0C)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF12_TIM15             ((uint8_t)0x0C)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF12_TIM16             ((uint8_t)0x0C)  /* TIM16 Alternate Function mapping        */
#define GPIO_AF12_I2C1              ((uint8_t)0x0C)  /* I2C1 Alternate Function mapping         */
#define GPIO_AF12_SPI0              ((uint8_t)0x0C)  /* SPI0 Alternate Function mapping         */
#define GPIO_AF12_I2S0              ((uint8_t)0x0C)  /* I2S0 Alternate Function mapping         */
#define GPIO_AF12_I2C2              ((uint8_t)0x0C)  /* I2C2 Alternate Function mapping         */
#define GPIO_AF12_SPI2              ((uint8_t)0x0C)  /* SPI2 Alternate Function mapping         */
#define GPIO_AF12_TIM7              ((uint8_t)0x0C)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF12_TIM9              ((uint8_t)0x0C)  /* TIM9 Alternate Function mapping         */
#define GPIO_AF12_TIM14             ((uint8_t)0x0C)  /* TIM14 Alternate Function mapping        */
#endif

/** 
  * @brief   AF 13 selection  
  */
#if defined(UM32x42x)    
#define GPIO_AF13_LPUART1           ((uint8_t)0x0D)  /* LPUART1 Alternate Function mapping      */
#define GPIO_AF13_TIM4              ((uint8_t)0x0D)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF13_TIM15             ((uint8_t)0x0D)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF13_TIM7              ((uint8_t)0x0D)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF13_LPUART0           ((uint8_t)0x0D)  /* LPUART0 Alternate Function mapping      */
#define GPIO_AF13_TIM8              ((uint8_t)0x0D)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF13_LPTIM1            ((uint8_t)0x0D)  /* LPTIM1 Alternate Function mapping       */
#define GPIO_AF13_TIM14             ((uint8_t)0x0D)  /* TIM14 Alternate Function mapping        */
#define GPIO_AF13_TIM0              ((uint8_t)0x0D)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF13_TIM3              ((uint8_t)0x0D)  /* TIM3 Alternate Function mapping         */
#endif
#if defined(UM324xF)
#define GPIO_AF13                ((uint8_t)0x0DU  )
#define GPIO_AF13_DCMI           (GPIO_AF13  )/* DCMI Alternate Function mapping */ 

#endif
#if defined(UM32x41x)
#define GPIO_AF13_TIM4              ((uint8_t)0x0D)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF13_TIM7              ((uint8_t)0x0D)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF13_TIM0              ((uint8_t)0x0D)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF13_TIM3              ((uint8_t)0x0D)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF13_TIM9              ((uint8_t)0x0D)  /* TIM9 Alternate Function mapping         */
#define GPIO_AF13_TIM8              ((uint8_t)0x0D)  /* TIM8 Alternate Function mapping         */
#endif

#if defined(UM324xH)    
#define GPIO_AF13_LPUART1           ((uint8_t)0x0D)  /* LPUART1 Alternate Function mapping      */
#define GPIO_AF13_TIM4              ((uint8_t)0x0D)  /* TIM4 Alternate Function mapping         */
#define GPIO_AF13_TIM15             ((uint8_t)0x0D)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF13_TIM7              ((uint8_t)0x0D)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF13_LPUART0           ((uint8_t)0x0D)  /* LPUART0 Alternate Function mapping      */
#define GPIO_AF13_TIM8              ((uint8_t)0x0D)  /* TIM8 Alternate Function mapping         */
#define GPIO_AF13_LPTIM1            ((uint8_t)0x0D)  /* LPTIM1 Alternate Function mapping       */
#define GPIO_AF13_TIM14             ((uint8_t)0x0D)  /* TIM14 Alternate Function mapping        */
#define GPIO_AF13_TIM0              ((uint8_t)0x0D)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF13_TIM3              ((uint8_t)0x0D)  /* TIM3 Alternate Function mapping         */
#endif

/** 
  * @brief   AF 14 selection  
  */
#if defined(UM32x42x)  
#define GPIO_AF14_TIM1              ((uint8_t)0x0E)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF14_TIM15             ((uint8_t)0x0E)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF14_TIM7              ((uint8_t)0x0E)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF14_TIM8              ((uint8_t)0x0E)  /* TIM Alternate Function mapping          */
#define GPIO_AF14_TIM0              ((uint8_t)0x0E)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF14_TIM14             ((uint8_t)0x0E)  /* TIM14 Alternate Function mapping        */
#define GPIO_AF14_TIM3              ((uint8_t)0x0E)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF14_TIM9              ((uint8_t)0x0E)  /* TIM9 Alternate Function mapping         */
#endif
#if defined(UM324xF)
#define GPIO_AF14                ((uint8_t)0x0EU  )
#endif
#if defined(UM32x41x)
#define GPIO_AF14_TIM1             ((uint8_t)0x0E)  /* TIM1 Alternate Function mapping        */
#define GPIO_AF14_TIM9             ((uint8_t)0x0E)  /* TIM9 Alternate Function mapping        */
#define GPIO_AF14_TIM7             ((uint8_t)0x0E)  /* TIM7 Alternate Function mapping        */
#define GPIO_AF14_TIM8             ((uint8_t)0x0E)  /* TIM8 Alternate Function mapping        */
#define GPIO_AF14_TIM3             ((uint8_t)0x0E)  /* TIM3 Alternate Function mapping        */
#define GPIO_AF14_TIM0             ((uint8_t)0x0E)  /* TIM0 Alternate Function mapping        */
#endif

#if defined(UM324xH)  
#define GPIO_AF14_TIM1              ((uint8_t)0x0E)  /* TIM1 Alternate Function mapping         */
#define GPIO_AF14_TIM15             ((uint8_t)0x0E)  /* TIM15 Alternate Function mapping        */
#define GPIO_AF14_TIM7              ((uint8_t)0x0E)  /* TIM7 Alternate Function mapping         */
#define GPIO_AF14_TIM8              ((uint8_t)0x0E)  /* TIM Alternate Function mapping          */
#define GPIO_AF14_TIM0              ((uint8_t)0x0E)  /* TIM0 Alternate Function mapping         */
#define GPIO_AF14_TIM14             ((uint8_t)0x0E)  /* TIM14 Alternate Function mapping        */
#define GPIO_AF14_TIM3              ((uint8_t)0x0E)  /* TIM3 Alternate Function mapping         */
#define GPIO_AF14_TIM9              ((uint8_t)0x0E)  /* TIM9 Alternate Function mapping         */
#endif
/** 
  * @brief   AF 15 selection  
  */
#if defined(UM32x42x)   
#define GPIO_AF15_EVENTOUT          ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping     */
#endif
#if defined(UM324xF)
#define GPIO_AF15                ((uint8_t)0x0FU  )
#define GPIO_AF15_EVENTOUT       (GPIO_AF15  )/* EVENTOUT Alternate Function mapping   */ 
#endif
#if defined(UM32x41x)
#define GPIO_AF15_EVENTOUT          ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping     */
#endif
#if defined(UM324xH)   
#define GPIO_AF15_EVENTOUT          ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping     */
#endif
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup GPIO_Private_Constants GPIO Private Constants
  * @{
  */
#define GPIO_MODE_Pos                           0U
#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)

#define EXTI_MODE_Pos                           (4U)
#define EXTI_MODE                               (0x3UL << EXTI_MODE_Pos)
#define TRIGGER_MODE_EDGE                       (0x1UL << EXTI_MODE_Pos)
#define TRIGGER_MODE_LEVEL                      (0x2UL << EXTI_MODE_Pos)

#define EXTI_TRIGGER_Pos                        (6U)
#define EXTI_TRIGGER_MODE                       (0x3UL << EXTI_TRIGGER_Pos)
#define TRIGGER_LEVEL_HIGH                      (0x0UL << EXTI_TRIGGER_Pos)
#define TRIGGER_LEVEL_LOW                       (0x1UL << EXTI_TRIGGER_Pos)
#define TRIGGER_EDGE_RISING                     (0x2UL << EXTI_TRIGGER_Pos)
#define TRIGGER_EDGE_FALLING                    (0x3UL << EXTI_TRIGGER_Pos)

#define EXTI_EGDE_Pos                           (10U)
#define EXTI_EGDE_MODE                          (0x1UL << EXTI_EGDE_Pos)
#define TRIGGER_EDGE_SINGLE                     (0x0UL << EXTI_EGDE_Pos)
#define TRIGGER_EDGE_DOUBLE                     (0x1UL << EXTI_EGDE_Pos)

#define EXTI_MODE_Mask                          (0x7FUL << EXTI_MODE_Pos)

/**
  * @}
  */


/** @defgroup GPIOx_Get_Port_Index GPIO Get Port Index
  * @{
  */
#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U : 3U)
/**
 * @}
 */

/**
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  GPIO_PORT  This parameter can be GPIOx where x can be(A..D)
  * @param  __EXTI_LINE__ specifies the EXTI line flag to check.
  *         This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
#define __HAL_GPIO_EXTI_GET_FLAG(GPIO_PORT,__EXTI_LINE__) (GPIO_PORT->RIS & (__EXTI_LINE__))

/**
  * @brief  Clears the EXTI's line pending flags.
  * @param  GPIO_PORT  This parameter can be GPIOx where x can be(A..D)    
  * @param  __EXTI_LINE__ specifies the EXTI lines flags to clear.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
#define __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PORT,__EXTI_LINE__) (GPIO_PORT->IC = (__EXTI_LINE__))

/**
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  GPIO_PORT  This parameter can be GPIOx where x can be(A..D)
  * @param  __EXTI_LINE__ specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
#define __HAL_GPIO_EXTI_GET_IT(GPIO_PORT,__EXTI_LINE__) ((GPIO_PORT->RIS & (__EXTI_LINE__)) && (GPIO_PORT->MIS & (__EXTI_LINE__)))

/**
  * @brief  Clears the EXTI's line pending bits.
  * @param  GPIO_PORT  This parameter can be GPIOx where x can be(A..D)
  * @param  __EXTI_LINE__ specifies the EXTI lines to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */                                     
#define __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PORT,__EXTI_LINE__) (GPIO_PORT->IC = __EXTI_LINE__)


/**
  * @brief  Lock gpio config.
  * @param  GPIO_PORT  This parameter can be GPIOx where x can be(A..D)    
  * @param  __GPIO_PIN__ GPIO_PIN num
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @note   Set to 1 to lock the configuration of the corresponding IO until the next reset
  * @retval None
  */
#define __HAL_GPIO_LOCK_IO(GPIO_PORT,__GPIO_PIN__)     SET_BIT(GPIO_PORT->LOCK ,__GPIO_PIN__)


/* Exported functions --------------------------------------------------------*/
/** @addtogroup GPIO_Exported_Functions
  * @{
  */


/** @addtogroup GPIO_Exported_Functions_Group1
  * @{
  */

/* Initialization and de-initialization functions *****************************/
/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device
  * @param  GPIO_Init pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);

/**
  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx where x can be (A..D) to select the GPIO peripheral for UM32X42X device
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


/**
 * @brief  Fills the GPIO_InitStruct member with its default value.
 * @param  GPIO_InitStruct pointer to a GPIO_InitType structure which will
 *         be initialized.
 * @retval None
 */
void GPIO_InitStructFunc(GPIO_InitTypeDef* GPIO_InitStruct);

/**
  * @}
  */


/** @addtogroup GPIO_Exported_Functions_Group2
  * @{
  */

/* IO operation functions *****************************************************/
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

/**
  * @}
  */


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __UM32x42x_HAL_GPIO_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


