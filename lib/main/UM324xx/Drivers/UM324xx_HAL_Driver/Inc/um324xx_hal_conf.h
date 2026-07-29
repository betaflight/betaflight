 /**
  ******************************************************************************
  * @file     um324xx_hal_conf.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_CONF_H__
#define __UM324XX_HAL_CONF_H__


#ifdef __cplusplus
extern "C" {
#endif


/* Exported constants --------------------------------------------------------*/
/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver 
  */
#define HAL_MODULE_ENABLED
#define HAL_RCM_MODULE_ENABLED 
#define HAL_PMU_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_OPA_MODULE_ENABLED
#define HAL_TS_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_I2C_EX_MODULE_ENABLED
#define HAL_SPI_EX_MODULE_ENABLED
#define HAL_UART_EX_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED 
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_SYSTICK_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_QSPI_MODULE_ENABLED
/* ########################## XTH/RCH Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (XTH) used in your application.
  *        This value is used by the RCM HAL module to compute the system frequency
  *        (when XTH is used as system clock source, directly or through the PLL).  
  */
#if !defined  (HSE_VALUE)  
#define HSE_VALUE         XTH_VALUE 
#endif
#define HSI_VALUE         RCH_VALUE 
#define LSE_VALUE         XTL_VALUE 
#define LSI_VALUE         RCL_VALUE 
  
#if !defined  (XTH_VALUE) 
  #define XTH_VALUE    (8000000U) /*!< Value of the External oscillator in Hz */
#endif /* XTH_VALUE */

#if !defined  (XTH_STARTUP_TIMEOUT)
  #define XTH_STARTUP_TIMEOUT    (100U)   /*!< Time out for XTH start up, in ms */
#endif /* XTH_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator (RCH) value.
  *        This value is used by the RCM HAL module to compute the system frequency
  *        (when RCH is used as system clock source, directly or through the PLL). 
  */
#if !defined  (RCH_VALUE)
  #define RCH_VALUE    (48000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* RCH_VALUE */

/**
  * @brief Internal Low Speed oscillator (RCL) value.
  */
#if !defined  (RCL_VALUE) 
 #define RCL_VALUE  (32768U)    
#endif /* RCL_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
/**
  * @brief External Low Speed oscillator (XTL) value.
  */
#if !defined  (XTL_VALUE)
 #define XTL_VALUE  (32768U)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* XTL_VALUE */

#if !defined  (XTL_STARTUP_TIMEOUT)
  #define XTL_STARTUP_TIMEOUT    (5000U)   /*!< Time out for XTL start up, in ms */
#endif /* XTL_STARTUP_TIMEOUT */

/* Tip: To avoid modifying this file each time you need to use different XTH,
   ===  you can define the XTH value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief Set the system source clock
  */ 
#define SYSCLK_USE_RCH          0		//RCH as system clock(48MHz)
#define SYSCLK_USE_XTH          1		//XTH as system clock(The frequency depends on the external crystal oscillator)
#define SYSCLK_USE_RCH_PLL      2		//RCH as the source clock of PLL0, the system clock frequency can be given in the macro definition FCLK
#define SYSCLK_USE_XTH_PLL      3		//XTH as the source clock of PLL0, the system clock frequency can be given in the macro definition FCLK

#define SYSCLK_SRC              SYSCLK_USE_XTH_PLL      //For USB applications, SYSCLK_USE_XTH_PLL is recommended (board level requires external 12Mhz crystal)

/**
  * @brief This is the HAL system configuration section
  */ 
#define  TICK_INT_PRIORITY            (0x0FU) /*!< tick interrupt priority */    
#define  PREFETCH_ENABLE              1U              
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U


/* Includes ------------------------------------------------------------------*/
#ifdef HAL_RCM_MODULE_ENABLED
  #include "um324xx_hal_rcm.h"
#endif /* HAL_RCM_MODULE_ENABLED */

#ifdef HAL_PMU_MODULE_ENABLED
  #include "um324xx_hal_pmu.h"
#endif /* HAL_PMU_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "um324xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_SYSTICK_MODULE_ENABLED
  #include "um324xx_hal_systick.h"
#endif /* HAL_SYSTICK_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "um324xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "um324xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "um324xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "um324xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
  #include "um324xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_UART_EX_MODULE_ENABLED
  #include "um324xx_hal_uart_ex.h"
#endif /* HAL_UART_EX_MODULE_ENABLED */

#ifdef HAL_LPUART_MODULE_ENABLED
  #include "um324xx_hal_lpuart.h"
#endif /* HAL_LPUART_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "um324xx_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
  #include "um324xx_hal_qspi.h"
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
  #include "um324xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_SPI_EX_MODULE_ENABLED
  #include "um324xx_hal_spi_ex.h"
#endif /* HAL_SPI_EX_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
  #include "um324xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "um324xx_hal_dcmi.h"
#endif /* HAL_DCMI_MODULE_ENABLED */

#ifdef HAL_EMC_MODULE_ENABLED
  #include "um324xx_hal_emc.h"
#endif /* HAL_EMC_MODULE_ENABLED */

#ifdef HAL_WWDT_MODULE_ENABLED
  #include "um324xx_hal_wwdt.h"
#endif /* HAL_WWDT_MODULE_ENABLED */

#ifdef HAL_IWDT_MODULE_ENABLED
  #include "um324xx_hal_iwdt.h"
#endif /* HAL_IWDT_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
  #include "um324xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
  #include "um324xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_LPTIM_MODULE_ENABLED
  #include "um324xx_hal_lptim.h"
#endif /* HAL_LPTIM_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "um324xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "um324xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_EMAC_MODULE_ENABLED
  #include "um324xx_hal_emac.h"
#endif /* HAL_EMAC_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
  #include "um324xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_SHA_MODULE_ENABLED
  #include "um324xx_hal_sha.h"
#endif /* HAL_SHA_MODULE_ENABLED */

#ifdef HAL_AES_MODULE_ENABLED
  #include "um324xx_hal_aes.h"
#endif /* HAL_AES_MODULE_ENABLED */

#ifdef HAL_CORDIC_MODULE_ENABLED
  #include "um324xx_hal_cordic.h"
#endif /* HAL_CORDIC_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "um324xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
  #include "um324xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
  #include "um324xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_VREF_MODULE_ENABLED
  #include "um324xx_hal_vref.h"
#endif /* HAL_VREF_MODULE_ENABLED */

#ifdef HAL_ACMP_MODULE_ENABLED
  #include "um324xx_hal_acmp.h"
#endif /* HAL_ACMP_MODULE_ENABLED */

#ifdef HAL_OPA_MODULE_ENABLED
  #include "um324xx_hal_opa.h"
#endif /* HAL_OPA_MODULE_ENABLED */

#ifdef HAL_TS_MODULE_ENABLED
  #include "um324xx_hal_ts.h"
#endif /* HAL_TS_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
  #include "um324xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_I2C_EX_MODULE_ENABLED
  #include "um324xx_hal_i2c_ex.h"
#endif /* HAL_I2C_EX_MODULE_ENABLED */

/** @addtogroup UM324xF_HAL_Driver
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


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup xxx_Exported_macro xxx Exported Macro
  * @{
  */ 

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup xxx_Exported_Functions
  * @{
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup xxx_Private_Macros 
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup UART_Private_Functions UART Private Functions
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
