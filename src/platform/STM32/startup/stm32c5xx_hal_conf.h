/**
  ******************************************************************************
  * @file    stm32c5xx_hal_conf.h
  * @brief   HAL configuration file for Betaflight STM32C5 targets.
  ******************************************************************************
  */

#ifndef STM32C5xx_HAL_CONF_H
#define STM32C5xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* ########################## Module Selection ############################## */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_USART_MODULE_ENABLED

/* ########################## Oscillator Values adaptation ####################*/
#if !defined  (HSE_VALUE)
#define HSE_VALUE              25000000UL
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    100UL
#endif /* HSE_STARTUP_TIMEOUT */

#if !defined  (HSI_VALUE)
#define HSI_VALUE              144000000UL
#endif /* HSI_VALUE */

#if !defined  (LSI_VALUE)
#define LSI_VALUE               32000UL
#endif /* LSI_VALUE */

#if !defined  (LSE_VALUE)
#define LSE_VALUE              32768UL
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    5000UL
#endif /* LSE_STARTUP_TIMEOUT */

#if !defined  (EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE    12288000UL
#endif /* EXTERNAL_CLOCK_VALUE */

/* ########################### System Configuration ######################### */
#define  VDD_VALUE                  3300UL
#define  TICK_INT_PRIORITY          ((1UL<<__NVIC_PRIO_BITS) - 1UL)
#define  USE_RTOS                   0U
#define  PREFETCH_ENABLE            0U

/* ########################## Assert Selection ############################## */
/* #define USE_FULL_ASSERT    1U */

/* ################## Register callback feature configuration ############### */
#define  USE_HAL_ADC_REGISTER_CALLBACKS       0U
#define  USE_HAL_COMP_REGISTER_CALLBACKS      0U
#define  USE_HAL_CORDIC_REGISTER_CALLBACKS    0U
#define  USE_HAL_CRC_REGISTER_CALLBACKS       0U
#define  USE_HAL_CRS_REGISTER_CALLBACKS       0U
#define  USE_HAL_DAC_REGISTER_CALLBACKS       0U
#define  USE_HAL_DMA_REGISTER_CALLBACKS       0U
#define  USE_HAL_FLASH_REGISTER_CALLBACKS     0U
#define  USE_HAL_HASH_REGISTER_CALLBACKS      0U
#define  USE_HAL_I2C_REGISTER_CALLBACKS       0U
#define  USE_HAL_I3C_REGISTER_CALLBACKS       0U
#define  USE_HAL_IWDG_REGISTER_CALLBACKS      0U
#define  USE_HAL_LPTIM_REGISTER_CALLBACKS     0U
#define  USE_HAL_OPAMP_REGISTER_CALLBACKS     0U
#define  USE_HAL_PCD_REGISTER_CALLBACKS       0U
#define  USE_HAL_PKA_REGISTER_CALLBACKS       0U
#define  USE_HAL_RAMCFG_REGISTER_CALLBACKS    0U
#define  USE_HAL_RNG_REGISTER_CALLBACKS       0U
#define  USE_HAL_RTC_REGISTER_CALLBACKS       0U
#define  USE_HAL_SMARTCARD_REGISTER_CALLBACKS 0U
#define  USE_HAL_SMBUS_REGISTER_CALLBACKS     0U
#define  USE_HAL_SPI_REGISTER_CALLBACKS       0U
#define  USE_HAL_TIM_REGISTER_CALLBACKS       0U
#define  USE_HAL_UART_REGISTER_CALLBACKS      0U
#define  USE_HAL_USART_REGISTER_CALLBACKS     0U
#define  USE_HAL_WWDG_REGISTER_CALLBACKS      0U
#define  USE_HAL_XSPI_REGISTER_CALLBACKS      0U

/* ################## SPI peripheral configuration ########################## */
#define USE_SPI_CRC                   0U

/* Includes ------------------------------------------------------------------*/
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32c5xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32c5xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32c5xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32c5xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32c5xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32c5xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32c5xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32c5xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32c5xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32c5xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
#include "stm32c5xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
#include "stm32c5xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32c5xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32c5xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32c5xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
#include "stm32c5xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

/* Exported macros -----------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* STM32C5xx_HAL_CONF_H */
