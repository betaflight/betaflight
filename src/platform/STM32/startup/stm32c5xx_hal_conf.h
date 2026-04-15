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
/* HAL2 uses USE_HAL_xxx_MODULE=1 (checked by stm32_hal.h) */
#define HAL_MODULE_ENABLED
#define USE_HAL_ADC_MODULE       1
#define USE_HAL_CORTEX_MODULE    1
#define USE_HAL_DMA_MODULE       1
#define USE_HAL_EXTI_MODULE      1
#define USE_HAL_FLASH_MODULE     1
#define USE_HAL_GPIO_MODULE      1
#define USE_HAL_I2C_MODULE       1
#define USE_HAL_PCD_MODULE       1
#define USE_HAL_PWR_MODULE       1
#define USE_HAL_RCC_MODULE       1
#define USE_HAL_RNG_MODULE       1
#define USE_HAL_RTC_MODULE       1
#define USE_HAL_SPI_MODULE       1
#define USE_HAL_TIM_MODULE       1
#define USE_HAL_UART_MODULE      1
#define USE_HAL_TAMP_MODULE      1
#define USE_HAL_USART_MODULE     1
/* Old-style defines for Betaflight platform code that checks these */
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
#define HAL_TAMP_MODULE_ENABLED
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

/* ################## PCD peripheral configuration ########################## */
#define USE_HAL_PCD_USER_DATA         1U

/* ################## FLASH peripheral configuration ######################## */
#define USE_HAL_FLASH_PROGRAM_BY_ADDR 1U
#define USE_HAL_FLASH_ERASE_PAGE      1U

/* ################## SPI peripheral configuration ########################## */
#define USE_SPI_CRC                   0U

/* NOTE: HAL2 (Cube 2.0) conf header must NOT include individual HAL module
 * headers here -- that causes a circular dependency because hal_def.h includes
 * this file before defining hal_status_t.  Module headers are included later
 * by the HAL driver .c files themselves.
 */

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
