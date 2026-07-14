/*
 * HAL configuration for the Betaflight N6 OpenBootloader. Mirrors the FSBL
 * stub's trimmed config and adds IWDG (boot watchdog armed before BF jump)
 * and PCD (USB device for the DFU loop).
 */

#ifndef STM32N6xx_HAL_CONF_H
#define STM32N6xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_XSPI_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED

#if !defined(HSE_VALUE)
#define HSE_VALUE              48000000UL
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    100UL
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE              32768UL
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    5000UL
#endif
#if !defined(MSI_VALUE)
#define MSI_VALUE              4000000UL
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE              64000000UL
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE              32000UL
#endif

#define VDD_VALUE             3300UL
#define TICK_INT_PRIORITY     15U
#define USE_RTOS              0U

#define USE_HAL_XSPI_REGISTER_CALLBACKS  0U
#define USE_HAL_PWR_REGISTER_CALLBACKS   0U
#define USE_SPI_CRC                      0U

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32n6xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32n6xx_hal_gpio.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32n6xx_hal_cortex.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32n6xx_hal_pwr.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32n6xx_hal_dma.h"
#endif
#ifdef HAL_XSPI_MODULE_ENABLED
#include "stm32n6xx_hal_xspi.h"
#endif
#ifdef HAL_IWDG_MODULE_ENABLED
#include "stm32n6xx_hal_iwdg.h"
#endif
#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32n6xx_hal_pcd.h"
#endif

#define assert_param(expr) ((void)0U)

#ifdef __cplusplus
}
#endif

#endif /* STM32N6xx_HAL_CONF_H */
