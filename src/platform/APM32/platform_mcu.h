/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef APM32F4

#include "apm32f4xx.h"
#include "apm32f4xx_dal.h"
#include "system_apm32f4xx.h"

#include "apm32f4xx_ddl_spi.h"
#include "apm32f4xx_ddl_gpio.h"
#include "apm32f4xx_ddl_dma.h"
#include "apm32f4xx_ddl_rcm.h"
#include "apm32f4xx_ddl_bus.h"
#include "apm32f4xx_ddl_tmr.h"
#include "apm32f4xx_ddl_system.h"
#include "apm32f4xx_ddl_adc.h"

#include "apm32f4xx_ddl_ex.h"

// Aliases
#define HAL_StatusTypeDef           DAL_StatusTypeDef
#define HAL_RCC_GetSysClockFreq     DAL_RCM_GetSysClockFreq
#define HAL_IncTick                 DAL_IncTick
#define HAL_TIM_IC_Start_IT         DAL_TMR_IC_Start_IT
#define HAL_TIM_IC_ConfigChannel    DAL_TMR_IC_ConfigChannel
#define HAL_NVIC_SetPriority        DAL_NVIC_SetPriority
#define HAL_NVIC_EnableIRQ          DAL_NVIC_EnableIRQ

#define __HAL_TIM_GetAutoreload     __DAL_TMR_GET_AUTORELOAD
#define __HAL_TIM_SetCounter        __DAL_TMR_SET_COUNTER
#define __HAL_DMA_GET_COUNTER       __DAL_DMA_GET_COUNTER
#define __HAL_UART_ENABLE_IT        __DAL_UART_ENABLE_IT

#define LL_TIM_InitTypeDef          DDL_TMR_InitTypeDef
#define LL_TIM_DeInit               DDL_TMR_DeInit
#define LL_TIM_OC_InitTypeDef       DDL_TMR_OC_InitTypeDef
#define LL_TIM_IC_InitTypeDef       DDL_TMR_IC_InitTypeDef
#define LL_TIM_SetAutoReload        DDL_TMR_SetAutoReload
#define LL_TIM_DisableIT_UPDATE     DDL_TMR_DisableIT_UPDATE
#define LL_TIM_DisableCounter       DDL_TMR_DisableCounter
#define LL_TIM_SetCounter           DDL_TMR_SetCounter
#define LL_TIM_ClearFlag_UPDATE     DDL_TMR_ClearFlag_UPDATE
#define LL_TIM_EnableIT_UPDATE      DDL_TMR_EnableIT_UPDATE
#define LL_TIM_EnableCounter        DDL_TMR_EnableCounter
#define LL_TIM_GenerateEvent_UPDATE DDL_TMR_GenerateEvent_UPDATE
#define LL_EX_TIM_DisableIT         DDL_EX_TMR_DisableIT

#define LL_DMA_InitTypeDef          DDL_DMA_InitTypeDef
#define LL_EX_DMA_DeInit            DDL_EX_DMA_DeInit
#define LL_EX_DMA_Init              DDL_EX_DMA_Init
#define LL_EX_DMA_DisableResource   DDL_EX_DMA_DisableResource
#define LL_EX_DMA_EnableResource    DDL_EX_DMA_EnableResource
#define LL_EX_DMA_GetDataLength     DDL_EX_DMA_GetDataLength
#define LL_EX_DMA_SetDataLength     DDL_EX_DMA_SetDataLength
#define LL_EX_DMA_EnableIT_TC       DDL_EX_DMA_EnableIT_TC

#define TIM_TypeDef                 TMR_TypeDef
#define TIM_HandleTypeDef           TMR_HandleTypeDef
#define TIM_ICPOLARITY_RISING       TMR_ICPOLARITY_RISING
#define TIM_CCxChannelCmd           TMR_CCxChannelCmd
#define TIM_CCx_DISABLE             TMR_CCx_DISABLE
#define TIM_CCx_ENABLE              TMR_CCx_ENABLE
#define TIM_CCxChannelCmd           TMR_CCxChannelCmd
#define TIM_IC_InitTypeDef          TMR_IC_InitTypeDef
#define TIM_ICPOLARITY_FALLING      TMR_ICPOLARITY_FALLING
#define TIM_ICSELECTION_DIRECTTI    TMR_ICSELECTION_DIRECTTI
#define TIM_ICPSC_DIV1              TMR_ICPSC_DIV1

#ifdef USE_DAL_DRIVER
#define USE_HAL_DRIVER
#endif

#ifdef USE_FULL_DDL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

#endif // APM32F4

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx)
#define USE_FAST_DATA

// Chip Unique ID on APM32F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#define USE_PIN_AF

#ifndef APM32F4
#define APM32F4
#endif

#endif

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS

#define USE_OVERCLOCK

#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10

#define DEFAULT_CPU_OVERCLOCK 0

#define FAST_IRQ_HANDLER

#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO        static

// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))

#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#define USE_TIMER_MGMT
#define USE_TIMER_AF

#if defined(APM32F4)

//speed is packed inside modebits 5 and 2,
#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)

#define IO_CONFIG_GET_MODE(cfg)  (((cfg) >> 0) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 4) & 0x01)
#define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 5) & 0x03)

#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)

#define SPIDEV_COUNT 3

#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->DR)

#define MAX_SPI_PIN_SEL 2

#define USE_TX_IRQ_HANDLER

#define UART_TX_BUFFER_ATTRIBUTE /* NONE */
#define UART_RX_BUFFER_ATTRIBUTE /* NONE */

#define UART_TRAIT_AF_PORT 1

#define UARTHARDWARE_MAX_PINS 4

#define UART_REG_RXD(base) ((base)->DATA)
#define UART_REG_TXD(base) ((base)->DATA)

#define DMA_TRAIT_CHANNEL 1

#define USB_DP_PIN PA12

#define FLASH_CONFIG_BUFFER_TYPE uint32_t
#endif
