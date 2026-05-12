/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef X32M7

#include "x32m7xx.h"
#include "x32m7xx_rcc.h"
#include "x32m7xx_i2c.h"
#include "x32m7xx_gpio.h"
#include "x32m7xx_pwr.h"
#include "x32m7xx_dma.h"
#include "x32m7xx_dmamux.h"
#include "x32m7xx_exti.h"
#include "x32m7xx_spi.h"
#include "x32m7xx_tim.h"
#include "x32m7xx_usart.h"
#include "x32m7xx_xspi_v2.h"
#include "x32m7xx_adc.h"
#include "x32m7xx_dac.h"
#include "x32m7xx_rtc.h"
#include "x32m7xx_sdmmc.h"
#include "x32m7xx_smu.h"
#include "misc.h"
#include "sdmmc_host.h"
#include "sdmmc_spec.h"

// Chip Unique ID on F4xx
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#ifndef X32M7
#define X32M7
#endif

#endif

#ifdef X32M7

/* DMA data mode type */
typedef enum {
    DMA_DATA_MODE_SINGLE = 0,
    DMA_DATA_MODE_MULTI  = 1
} dma_data_mode_enum;

/* DMA general configuration struct */

/* typedef struct
{
    dma_data_mode_enum data_mode;
    dma_subperipheral_enum sub_periph;
    union {
        dma_single_data_parameter_struct init_struct_s;
        dma_multi_data_parameter_struct  init_struct_m;
    } config;
} dma_general_config_struct;  //todo: move to dma_bsp.h */

#endif // X32M7

#ifdef X32M7

#define PLATFORM_TRAIT_ADC_DEVICE 1

#define SPI_TRAIT_AF_PORT         1
#define SPI_TRAIT_AF_PIN          1
#define I2C_TRAIT_STATE           1
#define I2C_TRAIT_AF_PIN          1
#define I2CDEV_COUNT              3
#define PLATFORM_TRAIT_RCC        1
#define UART_TRAIT_AF_PIN         1
#define UART_TRAIT_PINSWAP        1
#define SERIAL_TRAIT_PIN_CONFIG   1
#define DMA_TRAIT_MUX             1

#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
// #define USE_USB_CDC_HID
// #define USE_USB_MSC
// #define USE_PERSISTENT_MSC_RTC
// #define USE_MCO
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

typedef  I2C_Module          I2C_TypeDef;
typedef  I2C_InitType        I2C_HandleTypeDef;
typedef  GPIO_Module         GPIO_TypeDef;
typedef  GPIO_InitType       GPIO_InitTypeDef;
typedef  TIM_Module          TIM_TypeDef;
typedef  DMA_Module          DMA_TypeDef;
typedef  DMA_ChannelType     DMA_Stream_TypeDef;
typedef  DMA_ChannelType     DMA_Channel_TypeDef;
typedef  DMAMUX_Channel_TypeDef dmamux_channel_type;
typedef  SPI_Module          SPI_TypeDef;
typedef  ADC_Module          ADC_TypeDef;
typedef  USART_Module        USART_TypeDef;
typedef  EXTI_Module         EXTI_TypeDef;
typedef  EXTI_InitType       EXTI_InitTypeDef;
typedef  NVIC_InitType       NVIC_InitTypeDef;
#define DMA_InitTypeDef            DMA_ChInitType
#define TIM_OCInitTypeDef          OCInitType
#define TIM_ICInitTypeDef          TIM_ICInitType
#define TIM_OCStructInit           TIM_InitOcStruct
#define TIM_TimeBaseInitTypeDef    TIM_TimeBaseInitType

#define TIM_ICPolarity_Falling      TIM_IC_POLARITY_FALLING
#define TIM_ICPolarity_Rising       TIM_IC_POLARITY_RISING

#define Bit_RESET                  0
#define RCC_GetClocksFreq          RCC_GetClocksFreqValue
#define SYSCLK_Frequency           SysClkFreq

#define GPIO_SPEED_FREQ_LOW        GPIO_DC_2mA
#define GPIO_SPEED_FREQ_HIGH       GPIO_DC_2mA
#define GPIO_SPEED_FREQ_VERY_HIGH  GPIO_DC_2mA
#define GPIO_NOPULL                GPIO_NO_PULL
#define GPIO_PULLDOWN              GPIO_PULL_DOWN
#define GPIO_PULLUP                GPIO_PULL_UP

#define UART_TX_BUFFER_ATTRIBUTE /* EMPTY */
#define UART_RX_BUFFER_ATTRIBUTE /* EMPTY */

#endif /* X32M7 */

#define USE_ADC_DEVICE_0


#if defined(X32M7)
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#define DEFAULT_CPU_OVERCLOCK 0

#if defined(X32M7)
#define FAST_IRQ_HANDLER
#endif

#if defined(X32M7)
// F4 can't DMA to/from TCM (tightly-coupled memory) SRAM (where the stack lives)
#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO        static
#endif

#define DMA_CHANREQ_STRING "Request"

#if defined(X32M7)
// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
#endif

#ifdef USE_DMA_RAM
// Reserved for other GD series
#else
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#endif  // USE_DMA_RAM

#define USE_TIMER_MGMT
#define USE_TIMER_AF  

#if defined(X32M7)

// X32M7XX IO_Config reluer
//    7      6      5         4         3          2       1        0
// <ds-1> <ds-0> <speed> <mode_od>  <pupd-1> <pupd-0> <mode-1> <mode-0>
#define IO_CONFIG(mode, speed, pupd, ds) ((mode) | ((pupd) << 2) | ((speed) << 5) | ((ds) << 6))


#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SLEW_RATE_SLOW, GPIO_NO_PULL,   0x00)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SLEW_RATE_SLOW, GPIO_PULL_UP,   0x00)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SLEW_RATE_FAST, GPIO_NO_PULL,   0x00) // Fast Slew Rate
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SLEW_RATE_SLOW, GPIO_NO_PULL,   0x00)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SLEW_RATE_SLOW, GPIO_NO_PULL,   0x00)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SLEW_RATE_SLOW, GPIO_PULL_DOWN, 0x00)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SLEW_RATE_SLOW, GPIO_PULL_UP,   0x00)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SLEW_RATE_SLOW, GPIO_NO_PULL,   0x00)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SLEW_RATE_SLOW, GPIO_PULL_DOWN, 0x00)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SLEW_RATE_SLOW, GPIO_PULL_UP,   0x00)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SLEW_RATE_SLOW, GPIO_NO_PULL,   0x00)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SLEW_RATE_FAST, GPIO_PULL_UP,   0x00) // Fast Slew Rate  

#endif

#if defined(X32M7)

#define IO_CONFIG_GET_MODE(cfg)  ((((cfg) >> 0) & 0x03) | ((cfg) & 0x10))
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 5) & 0x01)
#define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_DS(cfg)    (((cfg) >> 6) & 0x03)

#endif

#if defined(X32M7)
#define FLASH_CONFIG_BUFFER_TYPE uint8_t
#endif

#if defined(X32M7)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SLEW_RATE_FAST, GPIO_NO_PULL,   0x00)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SLEW_RATE_FAST, GPIO_PULL_DOWN, 0x00)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SLEW_RATE_FAST, GPIO_PULL_UP,   0x00)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SLEW_RATE_FAST, GPIO_NO_PULL,   0x00)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SLEW_RATE_FAST, GPIO_PULL_UP,   0x00)
#else
#error "Invalid X32 MCU defined - requires SPI implementation"
#endif

#if defined(X32M7)
#if defined(X32M760) || defined(X32M76x) || defined(X32M7B)
#define SPIDEV_COUNT 6
#endif
#else
#define SPIDEV_COUNT 4
#endif

#define SPI_RX_DATA_REGISTER(base) ((base)->DAT)

#if defined(X32M7)
#define MAX_SPI_PIN_SEL 3
#else
#error Unknown MCU family
#endif

#if defined(X32M7)
#define USE_TX_IRQ_HANDLER
#endif

// // all pins on given uart use same AF


#if defined(X32M7)
#define UARTHARDWARE_MAX_PINS 7
#endif

#if defined(X32M7)
#define UART_REG_RXD(base) (((USART_Module *)(base))->DAT)
#define UART_REG_TXD(base) (((USART_Module *)(base))->DAT)
#endif

// #define USB_DP_PIN PA12

// #define X32_USART0    ((USART_TypeDef*)USART1)
// #define X32_USART1    ((USART_TypeDef*)USART2)
// #define X32_USART2    ((USART_TypeDef*)USART3)
// #define X32_UART4     ((USART_TypeDef*)USART4)
// #define X32_UART5     ((USART_TypeDef*)USART5)
// #define X32_USART10   ((USART_TypeDef*)UART10)

// #undef USART0
// #define USART0       ((USART_TypeDef*)X32_USART0)

// #undef USART1
// #define USART1       ((USART_TypeDef*)X32_USART1)

// #undef USART2
// #define USART2       ((USART_TypeDef*)X32_USART2)
// #undef UART4
// #define UART4        ((USART_TypeDef*)X32_UART4)
// #undef UART5
// #define UART5        ((USART_TypeDef*)X32_UART5)
// #undef USART10
// #define USART10      ((USART_TypeDef*)X32_USART10)

// #define _UART_GET_PREFIX(dev) _UART_GET_PREFIX_##dev

// #define _UART_GET_PREFIX_UARTDEV_0 USART
// #define _UART_GET_PREFIX_UARTDEV_1 USART
// #define _UART_GET_PREFIX_UARTDEV_2 USART
// #define _UART_GET_PREFIX_UARTDEV_3 UART
// #define _UART_GET_PREFIX_UARTDEV_4 UART
// #define _UART_GET_PREFIX_UARTDEV_5 USART
// #define _UART_GET_PREFIX_UARTDEV_6 UART
// #define _UART_GET_PREFIX_UARTDEV_7 UART
// #define _UART_GET_PREFIX_UARTDEV_8 UART
// #define _UART_GET_PREFIX_UARTDEV_9 UART
// #define _UART_GET_PREFIX_UARTDEV_10 USART
// #define _UART_GET_PREFIX_UARTDEV_LP1 LPUART

// // #if defined(X32M7)
// // We need to redefine ADC0, ADC1, etc.,
// // in the GD firmware library to be compatible with
// // such as the ADC_TypeDef * type in BF.

#define X32_ADC1    ((ADC_TypeDef*)ADC1_BASE)
#define X32_ADC2    ((ADC_TypeDef*)ADC2_BASE)
#define X32_ADC3    ((ADC_TypeDef*)ADC3_BASE)

#undef ADC1
#define ADC1       ((ADC_TypeDef*)X32_ADC1)
#undef ADC2
#define ADC2       ((ADC_TypeDef*)X32_ADC2)
#undef ADC3
#define ADC3       ((ADC_TypeDef*)X32_ADC3)

#define PERIPH_INT(periph)    ((uint32_t)periph)

#define I2C_HandleTypeDef   I2C_TypeDef
#define I2C_TRAIT_HANDLE 1

// NVIC priority utility macros
#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
