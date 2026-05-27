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

#if defined(X32M7)

#if !defined(CORE_CM4) && !defined(CORE_CM7)
#define CORE_CM7
#endif

#include "x32m7xx.h"
#include "system_x32m7xx.h"

#include "misc.h"
#include "x32m7xx_adc.h"
#include "x32m7xx_dac.h"
#include "x32m7xx_dbg.h"
#include "x32m7xx_dma.h"
#include "x32m7xx_dmamux.h"
#include "x32m7xx_exti.h"
#include "x32m7xx_gpio.h"
#include "x32m7xx_i2c.h"
#include "x32m7xx_lptim.h"
#include "x32m7xx_lpuart.h"
#include "x32m7xx_pwr.h"
#include "x32m7xx_rcc.h"
#include "x32m7xx_rtc.h"
#include "x32m7xx_sdmmc.h"
#include "x32m7xx_spi.h"
#include "x32m7xx_smu.h"
#include "x32m7xx_tim.h"
#include "x32m7xx_usart.h"
#include "x32m7xx_xspi_v2.h"
#include "x32m7xx_otpc.h"
#include "sdmmc_host.h"
#include "sdmmc_spec.h"

// Chip Unique ID on X32M7xx
extern uint32_t systemUniqueId[3];

#define DSHOT_BITBANGED_TIMER_ATIM1 DSHOT_BITBANGED_TIMER_TIM1
#define DSHOT_BITBANGED_TIMER_ATIM2 DSHOT_BITBANGED_TIMER_TIM8

#define U_ID_0 (systemUniqueId[0])
#define U_ID_1 (systemUniqueId[1])
#define U_ID_2 (systemUniqueId[2])

#define PLATFORM_TRAIT_ADC_DEVICE 1
#define PLATFORM_TRAIT_RCC        1
#define SERIAL_TRAIT_PIN_CONFIG   1

#define SPI_TRAIT_AF_PIN          1
#define I2C_TRAIT_AF_PIN          1
#define I2C_TRAIT_STATE           1
#define I2C_TRAIT_HANDLE          1
#define I2CDEV_COUNT              10

#define UART_TRAIT_AF_PIN         1
#define UART_TRAIT_PINSWAP        1
#define UART_TRAIT_BIDIR_PP_PREPEND 1
#define UARTHARDWARE_MAX_PINS     7

#define DMA_TRAIT_MUX             1

#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_DMA_SPEC
#ifdef USE_DSHOT
#define USE_DSHOT_CACHE_MGMT
#endif
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS
#define USE_TIMER_MGMT
#define USE_TIMER_AF
#define USE_ADC_DEVICE_0

#define TASK_GYROPID_DESIRED_PERIOD     125
#define SCHEDULER_DELAY_LIMIT           10
#define DEFAULT_CPU_OVERCLOCK           0
#define FAST_IRQ_HANDLER

#define DMA_DATA_ZERO_INIT         __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                   __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA

#define DMA_RAM                    __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI                 __attribute__((section(".DMA_RW_AXI"), aligned(32)))
#define DMA_RAM_R                  __attribute__((section(".DMA_RAM_R"), aligned(32)))
#define DMA_RAM_W                  __attribute__((section(".DMA_RAM_W"), aligned(32)))
#define DMA_RAM_RW                 __attribute__((section(".DMA_RAM_RW"), aligned(32)))

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
#define X32_DCACHE_ENABLED() ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
#define X32_CLEAN_DCACHE_BY_ADDR(addr, size) do { \
    if (X32_DCACHE_ENABLED()) { \
        SCB_CleanDCache_by_Addr((uint32_t *)(addr), (size)); \
    } \
} while (0)
#define X32_INVALIDATE_DCACHE_BY_ADDR(addr, size) do { \
    if (X32_DCACHE_ENABLED()) { \
        SCB_InvalidateDCache_by_Addr((uint32_t *)(addr), (size)); \
    } \
} while (0)
#else
#define X32_CLEAN_DCACHE_BY_ADDR(addr, size) do { (void)(addr); (void)(size); } while (0)
#define X32_INVALIDATE_DCACHE_BY_ADDR(addr, size) do { (void)(addr); (void)(size); } while (0)
#endif

extern uint8_t _dmaram_start__;
extern uint8_t _dmaram_end__;

#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))

#define SET_BIT(REG, BIT)           ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)         ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)          ((REG) & (BIT))
#define CLEAR_REG(REG)              ((REG) = (0x0))
#define WRITE_REG(REG, VAL)         ((REG) = (VAL))
#define READ_REG(REG)               ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

typedef I2C_Module          I2C_TypeDef;
typedef I2C_InitType        I2C_HandleTypeDef;
typedef GPIO_Module         GPIO_TypeDef;
typedef GPIO_InitType       GPIO_InitTypeDef;
typedef TIM_Module          TIM_TypeDef;
typedef DMA_Module          DMA_TypeDef;
typedef DMA_ChannelType     DMA_Stream_TypeDef;
typedef DMA_ChannelType     DMA_Channel_TypeDef;
typedef SPI_Module          SPI_TypeDef;
typedef ADC_Module          ADC_TypeDef;
typedef USART_Module        USART_TypeDef;
typedef EXTI_Module         EXTI_TypeDef;
typedef EXTI_InitType       EXTI_InitTypeDef;
typedef NVIC_InitType       NVIC_InitTypeDef;

#define DMA_InitTypeDef            DMA_ChInitType
#define TIM_OCInitTypeDef          OCInitType
#define TIM_ICInitTypeDef          TIM_ICInitType
#define TIM_OCStructInit           TIM_InitOcStruct
#define TIM_TimeBaseInitTypeDef    TIM_TimeBaseInitType

#define TIM_ICPolarity_Falling     TIM_IC_POLARITY_FALLING
#define TIM_ICPolarity_Rising      TIM_IC_POLARITY_RISING

#define RCC_GetClocksFreq          RCC_GetClocksFreqValue
#define SYSCLK_Frequency           SysClkFreq

#define GPIO_SPEED_FREQ_LOW        GPIO_DC_2mA
#define GPIO_SPEED_FREQ_HIGH       GPIO_DC_8mA
#define GPIO_SPEED_FREQ_VERY_HIGH  GPIO_DC_12mA
#define GPIO_NOPULL                GPIO_NO_PULL
#define GPIO_PULLDOWN              GPIO_PULL_DOWN
#define GPIO_PULLUP                GPIO_PULL_UP
#define GPIO_PIN_RESET             0

#define UART_TX_BUFFER_ATTRIBUTE     DMA_DATA_ZERO_INIT
#define UART_RX_BUFFER_ATTRIBUTE     DMA_DATA_ZERO_INIT
#define ENABLE_SERIAL_SKIP_CHECK_TX  1

// X32 IO config packing uses the speed field as GPIO_Current.
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

#define IO_CONFIG_GET_MODE(cfg)     ((cfg) & 0x13)
#define IO_CONFIG_GET_SPEED(cfg)    (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg)    (((cfg) >> 4) & 0x01)
#define IO_CONFIG_GET_PULL(cfg)     (((cfg) >> 5) & 0x03)
#define IO_CONFIG_GET_CURRENT(cfg)  IO_CONFIG_GET_SPEED(cfg)
#define IO_CONFIG_GET_DS(cfg)       IO_CONFIG_GET_SPEED(cfg)
#define IO_CONFIG_GET_SLEW(cfg)     (IO_CONFIG_GET_SPEED(cfg) == GPIO_SPEED_FREQ_LOW ? GPIO_SLEW_RATE_SLOW : GPIO_SLEW_RATE_FAST)

#define FLASH_CONFIG_BUFFER_TYPE uint32_t

#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_SCK_CFG       SPI_IO_AF_SCK_CFG_LOW
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)

#define SPIDEV_COUNT 7
#define MAX_SPI_PIN_SEL 5
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) (SPI_I2S_GetStatus((SPI_TypeDef *)(instance), SPI_I2S_RNE_FLAG) == SET)
#define SPI_RX_DATA_REGISTER(base) ((base)->DAT)

#define USE_TX_IRQ_HANDLER

#define UART_REG_RXD(base) (((USART_TypeDef *)(base))->DAT)
#define UART_REG_TXD(base) (((USART_TypeDef *)(base))->DAT)

#define DMA_CHANREQ_STRING "Request"
#define DMA_STCH_STRING    "Channel"

#define ADC_INTERNAL_VBAT4_ENABLED 1

#define USB_DP_PIN PA12

#ifdef USE_FAST_DATA
#define FAST_DATA_ZERO_INIT         __attribute__ ((section(".tcm_bss"), aligned(4)))
#define FAST_DATA                   __attribute__ ((section(".tcm_data"), aligned(4)))
#endif

// NVIC priority utility macros
#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base, sub) (((((base) << (4 - (7 - (NVIC_PRIORITY_GROUPING >> 8)))) | ((sub) & (0x0f >> (7 - (NVIC_PRIORITY_GROUPING >> 8))))) << 4) & 0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio) >> (4 - (7 - (NVIC_PRIORITY_GROUPING >> 8)))) >> 4)
#define NVIC_PRIORITY_SUB(prio) (((prio) >> 4) & (0x0f >> (7 - (NVIC_PRIORITY_GROUPING >> 8))))

#define PERIPH_INT(periph) ((uint32_t)(periph))

#endif
