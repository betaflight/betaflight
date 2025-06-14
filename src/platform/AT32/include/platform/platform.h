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

#if defined(AT32F435)

#include "at32f435_437.h"
#include "at32f435_437_i2c.h"
#include "i2c_application.h"

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#define I2C_TypeDef         i2c_type
#define I2C_HandleTypeDef   i2c_handle_type
#define GPIO_TypeDef        gpio_type
#define GPIO_InitTypeDef    gpio_init_type
#define TIM_TypeDef         tmr_type
#define TIM_OCInitTypeDef   tmr_output_config_type
#define DMA_TypeDef         dma_type
#define DMA_InitTypeDef     dma_init_type
#define DMA_Channel_TypeDef dma_channel_type
#define SPI_TypeDef         spi_type
#define ADC_TypeDef         adc_type
#define USART_TypeDef       usart_type
#define TIM_OCInitTypeDef   tmr_output_config_type
#define TIM_ICInitTypeDef   tmr_input_config_type
#define TIM_OCStructInit    tmr_output_default_para_init
#define TIM_Cmd             tmr_counter_enable
#define TIM_CtrlPWMOutputs  tmr_output_enable
#define TIM_TimeBaseInit    tmr_base_init
#define TIM_ARRPreloadConfig tmr_period_buffer_enable
#define SystemCoreClock     system_core_clock
#define EXTI_TypeDef        exint_type
#define EXTI_InitTypeDef    exint_init_type
#define USART_TypeDef       usart_type

// Chip Unique ID on F43X
#define U_ID_0 (*(uint32_t*)0x1ffff7e8)
#define U_ID_1 (*(uint32_t*)0x1ffff7ec)
#define U_ID_2 (*(uint32_t*)0x1ffff7f0)

#ifndef AT32F4
#define AT32F4
#endif

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#endif

#define USE_USB_MSC

#define USE_TIMER_MGMT
#define USE_TIMER_AF
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_ADC_INTERNAL

#define USE_LATE_TASK_STATISTICS

#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100

#define DEFAULT_CPU_OVERCLOCK 0
#define FAST_IRQ_HANDLER FAST_CODE

#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO        static

#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#define USE_LATE_TASK_STATISTICS

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER

#if defined(AT32F4)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_NONE )  // TODO
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_UP )
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_STRONGER , GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_NONE)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_MUX    , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_MUX    , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_MUX    , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_UP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_MUX    , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_NONE)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT  , GPIO_DRIVE_STRENGTH_MODERATE, 0,             GPIO_PULL_DOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT  , GPIO_DRIVE_STRENGTH_MODERATE, 0,             GPIO_PULL_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT  , GPIO_DRIVE_STRENGTH_MODERATE, 0,             GPIO_PULL_NONE)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT  , GPIO_DRIVE_STRENGTH_MODERATE, 0, GPIO_PULL_UP)

#define IO_CONFIG_GET_MODE(cfg)  (((cfg) >> 0) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 4) & 0x01)
#define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 5) & 0x03)

#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_DOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP)

#define SPIDEV_COUNT       4

#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->DR)

#define MAX_SPI_PIN_SEL    5

#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE

#define PLATFORM_TRAIT_RCC      1
#define UART_TRAIT_AF_PIN       1
#define UART_TRAIT_PINSWAP      1
#define SERIAL_TRAIT_PIN_CONFIG 1
#define I2C_TRAIT_AF_PIN        1
#define I2CDEV_COUNT            4
#define I2C_TRAIT_HANDLE        1
#define SPI_TRAIT_AF_PIN        1
#define UARTHARDWARE_MAX_PINS   5

#define UART_REG_RXD(base) ((base)->dt)
#define UART_REG_TXD(base) ((base)->dt)

#define DMA_TRAIT_MUX 1

#endif

#define FLASH_CONFIG_BUFFER_TYPE      uint32_t

#define USB_DP_PIN PA12
