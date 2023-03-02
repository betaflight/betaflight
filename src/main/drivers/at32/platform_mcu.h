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

#if defined(AT32F435ZMT7)

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
#define SystemCoreClock     system_core_clock
#define EXTI_TypeDef        exint_type
#define EXTI_InitTypeDef    exint_init_type
#define USART_TypeDef       usart_type

// Chip Unique ID on F43X
#define U_ID_0 (*(uint32_t*)0x1ffff7e8)
#define U_ID_1 (*(uint32_t*)0x1ffff7ec)
#define U_ID_2 (*(uint32_t*)0x1ffff7f0)

#define USE_PIN_AF

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

#define USE_TIMER_MGMT
#define USE_TIMER_AF
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_ADC_INTERNAL

#define USE_LATE_TASK_STATISTICS

#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100

#define DEFAULT_CPU_OVERCLOCK 0
#define FAST_IRQ_HANDLER

#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA

#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#define USE_LATE_TASK_STATISTICS

#ifndef SPI1_SCK_PIN
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#ifndef SPI2_SCK_PIN
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI4_SCK_PIN
#define SPI4_SCK_PIN    NONE
#define SPI4_MISO_PIN   NONE
#define SPI4_MOSI_PIN   NONE
#endif

#ifndef SPI5_SCK_PIN
#define SPI5_SCK_PIN    NONE
#define SPI5_MISO_PIN   NONE
#define SPI5_MOSI_PIN   NONE
#endif

#ifndef SPI6_SCK_PIN
#define SPI6_SCK_PIN    NONE
#define SPI6_MISO_PIN   NONE
#define SPI6_MOSI_PIN   NONE
#endif
