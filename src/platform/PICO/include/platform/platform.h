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

#include "RP2350.h"

#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

#define NVIC_PriorityGroup_2         0x500
#define PLATFORM_NO_LIBC             0
#define DEFIO_PORT_PINS              64

#ifdef RP2350

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#define I2C_TypeDef          i2c_inst_t
#define I2C_INST(i2c)        (i2c)

#define GPIO_TypeDef         io_bank0_hw_t
//#define GPIO_InitTypeDef
#define TIM_TypeDef          void*
//#define TIM_OCInitTypeDef

#define DMA_TypeDef          void*
#define DMA_InitTypeDef      dma_channel_config

#define ADC_TypeDef          void*

#define USART_TypeDef        uart_inst_t
#define UART_INST(uart)      (uart)

#define TIM_OCInitTypeDef    void*
#define TIM_ICInitTypeDef    void*
//#define TIM_OCStructInit
//#define TIM_Cmd
//#define TIM_CtrlPWMOutputs
//#define TIM_TimeBaseInit
//#define TIM_ARRPreloadConfig
//#define SystemCoreClock
//#define EXTI_TypeDef
//#define EXTI_InitTypeDef

// We have to use SPI0_Type (or void) because config will pass in SPI0, SPI1,
// which are defined in pico-sdk as SPI0_Type*.
// SPI_INST converts to the correct type for use in pico-sdk functions.
#define SPI_TypeDef          SPI0_Type
#define SPI_INST(spi)        ((spi_inst_t *)(spi))

#define QUADSPI_TypeDef      void
#define MAX_QUADSPI_PIN_SEL  1

#define QUADSPI_TRAIT_CS_SOFTWARE       1

#endif

#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO            static

#if PICO_COPY_TO_RAM == 0
#define FAST_CODE                       __attribute__((section(".fastcode")))
#else
#define FAST_CODE
#endif

#define FAST_IRQ_HANDLER                FAST_CODE


#define DEFAULT_CPU_OVERCLOCK           0

#ifdef TEST_SLOW_SCHEDULE
// (testing) allow time for more / all tasks
// 1000 // 50000 // 125 // 125us = 8kHz
#define TASK_GYROPID_DESIRED_PERIOD     30000
#else
// 125us = 8kHz
#define TASK_GYROPID_DESIRED_PERIOD     125
#endif

#define SCHEDULER_DELAY_LIMIT           10

// There is no library definition for pupd, so define one here
#define GPIO_PULLUP     1
#define GPIO_PULLDOWN   2

// speed will either GPIO_SLEW_RATE_SLOW or GPIO_SLEW_RATE_FAST
#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

// TODO update these and IOConfigGPIO
#define IOCFG_OUT_PP          IO_CONFIG(GPIO_OUT, 0, 0)
#define IOCFG_OUT_OD          IO_CONFIG(GPIO_OUT, 0, 0)
#define IOCFG_AF_PP           0
#define IOCFG_AF_OD           0
#define IOCFG_IPD             IO_CONFIG(GPIO_IN, 0, 0)
#define IOCFG_IPU             IO_CONFIG(GPIO_IN, 0, 0)
#define IOCFG_IN_FLOATING     IO_CONFIG(GPIO_IN, 0, 0)

// TODO update these and IOConfigGPIO
#define SPI_IO_AF_CFG           0
#define SPI_IO_AF_SCK_CFG_HIGH  0
#define SPI_IO_AF_SCK_CFG_LOW   0
#define SPI_IO_AF_SDI_CFG       0
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_OUT, 0, 0)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_IN, 0, GPIO_PULLUP)


#define SERIAL_UART_FIRST_INDEX     0
#define SERIAL_PIOUART_FIRST_INDEX  0

extern uint32_t systemUniqueId[3];

// PICOs have an 8 byte unique identifier.
#define U_ID_0 (systemUniqueId[0])
#define U_ID_1 (systemUniqueId[1])
#define U_ID_2 (systemUniqueId[2])

#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE

#define SERIAL_TRAIT_PIN_CONFIG 1

#define xDMA_GetCurrDataCounter(dma_resource) (((dma_channel_hw_t *)(dma_resource))->transfer_count)

#define USE_LATE_TASK_STATISTICS

#ifndef DEFAULT_VOLTAGE_METER_SCALE
// 100 = 1.00x (100%) scaling; override per target/board to match its VBAT divider
#define DEFAULT_VOLTAGE_METER_SCALE   100
#endif

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
