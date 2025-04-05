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

#define _ADDRESSMAP_H

#define NVIC_PriorityGroup_2         0x500

#define SPI_IO_AF_CFG           0
#define SPI_IO_AF_SCK_CFG_HIGH  0
#define SPI_IO_AF_SCK_CFG_LOW   0
#define SPI_IO_AF_SDI_CFG       0
#define SPI_IO_CS_CFG           0

// Register address offsets for atomic RMW aliases
#define REG_ALIAS_RW_BITS  (_u(0x0) << _u(12))
#define REG_ALIAS_XOR_BITS (_u(0x1) << _u(12))
#define REG_ALIAS_SET_BITS (_u(0x2) << _u(12))
#define REG_ALIAS_CLR_BITS (_u(0x3) << _u(12))

#include "RP2350.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

#if defined(RP2350A) || defined(RP2350B)

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#define I2C_TypeDef          i2c0_hw
//#define I2C_HandleTypeDef
#define GPIO_TypeDef         io_bank0_hw_t
//#define GPIO_InitTypeDef
#define TIM_TypeDef          void*
//#define TIM_OCInitTypeDef
#define DMA_TypeDef          void*
#define DMA_InitTypeDef      void*
//#define DMA_Channel_TypeDef
#define SPI_TypeDef          SPI0_Type
#define ADC_TypeDef          void*
#define USART_TypeDef        uart_inst_t
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
//#define IRQn_Type           void*

#define SPI_INST(spi) ((spi_inst_t *)(spi))


#endif

#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO            static
#define FAST_IRQ_HANDLER

#define DEFAULT_CPU_OVERCLOCK           0
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10

#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP          IO_CONFIG(GPIO_OUT, 0, 0)
#define IOCFG_OUT_OD          IO_CONFIG(GPIO_OUT, 0, 0)
#define IOCFG_AF_PP           0
#define IOCFG_AF_OD           0
#define IOCFG_IPD             IO_CONFIG(GPIO_IN, 0, 0)
#define IOCFG_IPU             IO_CONFIG(GPIO_IN, 0, 0)
#define IOCFG_IN_FLOATING     IO_CONFIG(GPIO_IN, 0, 0)

#define SERIAL_UART_FIRST_INDEX     0

extern uint32_t systemUniqueId[3];

// PICOs have an 8 byte unique identifier.
#define U_ID_0 (systemUniqueId[0])
#define U_ID_1 (systemUniqueId[1])
#define U_ID_2 (systemUniqueId[2])

#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE

#define MAX_SPI_PIN_SEL 4
#define SERIAL_TRAIT_PIN_CONFIG 1

#define xDMA_GetCurrDataCounter(dma_resource) (((dma_channel_hw_t *)(dma_resource))->transfer_count)
