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

#if defined(RP2350)

#include "RP2350.h"

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#define I2C_TypeDef          I2C0_Type
//#define I2C_HandleTypeDef
#define GPIO_TypeDef         IO_BANK0_Type
//#define GPIO_InitTypeDef
#define TIM_TypeDef          TIMER0_Type
//#define TIM_OCInitTypeDef
#define DMA_TypeDef          DMA_Type
//#define DMA_InitTypeDef
//#define DMA_Channel_TypeDef
#define SPI_TypeDef          SPI0_Type
#define ADC_TypeDef          ADC_Type
#define USART_TypeDef        UART0_Type
#define TIM_OCInitTypeDef    TIMER0_Type
#define TIM_ICInitTypeDef    TIMER0_Type
//#define TIM_OCStructInit
//#define TIM_Cmd
//#define TIM_CtrlPWMOutputs
//#define TIM_TimeBaseInit
//#define TIM_ARRPreloadConfig
//#define SystemCoreClock
//#define EXTI_TypeDef
//#define EXTI_InitTypeDef


#endif

#define DEFAULT_CPU_OVERCLOCK           0
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10

#define IOCFG_OUT_PP          0
#define IOCFG_OUT_OD          0
#define IOCFG_AF_PP           0
#define IOCFG_AF_OD           0
#define IOCFG_IPD             0
#define IOCFG_IPU             0
#define IOCFG_IN_FLOATING     0

