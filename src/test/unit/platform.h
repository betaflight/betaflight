/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdio.h>
#include <stdint.h>

#define USE_PARAMETER_GROUPS

#define U_ID_0 0
#define U_ID_1 1
#define U_ID_2 2

#define NOINLINE
#define FAST_CODE
#define FAST_CODE_NOINLINE
#define FAST_DATA_ZERO_INIT
#define FAST_DATA

#define PID_PROFILE_COUNT 4
#define CONTROL_RATE_PROFILE_COUNT  4
#define USE_MAG
#define USE_BARO
#define USE_GPS
#define USE_DASHBOARD
#define USE_TELEMETRY
#define USE_LED_STRIP
#define USE_SERVOS
#define USE_TRANSPONDER

typedef enum
{
    Mode_TEST = 0x0,
    Mode_Out_PP = 0x10
} GPIO_Mode;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {TEST_IRQ = 0 } IRQn_Type;
typedef enum {
    EXTI_Trigger_Rising = 0x08,
    EXTI_Trigger_Falling = 0x0C,
    EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

typedef struct
{
    void *test;
} GPIO_TypeDef;

typedef struct
{
    void* test;
} TIM_TypeDef;

typedef struct
{
    void* test;
} TIM_OCInitTypeDef;

typedef struct {
    void* test;
} DMA_TypeDef;

typedef struct {
    void* test;
} DMA_Channel_TypeDef;

uint8_t DMA_GetFlagStatus(void *);
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState );
void DMA_ClearFlag(uint32_t);

typedef struct
{
    void* test;
} SPI_TypeDef;

typedef struct
{
    void* test;
} USART_TypeDef;

typedef struct
{
    void *test;
} I2C_TypeDef;

typedef struct
{
    void* test;
} ADC_TypeDef;

#define WS2811_DMA_TC_FLAG (void *)1
#define WS2811_DMA_HANDLER_IDENTIFER 0
#define NVIC_PriorityGroup_2 0x500

#define MCU_TYPE_ID   99
#define MCU_TYPE_NAME "UNIT_TEST"

#include "target.h"

#include "target/common_defaults_post.h"
