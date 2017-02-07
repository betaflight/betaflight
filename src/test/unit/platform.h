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

#define U_ID_0 0
#define U_ID_1 1
#define U_ID_2 2

typedef enum
{
    Mode_TEST = 0x0,
    Mode_Out_PP = 0x10,
} GPIO_Mode;

typedef struct
{
    void* test;
} GPIO_TypeDef;

typedef struct
{
    void* test;
} TIM_TypeDef;

typedef enum {
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,
  EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

typedef enum {TEST_IRQ = 0 } IRQn_Type;

typedef struct {
    void* test;
} DMA_Channel_TypeDef;

uint8_t DMA_GetFlagStatus(uint32_t);
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState );
void DMA_ClearFlag(uint32_t);

typedef struct
{
  uint32_t CTRL;                    /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  uint32_t LOAD;                    /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register       */
  uint32_t VAL;                     /*!< Offset: 0x008 (R/W)  SysTick Current Value Register      */
  uint32_t CALIB;                   /*!< Offset: 0x00C (R/ )  SysTick Calibration Register        */
} SysTick_Type;

#define SysTick_CTRL_COUNTFLAG_Pos         16                                             /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk         (1UL << SysTick_CTRL_COUNTFLAG_Pos)            /*!< SysTick CTRL: COUNTFLAG Mask */

extern SysTick_Type *SysTick;


#define WS2811_DMA_TC_FLAG 1
#define WS2811_DMA_HANDLER_IDENTIFER 0

#include "target.h"
