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

#define MAG
#define BARO
#define GPS
#define TELEMETRY
#define LED_STRIP
#define USE_SERVOS

#define USART1              ((USART_TypeDef *) 1)
#define USART2              ((USART_TypeDef *) 2)
#define USART3              ((USART_TypeDef *) 3)
#define UART4              ((USART_TypeDef *) 4)
#define UART5              ((USART_TypeDef *) 5)

#define NVIC_PriorityGroup_2         ((uint32_t)0x500)

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

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

typedef enum {TEST_IRQ = 0 } IRQn_Type;

typedef struct {
    uint32_t ISR;
    uint32_t IFCR;
} DMA_TypeDef;

typedef struct {
    void* test;
} DMA_Channel_TypeDef;

//typedef struct DMA_Channel_Struct DMA_Channel_TypeDef;
typedef struct USART_Struct USART_TypeDef;

uint8_t DMA_GetFlagStatus(uint32_t);
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState );
void DMA_ClearFlag(uint32_t);

#define WS2811_DMA_TC_FLAG 1
#define WS2811_DMA_HANDLER_IDENTIFER 0

#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 6

#define USE_ADC

#define ADC_CHANNEL_COUNT 3

#define ADC_BATTERY     ADC_CHANNEL0
#define ADC_CURRENT     ADC_CHANNEL1
#define ADC_EXTERNAL    ADC_CHANNEL2

typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
} FLASH_Status;

void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);

#include "target.h"
