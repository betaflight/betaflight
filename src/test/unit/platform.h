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

#define MAG
#define BARO
#define GPS
#define TELEMETRY
#define LED_STRIP
#define USE_SERVOS

#define SERIAL_PORT_COUNT 4

typedef enum
{
    Mode_TEST = 0x0,
} GPIO_Mode;

typedef struct
{
    void* test;
} GPIO_TypeDef;

typedef struct
{
    void* test;
} TIM_TypeDef;

typedef struct DMA_Channel_Struct DMA_Channel_TypeDef;
typedef struct USART_Struct USART_TypeDef;

#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 6

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

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
