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
#define DISPLAY
#define TELEMETRY
#define LED_STRIP
#define USE_SERVOS

#define SERIAL_PORT_COUNT 4

#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 6

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

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

typedef enum {TEST_IRQ = 0 } IRQn_Type;
