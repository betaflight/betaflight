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

#if defined(STM32F10X)
typedef enum
{
    Mode_AIN = 0x0,
    Mode_IN_FLOATING = 0x04,
    Mode_IPD = 0x28,
    Mode_IPU = 0x48,
    Mode_Out_OD = 0x14,
    Mode_Out_PP = 0x10,
    Mode_AF_OD = 0x1C,
    Mode_AF_PP = 0x18
} GPIO_Mode;
#endif

#ifdef STM32F303xC

/*
typedef enum
{
  GPIO_Mode_IN   = 0x00, // GPIO Input Mode
  GPIO_Mode_OUT  = 0x01, // GPIO Output Mode
  GPIO_Mode_AF   = 0x02, // GPIO Alternate function Mode
  GPIO_Mode_AN   = 0x03  // GPIO Analog In/Out Mode
}GPIOMode_TypeDef;

typedef enum
{
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;

typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;
*/

typedef enum
{
    Mode_AIN =          (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_AN,
    Mode_IN_FLOATING =  (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_IN,
    Mode_IPD =          (GPIO_PuPd_DOWN   << 2) | GPIO_Mode_IN,
    Mode_IPU =          (GPIO_PuPd_UP     << 2) | GPIO_Mode_IN,
    Mode_Out_OD =       (GPIO_OType_OD << 4) | GPIO_Mode_OUT,
    Mode_Out_PP =       (GPIO_OType_PP << 4) | GPIO_Mode_OUT,
    Mode_AF_OD =        (GPIO_OType_OD << 4) | GPIO_Mode_AF,
    Mode_AF_PP =        (GPIO_OType_PP << 4) | GPIO_Mode_AF,
    Mode_AF_PP_PD =     (GPIO_OType_PP << 4) | (GPIO_PuPd_DOWN  << 2) | GPIO_Mode_AF,
    Mode_AF_PP_PU =     (GPIO_OType_PP << 4) | (GPIO_PuPd_UP    << 2) | GPIO_Mode_AF
} GPIO_Mode;
#endif

typedef enum
{
    Speed_10MHz = 1,
    Speed_2MHz,
    Speed_50MHz
} GPIO_Speed;

typedef enum
{
    Pin_0 = 0x0001,
    Pin_1 = 0x0002,
    Pin_2 = 0x0004,
    Pin_3 = 0x0008,
    Pin_4 = 0x0010,
    Pin_5 = 0x0020,
    Pin_6 = 0x0040,
    Pin_7 = 0x0080,
    Pin_8 = 0x0100,
    Pin_9 = 0x0200,
    Pin_10 = 0x0400,
    Pin_11 = 0x0800,
    Pin_12 = 0x1000,
    Pin_13 = 0x2000,
    Pin_14 = 0x4000,
    Pin_15 = 0x8000,
    Pin_All = 0xFFFF
} GPIO_Pin;

typedef struct
{
    uint16_t pin;
    GPIO_Mode mode;
    GPIO_Speed speed;
} gpio_config_t;

#ifndef UNIT_TEST
static inline void digitalHi(GPIO_TypeDef *p, uint16_t i) { p->BSRR = i; }
static inline void digitalLo(GPIO_TypeDef *p, uint16_t i)     { p->BRR = i; }
static inline void digitalToggle(GPIO_TypeDef *p, uint16_t i) { p->ODR ^= i; }
static inline uint16_t digitalIn(GPIO_TypeDef *p, uint16_t i) {return p->IDR & i; }
#else
uint16_t digitalIn(GPIO_TypeDef *p, uint16_t i);
#endif

void gpioInit(GPIO_TypeDef *gpio, const gpio_config_t *config);
void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc);
void gpioPinRemapConfig(uint32_t remap, bool enable);
