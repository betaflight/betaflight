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

#include "platform.h"

#ifdef STM32F1
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

#ifdef STM32F3
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

#ifdef STM32F4
typedef enum
{
    Mode_AIN         = (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_AN,
    Mode_IN_FLOATING = (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_IN,
    Mode_IPD         = (GPIO_PuPd_DOWN   << 2) | GPIO_Mode_IN,
    Mode_IPU         = (GPIO_PuPd_UP     << 2) | GPIO_Mode_IN,
    Mode_Out_OD      = (GPIO_OType_OD    << 4) | GPIO_Mode_OUT,
    Mode_Out_PP      = (GPIO_OType_PP    << 4) | GPIO_Mode_OUT,
    Mode_AF_OD       = (GPIO_OType_OD    << 4) | GPIO_Mode_AF,
    Mode_AF_PP       = (GPIO_OType_PP    << 4) | GPIO_Mode_AF,
    Mode_AF_PP_PD    = (GPIO_OType_PP    << 4) | (GPIO_PuPd_DOWN  << 2) | GPIO_Mode_AF,
    Mode_AF_PP_PU    = (GPIO_OType_PP    << 4) | (GPIO_PuPd_UP    << 2) | GPIO_Mode_AF
} GPIO_Mode;
#endif

#ifdef STM32F7
typedef enum
{
    Mode_AIN         = (GPIO_NOPULL << 5) | GPIO_MODE_ANALOG,
    Mode_IN_FLOATING = (GPIO_NOPULL << 5) | GPIO_MODE_INPUT,
    Mode_IPD         = (GPIO_PULLDOWN << 5) | GPIO_MODE_INPUT,
    Mode_IPU         = (GPIO_PULLUP   << 5) | GPIO_MODE_INPUT,
    Mode_Out_OD      = GPIO_MODE_OUTPUT_OD,
    Mode_Out_PP      = GPIO_MODE_OUTPUT_PP,
    Mode_AF_OD       = GPIO_MODE_AF_OD,
    Mode_AF_PP       = GPIO_MODE_AF_PP,
    Mode_AF_PP_PD    = (GPIO_PULLDOWN << 5) | GPIO_MODE_AF_PP,
    Mode_AF_PP_PU    = (GPIO_PULLUP   << 5) | GPIO_MODE_AF_PP
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
#if defined(USE_HAL_DRIVER)
static inline void digitalHi(GPIO_TypeDef *p, uint16_t i) { HAL_GPIO_WritePin(p,i,GPIO_PIN_SET); }
static inline void digitalLo(GPIO_TypeDef *p, uint16_t i) { HAL_GPIO_WritePin(p,i,GPIO_PIN_RESET); }
static inline void digitalToggle(GPIO_TypeDef *p, uint16_t i) { HAL_GPIO_TogglePin(p,i); }
static inline uint16_t digitalIn(GPIO_TypeDef *p, uint16_t i) { return HAL_GPIO_ReadPin(p,i); }
#else
#if defined(STM32F4)
static inline void digitalHi(GPIO_TypeDef *p, uint16_t i) { p->BSRRL = i; }
static inline void digitalLo(GPIO_TypeDef *p, uint16_t i) { p->BSRRH = i; }
#else
static inline void digitalHi(GPIO_TypeDef *p, uint16_t i) { p->BSRR = i; }
static inline void digitalLo(GPIO_TypeDef *p, uint16_t i) { p->BRR = i; }
#endif
static inline void digitalToggle(GPIO_TypeDef *p, uint16_t i) { p->ODR ^= i; }
static inline uint16_t digitalIn(GPIO_TypeDef *p, uint16_t i) { return p->IDR & i; }
#endif
#endif


void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config);
void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc);
void gpioPinRemapConfig(uint32_t remap, bool enable);
