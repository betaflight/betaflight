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

typedef struct led_config_s {
    GPIO_TypeDef *gpio;
    uint16_t pin;
} led_config_t;

extern led_config_t led_config[3];

// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              digitalToggle(led_config[0].gpio, led_config[0].pin)
#ifndef LED0_INVERTED
#define LED0_OFF                 digitalHi(led_config[0].gpio, led_config[0].pin)
#define LED0_ON                  digitalLo(led_config[0].gpio, led_config[0].pin)
#else
#define LED0_OFF                 digitalLo(led_config[0].gpio, led_config[0].pin)
#define LED0_ON                  digitalHi(led_config[0].gpio, led_config[0].pin)
#endif // inverted
#else
#define LED0_TOGGLE              do {} while(0)
#define LED0_OFF                 do {} while(0)
#define LED0_ON                  do {} while(0)
#endif

#ifdef LED1
#define LED1_TOGGLE              digitalToggle(led_config[1].gpio, led_config[1].pin)
#ifndef LED1_INVERTED
#define LED1_OFF                 digitalHi(led_config[1].gpio, led_config[1].pin)
#define LED1_ON                  digitalLo(led_config[1].gpio, led_config[1].pin)
#else
#define LED1_OFF                 digitalLo(led_config[1].gpio, led_config[1].pin)
#define LED1_ON                  digitalHi(led_config[1].gpio, led_config[1].pin)
#endif // inverted
#else
#define LED1_TOGGLE              do {} while(0)
#define LED1_OFF                 do {} while(0)
#define LED1_ON                  do {} while(0)
#endif


#ifdef LED2
#define LED2_TOGGLE              digitalToggle(led_config[2].gpio, led_config[2].pin)
#ifndef LED2_INVERTED
#define LED2_OFF                 digitalHi(led_config[2].gpio, led_config[2].pin)
#define LED2_ON                  digitalLo(led_config[2].gpio, led_config[2].pin)
#else
#define LED2_OFF                 digitalLo(led_config[2].gpio, led_config[2].pin)
#define LED2_ON                  digitalHi(led_config[2].gpio, led_config[2].pin)
#endif // inverted
#else
#define LED2_TOGGLE              do {} while(0)
#define LED2_OFF                 do {} while(0)
#define LED2_ON                  do {} while(0)
#endif

void ledInit(bool alternative_led);
