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

#ifdef BEEPER
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN)
#define BEEP_OFF                 systemBeep(false)
#define BEEP_ON                  systemBeep(true)
#else
#define BEEP_TOGGLE
#define BEEP_OFF
#define BEEP_ON
#endif

typedef struct beeperConfig_s {
    uint32_t gpioPeripheral;
    uint16_t gpioPin;
    GPIO_TypeDef *gpioPort;
    GPIO_Mode gpioMode;
    bool isInverted;
} beeperConfig_t;

void systemBeep(bool onoff);
#ifdef CC3D
void beeperInit(beeperConfig_t *beeperConfig, uint8_t use_buzzer_p6);
#else
void beeperInit(beeperConfig_t *beeperConfig);
#endif

void initBeeperHardware(beeperConfig_t *config);
