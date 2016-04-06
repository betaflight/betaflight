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

typedef enum {
    INPUT_FILTERING_DISABLED = 0,
    INPUT_FILTERING_ENABLED
} inputFilteringMode_e;

typedef struct pwmRxConfig_s {
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.
} pwmRxConfig_t;

PG_DECLARE(pwmRxConfig_t, pwmRxConfig);

#define PPM_RCVR_TIMEOUT            0


void ppmInConfig(const timerHardware_t *timerHardwarePtr);
void ppmAvoidPWMTimerClash(const timerHardware_t *timerHardwarePtr, TIM_TypeDef *sharedPwmTimer);

void pwmInConfig(const timerHardware_t *timerHardwarePtr, uint8_t channel);
uint16_t pwmRead(uint8_t channel);
uint16_t ppmRead(uint8_t channel);

bool isPPMDataBeingReceived(void);
void resetPPMDataReceivedState(void);

void pwmRxInit(void);

bool isPWMDataBeingReceived(void);
