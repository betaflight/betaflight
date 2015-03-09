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

#define FAILSAFE_POWER_ON_DELAY_US (1000 * 1000 * 5)

typedef struct failsafeConfig_s {
    uint8_t failsafe_delay;                 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_off_delay;             // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint16_t failsafe_min_usec;
    uint16_t failsafe_max_usec;
} failsafeConfig_t;


typedef struct failsafeState_s {
    int16_t counter;
    int16_t events;
    bool enabled;
} failsafeState_t;

void useFailsafeConfig(failsafeConfig_t *failsafeConfigToUse);

void failsafeEnable(void);
void failsafeOnRxCycle(void);
void failsafeCheckPulse(uint8_t channel, uint16_t pulseDuration);
void failsafeUpdateState(void);

void failsafeReset(void);


bool failsafeIsEnabled(void);
bool failsafeIsIdle(void);
bool failsafeHasTimerElapsed(void);
bool failsafeShouldForceLanding(bool armed);
bool failsafeShouldHaveCausedLandingByNow(void);

