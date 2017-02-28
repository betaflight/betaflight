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

#include <stdint.h>

#include "config/parameter_group.h"

#define MAX_CONTROL_RATE_PROFILE_COUNT 3
/*
Max and min available values for rates are now stored as absolute
tenths of degrees-per-second [dsp/10]
That means, max. rotation rate 180 equals 1800dps

New defaults of 200dps for pitch,roll and yaw are more less
equivalent of rates 0 from previous versions of iNav, Cleanflight, Baseflight
and so on.
*/
#define CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX  180
#define CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MIN  6
#define CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_DEFAULT  20
#define CONTROL_RATE_CONFIG_YAW_RATE_MAX         180
#define CONTROL_RATE_CONFIG_YAW_RATE_MIN         2
#define CONTROL_RATE_CONFIG_YAW_RATE_DEFAULT     20

#define CONTROL_RATE_CONFIG_TPA_MAX              100

typedef struct controlRateConfig_s {
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint8_t rcYawExpo8;
} controlRateConfig_t;

PG_DECLARE_ARRAY(controlRateConfig_t, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);

extern const controlRateConfig_t *currentControlRateProfile;

void setControlRateProfile(uint8_t profileIndex);
void changeControlRateProfile(uint8_t profileIndex);
void activateControlRateConfig(void);
uint8_t getCurrentControlRateProfile(void);
