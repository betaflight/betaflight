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

#include "flight/pid.h"
#include "fc/rc_controls.h"

#ifdef USE_SERVOS
#include "flight/mixer.h"
#include "flight/servos.h"
#include "io/gimbal.h"
#endif

typedef struct profile_s {
    pidProfile_t pidProfile;

    uint8_t defaultRateProfileIndex;

    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    modeActivationOperator_e modeActivationOperator;

    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];

    // Radio/ESC-related configuration

    rcControlsConfig_t rcControlsConfig;

    uint8_t throttle_tilt_compensation_strength;      // the correction that will be applied at throttle_correction_angle.

#ifdef USE_SERVOS
    // Servo-related stuff
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS]; // servo configuration
    // gimbal-related configuration
    gimbalConfig_t gimbalConfig;

    uint16_t flaperon_throw_offset;
    uint8_t flaperon_throw_inverted;

#endif
} profile_t;
