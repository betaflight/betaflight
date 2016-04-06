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

#define PID_MIN      0
#define PID_MAX      200
#define PID_F_MIN    0
#define PID_F_MAX    100
#define RC_RATE_MIN  0
#define RC_RATE_MAX  250
#define EXPO_MIN     0
#define EXPO_MAX     100

typedef enum {
    ADJUSTMENT_NONE = 0,
    ADJUSTMENT_RC_RATE,
    ADJUSTMENT_RC_EXPO,
    ADJUSTMENT_THROTTLE_EXPO,
    ADJUSTMENT_PITCH_ROLL_RATE,
    ADJUSTMENT_YAW_RATE,
    ADJUSTMENT_PITCH_ROLL_P,
    ADJUSTMENT_PITCH_ROLL_I,
    ADJUSTMENT_PITCH_ROLL_D,
    ADJUSTMENT_YAW_P,
    ADJUSTMENT_YAW_I,
    ADJUSTMENT_YAW_D,
    ADJUSTMENT_RATE_PROFILE,
    ADJUSTMENT_PITCH_RATE,
    ADJUSTMENT_ROLL_RATE,
    ADJUSTMENT_PITCH_P,
    ADJUSTMENT_PITCH_I,
    ADJUSTMENT_PITCH_D,
    ADJUSTMENT_ROLL_P,
    ADJUSTMENT_ROLL_I,
    ADJUSTMENT_ROLL_D,
    ADJUSTMENT_ALT_P,
    ADJUSTMENT_ALT_I,
    ADJUSTMENT_ALT_D,
    ADJUSTMENT_VEL_P,
    ADJUSTMENT_VEL_I,
    ADJUSTMENT_VEL_D,
    ADJUSTMENT_MAG_P,
    ADJUSTMENT_POS_P,
    ADJUSTMENT_POS_I,
    ADJUSTMENT_POSR_P,
    ADJUSTMENT_POSR_I,
    ADJUSTMENT_POSR_D,
    ADJUSTMENT_NAVR_P,
    ADJUSTMENT_NAVR_I,
    ADJUSTMENT_NAVR_D,
    ADJUSTMENT_LEVEL_P,
    ADJUSTMENT_LEVEL_I,
    ADJUSTMENT_LEVEL_D,

} adjustmentFunction_e;

#define ADJUSTMENT_FUNCTION_COUNT 39

typedef enum {
    ADJUSTMENT_MODE_STEP,
    ADJUSTMENT_MODE_SELECT
} adjustmentMode_e;

typedef struct adjustmentStepConfig_s {
    uint8_t step;
} adjustmentStepConfig_t;

typedef struct adjustmentSelectConfig_s {
    uint8_t switchPositions;
} adjustmentSelectConfig_t;

typedef union adjustmentConfig_u {
    adjustmentStepConfig_t stepConfig;
    adjustmentSelectConfig_t selectConfig;
} adjustmentData_t;

typedef struct adjustmentConfig_s {
    uint8_t adjustmentFunction;
    uint8_t mode;
    adjustmentData_t data;
} adjustmentConfig_t;

typedef struct adjustmentRange_s {
    // when aux channel is in range...
    uint8_t auxChannelIndex;
    channelRange_t range;

    // ..then apply the adjustment function to the auxSwitchChannel ...
    uint8_t adjustmentFunction;
    uint8_t auxSwitchChannelIndex;

    // ... via slot
    uint8_t adjustmentIndex;
} adjustmentRange_t;

#define ADJUSTMENT_INDEX_OFFSET 1

typedef struct adjustmentState_s {
    uint8_t auxChannelIndex;
    adjustmentConfig_t config;
    uint32_t timeoutAt;
    adjustmentRange_t *range;
} adjustmentState_t;

#define MAX_ADJUSTMENT_RANGE_COUNT 12 // enough for 2 * 6pos switches.

typedef struct adjustmentProfile_s {
    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];
} adjustmentProfile_t;

PG_DECLARE_PROFILE(adjustmentProfile_t, adjustmentProfile);

#ifndef MAX_SIMULTANEOUS_ADJUSTMENT_COUNT
#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 4 // enough for 4 x 3position switches / 4 aux channel
#endif


void resetAdjustmentStates(void);
void configureAdjustmentState(adjustmentRange_t *adjustmentRange);
void updateAdjustmentStates(adjustmentRange_t *adjustmentRanges);
void processRcAdjustments(controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig);
