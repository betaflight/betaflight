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
    BOXARM = 0,
    BOXANGLE,
    BOXHORIZON,
    BOXBARO,
    // BOXVARIO,
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXPASSTHRU,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLEDLOW,
    BOXLLIGHTS,
    BOXCALIB,
    BOXGOV,
    BOXOSD,
    BOXTELEMETRY,
    BOXAUTOTUNE,
    BOXSONAR,
    CHECKBOX_ITEM_COUNT
} boxId_e;

extern uint32_t rcModeActivationMask;

#define IS_RC_MODE_ACTIVE(modeId) ((1 << modeId) & rcModeActivationMask)
#define ACTIVATE_RC_MODE(modeId) (rcModeActivationMask |= (1 << modeId))

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8
} rc_alias_e;

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 40
// 40 is enough for 1 mode for each position of 11 * 3 position switches and a 6 pos switch.
// however, that is unlikely because you don't define the 'off' positions, so for a 3 position
// switch it's normal that only 2 values would be configured.
// this leaves plenty of 'slots' free for cases where you enable multiple modes for a switch
// position (like gps rth + horizon + baro + beeper)

#define CHANNEL_RANGE_MIN 900
#define CHANNEL_RANGE_MAX 2100

#define MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_STEP(channelValue) ((constrain(channelValue, CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / 25)

#define MIN_MODE_RANGE_STEP 0
#define MAX_MODE_RANGE_STEP ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / 25)

typedef struct modeActivationCondition_s {
    boxId_e modeId;
    uint8_t auxChannelIndex;

    // steps are 25 apart
    // a value of 0 corresponds to a channel value of 900 or less
    // a value of 48 corresponds to a channel value of 2100 or more
    // 48 steps between 900 and 1200
    uint8_t rangeStartStep;
    uint8_t rangeEndStep;
} modeActivationCondition_t;

#define IS_MODE_RANGE_USABLE(modeActivationCondition) (modeActivationCondition->rangeStartStep < modeActivationCondition->rangeEndStep)

typedef struct controlRateConfig_s {
    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rollPitchRate;
    uint8_t yawRate;
} controlRateConfig_t;

extern int16_t rcCommand[4];

bool areSticksInApModePosition(uint16_t ap_mode);
throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle);
void processRcStickPositions(rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool retarded_arm, bool disarm_kill_switch);


void updateActivatedModes(modeActivationCondition_t *modeActivationConditions);
void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions);
