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

#include "config/parameter_group.h"

typedef enum {
    BOXARM          = 0,
    BOXANGLE        = 1,
    BOXHORIZON      = 2,
    BOXNAVALTHOLD   = 3,    // old BOXBARO
    BOXHEADINGHOLD  = 4,    // old MAG
    BOXHEADFREE     = 5,
    BOXHEADADJ      = 6,
    BOXCAMSTAB      = 7,
    BOXNAVRTH       = 8,    // old GPSHOME
    BOXNAVPOSHOLD   = 9,    // old GPSHOLD
    BOXPASSTHRU     = 10,
    BOXBEEPERON     = 11,
    BOXLEDLOW       = 12,
    BOXLLIGHTS      = 13,
    BOXNAVLAUNCH    = 14,
    BOXOSD          = 15,
    BOXTELEMETRY    = 16,
    BOXBLACKBOX     = 17,
    BOXFAILSAFE     = 18,
    BOXNAVWP        = 19,
    BOXAIRMODE      = 20,
    BOXHOMERESET    = 21,
    BOXGCSNAV       = 22,
    BOXKILLSWITCH   = 23,   // old HEADING LOCK
    BOXSURFACE      = 24,
    BOXFLAPERON     = 25,
    BOXTURNASSIST   = 26,
    BOXAUTOTRIM     = 27,
    BOXAUTOTUNE     = 28,
    CHECKBOX_ITEM_COUNT
} boxId_e;

extern uint32_t rcModeActivationMask;

#define IS_RC_MODE_ACTIVE(modeId) ((1 << (modeId)) & rcModeActivationMask)
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

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

typedef enum {
    ROL_LO = (1 << (2 * ROLL)),
    ROL_CE = (3 << (2 * ROLL)),
    ROL_HI = (2 << (2 * ROLL)),

    PIT_LO = (1 << (2 * PITCH)),
    PIT_CE = (3 << (2 * PITCH)),
    PIT_HI = (2 << (2 * PITCH)),

    YAW_LO = (1 << (2 * YAW)),
    YAW_CE = (3 << (2 * YAW)),
    YAW_HI = (2 << (2 * YAW)),

    THR_LO = (1 << (2 * THROTTLE)),
    THR_CE = (3 << (2 * THROTTLE)),
    THR_HI = (2 << (2 * THROTTLE))
} stickPositions_e;

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 20

#define CHANNEL_RANGE_MIN 900
#define CHANNEL_RANGE_MAX 2100

#define MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_STEP(channelValue) ((constrain(channelValue, CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / 25)

#define MIN_MODE_RANGE_STEP 0
#define MAX_MODE_RANGE_STEP ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / 25)


// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 1200
typedef struct channelRange_s {
    uint8_t startStep;
    uint8_t endStep;
} channelRange_t;

typedef struct modeActivationCondition_s {
    boxId_e modeId;
    uint8_t auxChannelIndex;
    channelRange_t range;
} modeActivationCondition_t;

#define IS_RANGE_USABLE(range) ((range)->startStep < (range)->endStep)

PG_DECLARE_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions);

typedef enum {
    MODE_OPERATOR_OR, // default
    MODE_OPERATOR_AND
} modeActivationOperator_e;

typedef struct modeActivationOperatorConfig_s {
    modeActivationOperator_e modeActivationOperator;
} modeActivationOperatorConfig_t;

PG_DECLARE(modeActivationOperatorConfig_t, modeActivationOperatorConfig);

extern int16_t rcCommand[4];

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t pos_hold_deadband;              // Adds ability to adjust the Hold-position when moving the sticks (assisted mode)
    uint8_t alt_hold_deadband;              // Defines the neutral zone of throttle stick during altitude hold
    uint16_t deadband3d_throttle;           // default throttle deadband from MIDRC
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

typedef struct armingConfig_s {
    uint8_t fixed_wing_auto_arm;            // Auto-arm fixed wing aircraft on throttle up and never disarm
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

stickPositions_e getRcStickPositions(void);
bool checkStickPosition(stickPositions_e stickPos);

bool areUsingSticksToArm(void);

bool areSticksInApModePosition(uint16_t ap_mode);
throttleStatus_e calculateThrottleStatus(void);
rollPitchStatus_e calculateRollPitchCenterStatus(void);
void processRcStickPositions(throttleStatus_e throttleStatus, bool disarm_kill_switch, bool fixed_wing_auto_arm);

bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range);
void updateActivatedModes(void);

bool isUsingSticksForArming(void);
bool isUsingNavigationModes(void);

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc);
bool isModeActivationConditionPresent(boxId_e modeId);
void updateUsedModeActivationConditionFlags(void);

void configureModeActivationCondition(int macIndex, boxId_e modeId, uint8_t auxChannelIndex, uint16_t startPwm, uint16_t endPwm);