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

#include "rx/rx.h"

typedef enum {
    BOXARM = 0,
    BOXANGLE,
    BOXHORIZON,
    BOXNAVALTHOLD,  // old BOXBARO
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXNAVRTH,      // old GPSHOME
    BOXNAVPOSHOLD,  // old GPSHOLD
    BOXPASSTHRU,
    BOXBEEPERON,
    BOXLEDLOW,
    BOXLLIGHTS,
    BOXNAVLAUNCH,
    BOXOSD,
    BOXTELEMETRY,
    BOXBLACKBOX,
    BOXFAILSAFE,
    BOXNAVWP,
    BOXAIRMODE,
    BOXHOMERESET,
    BOXGCSNAV,
    BOXHEADINGLOCK,
    BOXSURFACE,
    BOXFLAPERON,
    BOXTURNASSIST,
    BOXAUTOTRIM,
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

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 20

#define CHANNEL_RANGE_MIN 900
#define CHANNEL_RANGE_MAX 2100

#define MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_STEP(channelValue) ((constrain(channelValue, CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / 25)

#define MIN_MODE_RANGE_STEP 0
#define MAX_MODE_RANGE_STEP ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / 25)

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

typedef enum {
    MODE_OPERATOR_OR,
    MODE_OPERATOR_AND
} modeActivationOperator_e;

typedef struct controlRateConfig_s {
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint8_t rcYawExpo8;
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
} controlRateConfig_t;

extern int16_t rcCommand[4];

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t pos_hold_deadband;             // Adds ability to adjust the Hold-position when moving the sticks (assisted mode)
    uint8_t alt_hold_deadband;             // Defines the neutral zone of throttle stick during altitude hold
} rcControlsConfig_t;

typedef struct armingConfig_s {
    uint8_t fixed_wing_auto_arm;            // Auto-arm fixed wing aircraft on throttle up and never disarm
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
} armingConfig_t;

bool areUsingSticksToArm(void);

bool areSticksInApModePosition(uint16_t ap_mode);
throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle);
rollPitchStatus_e calculateRollPitchCenterStatus(rxConfig_t *rxConfig);
void processRcStickPositions(rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool disarm_kill_switch, bool fixed_wing_auto_arm);

void updateActivatedModes(modeActivationCondition_t *modeActivationConditions, modeActivationOperator_e modeActivationOperator);


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

} adjustmentFunction_e;

#define ADJUSTMENT_FUNCTION_COUNT 21

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
    const adjustmentConfig_t *config;
    uint32_t timeoutAt;
} adjustmentState_t;


#ifndef MAX_SIMULTANEOUS_ADJUSTMENT_COUNT
#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 4 // enough for 4 x 3position switches / 4 aux channel
#endif

#define MAX_ADJUSTMENT_RANGE_COUNT 12 // enough for 2 * 6pos switches.

void resetAdjustmentStates(void);
void configureAdjustment(uint8_t index, uint8_t auxChannelIndex, const adjustmentConfig_t *adjustmentConfig);
void updateAdjustmentStates(adjustmentRange_t *adjustmentRanges);
void processRcAdjustments(controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig);

bool isUsingSticksForArming(void);
bool isUsingNavigationModes(void);

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc);
bool isModeActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId);
struct motorConfig_s;
struct pidProfile_s;
void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, struct motorConfig_s *motorConfigToUse, struct pidProfile_s *pidProfileToUse);
