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

#include <stdbool.h>

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
    BOXGTUNE,
    BOXSONAR,
    BOXSERVO1,
    BOXSERVO2,
    BOXSERVO3,
    BOXBLACKBOX,
    BOXFAILSAFE,
    BOXAIRMODE,
    BOX3DDISABLESWITCH,
    BOXFPVANGLEMIX,
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

#define AIRMODEDEADBAND 12

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

typedef enum {
    RC_SMOOTHING_OFF = 0,
    RC_SMOOTHING_DEFAULT,
    RC_SMOOTHING_AUTO,
    RC_SMOOTHING_MANUAL
} rcSmoothing_t;

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

// Roll/pitch rates are a proportion used for mixing, so it tops out at 1.0:
#define CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX  100

/* Meaningful yaw rates are effectively unbounded because they are treated as a rotation rate multiplier: */
#define CONTROL_RATE_CONFIG_YAW_RATE_MAX         255

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

typedef struct modeActivationProfile_s {
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
} modeActivationProfile_t;

#define IS_RANGE_USABLE(range) ((range)->startStep < (range)->endStep)

typedef struct controlRateConfig_s {
    uint8_t rcRate8;
    uint8_t rcYawRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint8_t rcYawExpo8;
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
} controlRateConfig_t;

extern int16_t rcCommand[4];
extern int16_t rcCommandSmooth[4];

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_deadband;              // defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
    uint8_t alt_hold_fast_change;           // when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
    int8_t yaw_control_direction;           // change control direction of yaw (inverted, normal)
} rcControlsConfig_t;

typedef struct armingConfig_s {
    uint8_t gyro_cal_on_first_arm;          // allow disarm/arm on throttle down + roll left/right
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
} armingConfig_t;

bool areUsingSticksToArm(void);

bool areSticksInApModePosition(uint16_t ap_mode);
struct rxConfig_s;
throttleStatus_e calculateThrottleStatus(struct rxConfig_s *rxConfig, uint16_t deadband3d_throttle);
void processRcStickPositions(struct rxConfig_s *rxConfig, throttleStatus_e throttleStatus, bool disarm_kill_switch);

bool isRangeActive(uint8_t auxChannelIndex, channelRange_t *range);
void updateActivatedModes(modeActivationCondition_t *modeActivationConditions);


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
    ADJUSTMENT_RC_RATE_YAW,
    ADJUSTMENT_D_SETPOINT,
    ADJUSTMENT_D_SETPOINT_TRANSITION,
    ADJUSTMENT_FUNCTION_COUNT
} adjustmentFunction_e;


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

#define MAX_ADJUSTMENT_RANGE_COUNT 15

typedef struct adjustmentProfile_s {
    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];
} adjustmentProfile_t;

bool isAirmodeActive(void);
void resetAdjustmentStates(void);
void updateAdjustmentStates(adjustmentRange_t *adjustmentRanges);
void processRcAdjustments(controlRateConfig_t *controlRateConfig, struct rxConfig_s *rxConfig);

bool isUsingSticksForArming(void);

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc);
bool isModeActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId);
struct pidProfile_s;
struct motorConfig_s;
void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, struct motorConfig_s *motorConfigToUse, struct pidProfile_s *pidProfileToUse);
