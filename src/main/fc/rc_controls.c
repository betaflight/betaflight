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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/failsafe.h"

#include "io/gps.h"
#include "io/beeper.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#define AIRMODE_DEADBAND 25
#define MIN_RC_TICK_INTERVAL_MS     20

// true if arming is done via the sticks (as opposed to a switch)
static bool isUsingSticksToArm = true;

// Count of mode activation ranged (per box mode)
static uint8_t specifiedConditionCountPerMode[CHECKBOX_ITEM_COUNT];

#ifdef NAV
// true if pilot has any of GPS modes configured
static bool isUsingNAVModes = false;
#endif

stickPositions_e rcStickPositions;

int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e

PG_REGISTER_WITH_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

PG_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig,
    .deadband = 5,
    .yaw_deadband = 5,
    .pos_hold_deadband = 20,
    .alt_hold_deadband = 50,
    .deadband3d_throttle = 50
);

PG_REGISTER_WITH_RESET_TEMPLATE(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 0);

PG_RESET_TEMPLATE(armingConfig_t, armingConfig,
    .disarm_kill_switch = 1,
    .auto_disarm_delay = 5
);

PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);
PG_REGISTER(modeActivationOperatorConfig_t, modeActivationOperatorConfig, PG_MODE_ACTIVATION_OPERATOR_CONFIG, 0);

bool isUsingSticksForArming(void)
{
    return isUsingSticksToArm;
}

#if defined(NAV)
bool isUsingNavigationModes(void)
{
    return isUsingNAVModes;
}
#endif

bool areSticksInApModePosition(uint16_t ap_mode)
{
    return ABS(rcCommand[ROLL]) < ap_mode && ABS(rcCommand[PITCH]) < ap_mode;
}

throttleStatus_e calculateThrottleStatus(void)
{
    const uint16_t deadband3d_throttle = rcControlsConfig()->deadband3d_throttle;
    if (feature(FEATURE_3D) && (rcData[THROTTLE] > (rxConfig()->midrc - deadband3d_throttle) && rcData[THROTTLE] < (rxConfig()->midrc + deadband3d_throttle)))
        return THROTTLE_LOW;
    else if (!feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig()->mincheck))
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}

rollPitchStatus_e calculateRollPitchCenterStatus(void)
{
    if (((rcData[PITCH] < (rxConfig()->midrc + AIRMODE_DEADBAND)) && (rcData[PITCH] > (rxConfig()->midrc -AIRMODE_DEADBAND)))
            && ((rcData[ROLL] < (rxConfig()->midrc + AIRMODE_DEADBAND)) && (rcData[ROLL] > (rxConfig()->midrc -AIRMODE_DEADBAND))))
        return CENTERED;

    return NOT_CENTERED;
}

stickPositions_e getRcStickPositions(void)
{
    return rcStickPositions;
}

bool checkStickPosition(stickPositions_e stickPos)
{
    const uint8_t mask[4] = { 0x03, 0x0C, 0x30, 0xC0 };
    for (int i = 0; i < 4; i++) {
        if (((stickPos & mask[i]) != 0) && ((stickPos & mask[i]) != (rcStickPositions & mask[i]))) {
            return false;
        }
    }

    return true;
}

static void updateRcStickPositions(void)
{
    stickPositions_e tmp = 0;

    tmp |= ((rcData[ROLL] > rxConfig()->mincheck) ? 0x02 : 0x00) << (ROLL * 2);
    tmp |= ((rcData[ROLL] < rxConfig()->maxcheck) ? 0x01 : 0x00) << (ROLL * 2);

    tmp |= ((rcData[PITCH] > rxConfig()->mincheck) ? 0x02 : 0x00) << (PITCH * 2);
    tmp |= ((rcData[PITCH] < rxConfig()->maxcheck) ? 0x01 : 0x00) << (PITCH * 2);

    tmp |= ((rcData[YAW] > rxConfig()->mincheck) ? 0x02 : 0x00) << (YAW * 2);
    tmp |= ((rcData[YAW] < rxConfig()->maxcheck) ? 0x01 : 0x00) << (YAW * 2);

    tmp |= ((rcData[THROTTLE] > rxConfig()->mincheck) ? 0x02 : 0x00) << (THROTTLE * 2);
    tmp |= ((rcData[THROTTLE] < rxConfig()->maxcheck) ? 0x01 : 0x00) << (THROTTLE * 2);

    rcStickPositions = tmp;
}

void processRcStickPositions(throttleStatus_e throttleStatus, bool disarm_kill_switch, bool fixed_wing_auto_arm)
{
    static timeMs_t lastTickTimeMs = 0;
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint32_t rcSticks;           // this hold sticks position for command combos
    static uint8_t rcDisarmTicks;       // this is an extra guard for disarming through switch to prevent that one frame can disarm it
    const timeMs_t currentTimeMs = millis();

    updateRcStickPositions();

    uint32_t stTmp = getRcStickPositions();
    if (stTmp == rcSticks) {
        if (rcDelayCommand < 250) {
            if ((currentTimeMs - lastTickTimeMs) >= MIN_RC_TICK_INTERVAL_MS) {
                lastTickTimeMs = currentTimeMs;
                rcDelayCommand++;
            }
        }
    } else
        rcDelayCommand = 0;

    rcSticks = stTmp;

    // perform actions
    if (!isUsingSticksToArm) {
        if (IS_RC_MODE_ACTIVE(BOXARM)) {
            rcDisarmTicks = 0;
            // Arming via ARM BOX
            if (throttleStatus == THROTTLE_LOW) {
                if (ARMING_FLAG(OK_TO_ARM)) {
                    mwArm();
                }
            }
        } else {
            // Disarming via ARM BOX
            // Don't disarm via switch if failsafe is active or receiver doesn't receive data - we can't trust receiver
            // and can't afford to risk disarming in the air
            if (ARMING_FLAG(ARMED) && !IS_RC_MODE_ACTIVE(BOXFAILSAFE) && rxIsReceivingSignal() && !failsafeIsActive()) {
                rcDisarmTicks++;
                if (rcDisarmTicks > 3) {    // Wait for at least 3 RX ticks (60ms @ 50Hz RX)
                    if (disarm_kill_switch) {
                        mwDisarm(DISARM_SWITCH);
                    } else if (throttleStatus == THROTTLE_LOW) {
                        mwDisarm(DISARM_SWITCH);
                    }
                }
            }
            else {
                rcDisarmTicks = 0;
            }
        }
    }

    // KILLSWITCH disarms instantly
    if (IS_RC_MODE_ACTIVE(BOXKILLSWITCH)) {
        mwDisarm(DISARM_KILLSWITCH);
    }

    if (rcDelayCommand != 20) {
        return;
    }

   if (isUsingSticksToArm) {
        // Disarm on throttle down + yaw
        if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
            // Dont disarm if fixedwing and motorstop
            if (STATE(FIXED_WING) && feature(FEATURE_MOTOR_STOP) && fixed_wing_auto_arm) {
                return;
            }
            else if (ARMING_FLAG(ARMED)) {
                mwDisarm(DISARM_STICKS);
            }
            else {
                beeper(BEEPER_DISARM_REPEAT);    // sound tone while stick held
                rcDelayCommand = 0;              // reset so disarm tone will repeat
            }
        }
   }

    if (ARMING_FLAG(ARMED)) {
        // actions during armed
        return;
    }

    // actions during not armed
    int i = 0;

    // GYRO calibration
    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
        gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
        return;
    }


#if defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE)
    // Save waypoint list
    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {
        const bool success = saveNonVolatileWaypointList();
        beeper(success ? BEEPER_ACTION_SUCCESS : BEEPER_ACTION_FAIL);
    }

    // Load waypoint list
    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {
        const bool success = loadNonVolatileWaypointList();
        beeper(success ? BEEPER_ACTION_SUCCESS : BEEPER_ACTION_FAIL);
    }
#endif

    // Multiple configuration profiles
    if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
        i = 1;
    else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
        i = 2;
    else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
        i = 3;
    if (i) {
        setConfigProfileAndWriteEEPROM(i - 1);
        return;
    }

    // Save config
    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) {
        saveConfigAndNotify();
    }


    // Arming by sticks
    if (isUsingSticksToArm) {
        if (STATE(FIXED_WING) && feature(FEATURE_MOTOR_STOP) && fixed_wing_auto_arm) {
            // Auto arm on throttle when using fixedwing and motorstop
            if (throttleStatus != THROTTLE_LOW) {
                mwArm();
                return;
            }
        }
        else {
            if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) {
                // Arm via YAW
                mwArm();
                return;
            }
        }
    }


    // Calibrating Acc
    if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
        return;
    }


    // Calibrating Mag
    if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) {
        ENABLE_STATE(CALIBRATE_MAG);
        return;
    }


    // Accelerometer Trim
    if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
        applyAndSaveBoardAlignmentDelta(0, -2);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
        applyAndSaveBoardAlignmentDelta(0, 2);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
        applyAndSaveBoardAlignmentDelta(-2, 0);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
        applyAndSaveBoardAlignmentDelta(2, 0);
        rcDelayCommand = 10;
        return;
    }
}

bool isModeActivationConditionPresent(boxId_e modeId)
{
    for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        if (modeActivationConditions(index)->modeId == modeId && IS_RANGE_USABLE(&modeActivationConditions(index)->range)) {
            return true;
        }
    }

    return false;
}

bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range) {
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }

    uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= 900 + (range->startStep * 25) &&
            channelValue < 900 + (range->endStep * 25));
}

void updateActivatedModes(void)
{
    // Unfortunately for AND logic it's not enough to simply check if any of the specified channel range conditions are valid for a mode.
    // We need to count the total number of conditions specified for each mode, and check that all those conditions are currently valid.

    uint8_t activeConditionCountPerMode[CHECKBOX_ITEM_COUNT];
    memset(activeConditionCountPerMode, 0, CHECKBOX_ITEM_COUNT);

    for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        if (isRangeActive(modeActivationConditions(index)->auxChannelIndex, &modeActivationConditions(index)->range)) {
            // Increment the number of valid conditions for this mode
            activeConditionCountPerMode[modeActivationConditions(index)->modeId]++;
        }
    }

    // Disable all modes to begin with
    rcModeActivationMask = 0;

    // Now see which modes should be enabled
    for (int modeIndex = 0; modeIndex < CHECKBOX_ITEM_COUNT; modeIndex++) {
        // only modes with conditions specified are considered
        if (specifiedConditionCountPerMode[modeIndex] > 0) {
            // For AND logic, the specified condition count and valid condition count must be the same.
            // For OR logic, the valid condition count must be greater than zero.

            if (modeActivationOperatorConfig()->modeActivationOperator == MODE_OPERATOR_AND) {
                // AND the conditions
                if (activeConditionCountPerMode[modeIndex] == specifiedConditionCountPerMode[modeIndex]) {
                    ACTIVATE_RC_MODE(modeIndex);
                }
            }
            else {
                // OR the conditions
                if (activeConditionCountPerMode[modeIndex] > 0) {
                    ACTIVATE_RC_MODE(modeIndex);
                }
            }
        }
    }
}

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {
    return MIN(ABS(rcData[axis] - midrc), 500);
}

void updateUsedModeActivationConditionFlags(void)
{
    memset(specifiedConditionCountPerMode, 0, CHECKBOX_ITEM_COUNT);
    for (int index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        if (IS_RANGE_USABLE(&modeActivationConditions(index)->range)) {
            specifiedConditionCountPerMode[modeActivationConditions(index)->modeId]++;
        }
    }

    isUsingSticksToArm = !isModeActivationConditionPresent(BOXARM);

#ifdef NAV
    isUsingNAVModes = isModeActivationConditionPresent(BOXNAVPOSHOLD) ||
                        isModeActivationConditionPresent(BOXNAVRTH) ||
                        isModeActivationConditionPresent(BOXNAVWP);
#endif
}

void configureModeActivationCondition(int macIndex, boxId_e modeId, uint8_t auxChannelIndex, uint16_t startPwm, uint16_t endPwm)
{
    modeActivationConditionsMutable(macIndex)->modeId = modeId;
    modeActivationConditionsMutable(macIndex)->auxChannelIndex = auxChannelIndex;
    modeActivationConditionsMutable(macIndex)->range.startStep = CHANNEL_VALUE_TO_STEP(startPwm);
    modeActivationConditionsMutable(macIndex)->range.endStep = CHANNEL_VALUE_TO_STEP(endPwm);
}
