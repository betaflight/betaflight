/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_WING_LAUNCH

#include "build/debug.h"

#include "drivers/time.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/wing_launch.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

static wingLaunchState_e launchState = WING_LAUNCH_IDLE;
static timeUs_t stateStartTimeUs = 0;
static bool wasArmed = false;
static float motorOutput = 0.0f;
static float transitionFactor = 0.0f;
static bool throttleGatePassed = false;
static timeDelta_t cachedStateElapsedMs = 0;

// config cache (populated from pidProfile in wingLaunchInit)
static uint16_t motorDelayMs = 100;
static uint16_t motorRampMs = 500;
static float launchThrottle = 0.75f;
static uint16_t climbTimeMs = 3000;
static float climbAngleDeg = 45.0f;
static uint16_t transitionMs = 1000;
static float maxTiltDeg = 45.0f;
static float idleThrottle = 0.0f;
static float accelThreshG = 0.0f;
static float stickOverrideThresh = 0.0f;

static void transitionToState(wingLaunchState_e newState, timeUs_t currentTimeUs)
{
    launchState = newState;
    stateStartTimeUs = currentTimeUs;
}

static timeDelta_t stateElapsedMs(timeUs_t currentTimeUs)
{
    return cmpTimeUs(currentTimeUs, stateStartTimeUs) / 1000;
}

void wingLaunchInit(const pidProfile_t *pidProfile)
{
    motorDelayMs = pidProfile->wing_launch_motor_delay;
    motorRampMs = pidProfile->wing_launch_motor_ramp;
    launchThrottle = pidProfile->wing_launch_throttle / 100.0f;
    climbTimeMs = pidProfile->wing_launch_climb_time;
    climbAngleDeg = (float)pidProfile->wing_launch_climb_angle;
    transitionMs = pidProfile->wing_launch_transition;
    maxTiltDeg = (float)pidProfile->wing_launch_max_tilt;
    idleThrottle = pidProfile->wing_launch_idle_thr / 100.0f;
    accelThreshG = pidProfile->wing_launch_accel_thresh / 10.0f;
    stickOverrideThresh = pidProfile->wing_launch_stick_override / 100.0f * 500.0f;

    launchState = WING_LAUNCH_IDLE;
    stateStartTimeUs = 0;
    wasArmed = false;
    motorOutput = idleThrottle;
    transitionFactor = 0.0f;
    throttleGatePassed = false;
    cachedStateElapsedMs = 0;
}

static void emitDebugAndCache(timeUs_t currentTimeUs)
{
    // Snapshot the current state's elapsed time and broadcast the debug frame.
    // Called at the end of a normal update tick AND before the early-return
    // path when a switch-off/tilt abort has just transitioned to ABORT, so the
    // blackbox log shows the new state on the same frame the transition happens.
    cachedStateElapsedMs = stateElapsedMs(currentTimeUs);
    DEBUG_SET(DEBUG_WING_LAUNCH, 0, launchState);
    DEBUG_SET(DEBUG_WING_LAUNCH, 1, cachedStateElapsedMs);
    DEBUG_SET(DEBUG_WING_LAUNCH, 2, lrintf(motorOutput * 1000));
    DEBUG_SET(DEBUG_WING_LAUNCH, 3, lrintf(acc.accMagnitude * 100));
    DEBUG_SET(DEBUG_WING_LAUNCH, 4, lrintf(attitude.values.pitch));
    DEBUG_SET(DEBUG_WING_LAUNCH, 5, lrintf(attitude.values.roll));
    DEBUG_SET(DEBUG_WING_LAUNCH, 6, lrintf(transitionFactor * 1000));
    DEBUG_SET(DEBUG_WING_LAUNCH, 7, launchState == WING_LAUNCH_CLIMBING
        ? MAX(0, (int32_t)((int32_t)climbTimeMs - (int32_t)cachedStateElapsedMs)) : 0);
}

static bool isStickOverrideActive(void)
{
    if (stickOverrideThresh <= 0.0f) {
        return false;
    }
    return ABS(rcCommand[ROLL]) > stickOverrideThresh
        || ABS(rcCommand[PITCH]) > stickOverrideThresh
        || ABS(rcCommand[YAW]) > stickOverrideThresh;
}

void wingLaunchUpdate(timeUs_t currentTimeUs)
{
    // abort if switch turned off or disarmed mid-launch
    if (!IS_RC_MODE_ACTIVE(BOXAUTOLAUNCH) || !ARMING_FLAG(ARMED)) {
        if (launchState == WING_LAUNCH_DETECTED && ARMING_FLAG(ARMED)) {
            // Pre-throw cancel: pilot turned autolaunch off while still holding
            // the plane. Force-disarm so motors can't follow the throttle stick,
            // and require an arm-switch cycle so a latched arm switch can't
            // immediately re-arm the craft at motor_idle on the next tick.
            disarm(DISARM_REASON_SWITCH);
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        }
        if (launchState != WING_LAUNCH_IDLE
            && launchState != WING_LAUNCH_COMPLETE
            && launchState != WING_LAUNCH_ABORT
            && ARMING_FLAG(ARMED)) {
            // Route mid-launch aborts that occur while still armed
            // (tilt/switch-off) through the ABORT state. The ABORT case latches
            // motors off via the mixer hook in mixer.c and drives the OSD
            // "ABORT" indicator. On disarm we skip this branch and fall
            // straight through to wingLaunchReset() below since the ABORT
            // state would never be observed and BF's disarm path handles
            // motor cut / I-term reset itself.
            motorOutput = 0.0f;
            pidResetIterm();
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            transitionToState(WING_LAUNCH_ABORT, currentTimeUs);
        }
        if (!ARMING_FLAG(ARMED)) {
            wingLaunchReset();
        }
        emitDebugAndCache(currentTimeUs);
        return;
    }

    const float rollAngleDeg = ABS(attitude.values.roll) / 10.0f;
    const float pitchAngleDeg = attitude.values.pitch / 10.0f;

    switch (launchState) {
    case WING_LAUNCH_IDLE:
        if (ARMING_FLAG(ARMED) && !wasArmed) {
            pidResetIterm();
            // always require throw detection -- motors stay idle until accel spike
            transitionToState(WING_LAUNCH_DETECTED, currentTimeUs);
        }
        wasArmed = ARMING_FLAG(ARMED);
        break;

    case WING_LAUNCH_DETECTED:
        // Note: motorOutput is set to idleThrottle here purely for consumers like
        // wingLaunchGetThrottle(). The physical motor pins are forced OFF during
        // DETECTED by the applyMotorStop() hook in mixer.c -- this value is
        // never actually emitted while the pilot is holding the aircraft.
        motorOutput = idleThrottle;
        pidResetIterm(); // prevent I-term windup while aircraft is held in hand
        // Throttle gate: require pilot to raise throttle before throw detection
        // arms. Threshold is intentionally coupled to wing_launch_throttle --
        // holding the stick at launch-power level is the pilot's commitment
        // signal ("I'm ready to throw at the power I've configured"). Raising
        // wing_launch_throttle therefore also raises the gate; a separate
        // gate parameter isn't exposed in order to keep the CLI small.
        if (!throttleGatePassed) {
            const float throttleGatePwm = PWM_RANGE_MIN + launchThrottle * PWM_RANGE;
            if (rcData[THROTTLE] >= throttleGatePwm) {
                throttleGatePassed = true;
            }
        }
        if (throttleGatePassed && acc.accMagnitude > accelThreshG) {
            transitionToState(WING_LAUNCH_MOTOR_DELAY, currentTimeUs);
        }
        // no tilt abort here -- motors are at idle, pilot may hold plane at any angle
        // tilt safety activates in MOTOR_RAMP and CLIMBING when motors are actually live
        break;

    case WING_LAUNCH_MOTOR_DELAY:
        if (stateElapsedMs(currentTimeUs) >= motorDelayMs) {
            motorOutput = idleThrottle;
            transitionToState(WING_LAUNCH_MOTOR_RAMP, currentTimeUs);
        }
        break;

    case WING_LAUNCH_MOTOR_RAMP:
    {
        const timeDelta_t elapsedMs = stateElapsedMs(currentTimeUs);
        if (elapsedMs >= motorRampMs) {
            motorOutput = launchThrottle;
            transitionToState(WING_LAUNCH_CLIMBING, currentTimeUs);
        } else {
            const float rampProgress = (float)elapsedMs / (float)motorRampMs;
            motorOutput = idleThrottle + (launchThrottle - idleThrottle) * rampProgress;
        }
        // safety check during ramp. Tilt-abort takes priority over
        // stick-override so a simultaneous roll/dive + pilot-stick-grab
        // (typical mishap reaction) still routes through ABORT instead of
        // being silently overwritten to TRANSITION on the same tick.
        if (rollAngleDeg > maxTiltDeg || pitchAngleDeg < -maxTiltDeg) {
            transitionToState(WING_LAUNCH_ABORT, currentTimeUs);
        } else if (isStickOverrideActive()) {
            transitionToState(WING_LAUNCH_TRANSITION, currentTimeUs);
        }
        break;
    }

    case WING_LAUNCH_CLIMBING:
        motorOutput = launchThrottle;
        if (stateElapsedMs(currentTimeUs) >= climbTimeMs) {
            transitionToState(WING_LAUNCH_TRANSITION, currentTimeUs);
        }
        // Safety: tilt-abort takes priority over stick-override on the same
        // tick -- see MOTOR_RAMP rationale above.
        if (rollAngleDeg > maxTiltDeg || pitchAngleDeg < -maxTiltDeg) {
            transitionToState(WING_LAUNCH_ABORT, currentTimeUs);
        } else if (isStickOverrideActive()) {
            transitionToState(WING_LAUNCH_TRANSITION, currentTimeUs);
        }
        break;

    case WING_LAUNCH_TRANSITION:
    {
        const timeDelta_t elapsedMs = stateElapsedMs(currentTimeUs);
        if (elapsedMs >= transitionMs) {
            transitionFactor = 1.0f;
            pidResetIterm();
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            transitionToState(WING_LAUNCH_COMPLETE, currentTimeUs);
            // Break out before the tilt check below so a COMPLETE transition
            // isn't overwritten by a same-tick ABORT (motors are handed back
            // to the pilot in COMPLETE; recording a one-tick COMPLETE->ABORT
            // flicker in logs would be confusing).
            break;
        } else {
            transitionFactor = (float)elapsedMs / (float)transitionMs;
            // motorOutput is intentionally NOT re-assigned here: the mixer
            // blend interpolates from wingLaunchGetThrottle() (== motorOutput)
            // to the pilot stick using transitionFactor. Leaving motorOutput
            // at whatever value was carried in from the previous state means
            // a stick-override triggered mid-MOTOR_RAMP (when motorOutput is
            // a partial ramp value) blends smoothly from that partial value
            // to the pilot stick instead of jumping up to launch_throttle.
            // For the normal CLIMBING -> TRANSITION path, motorOutput is
            // already launchThrottle, so behaviour is unchanged.
        }
        // Safety abort: motors are still live at launch_throttle during the
        // blend (up to transition_ms). Match MOTOR_RAMP/CLIMBING and cut them
        // if the aircraft is inverted or dives past max_tilt. Stick-override
        // is intentionally NOT re-checked here: stick-override is the edge
        // that got us into TRANSITION in the first place, so re-checking it
        // would short-circuit the blend the pilot just triggered.
        if (rollAngleDeg > maxTiltDeg || pitchAngleDeg < -maxTiltDeg) {
            transitionToState(WING_LAUNCH_ABORT, currentTimeUs);
        }
        break;
    }

    case WING_LAUNCH_ABORT:
        // Latch in ABORT until the pilot disarms. Motors are held off by the
        // applyMotorStop() hook in mixer.c while state == ABORT, so a
        // tilt/switch-off abort cannot hand pilot-stick throttle back to the
        // motors mid-failure. wingLaunchReset() returns to IDLE on disarm.
        motorOutput = 0.0f;
        pidResetIterm();
        DISABLE_FLIGHT_MODE(ANGLE_MODE);
        break;

    case WING_LAUNCH_COMPLETE:
    default:
        break;
    }

    emitDebugAndCache(currentTimeUs);
}

void wingLaunchReset(void)
{
    launchState = WING_LAUNCH_IDLE;
    wasArmed = false;
    motorOutput = idleThrottle;
    transitionFactor = 0.0f;
    throttleGatePassed = false;
    cachedStateElapsedMs = 0;
}

bool isWingLaunchActive(void)
{
    return launchState >= WING_LAUNCH_DETECTED && launchState <= WING_LAUNCH_CLIMBING;
}

bool isWingLaunchInProgress(void)
{
    return launchState >= WING_LAUNCH_DETECTED && launchState <= WING_LAUNCH_TRANSITION;
}

float wingLaunchGetThrottle(void)
{
    switch (launchState) {
    case WING_LAUNCH_IDLE:
    case WING_LAUNCH_DETECTED:
    case WING_LAUNCH_MOTOR_DELAY:
        return idleThrottle;
    case WING_LAUNCH_MOTOR_RAMP:
    case WING_LAUNCH_CLIMBING:
    case WING_LAUNCH_TRANSITION:
        return motorOutput;
    default:
        return -1.0f;
    }
}

float wingLaunchGetPitchAngle(void)
{
    // Keep elevator neutral while plane is in pilot's hand (IDLE/DETECTED).
    // Only command climb attitude once a throw has been detected and the
    // launch has progressed into MOTOR_DELAY or later.
    if (launchState == WING_LAUNCH_TRANSITION) {
        return climbAngleDeg * (1.0f - transitionFactor);
    }
    if (launchState == WING_LAUNCH_MOTOR_DELAY
        || launchState == WING_LAUNCH_MOTOR_RAMP
        || launchState == WING_LAUNCH_CLIMBING) {
        return climbAngleDeg;
    }
    return 0.0f;
}

float wingLaunchGetTransitionFactor(void)
{
    return transitionFactor;
}

int32_t wingLaunchGetClimbTimeRemainingMs(void)
{
    // Use the state-machine-cached elapsed from the last wingLaunchUpdate() so
    // the OSD countdown stays in lockstep with state transitions instead of
    // drifting by one update interval off an independent micros() read.
    if (launchState == WING_LAUNCH_CLIMBING) {
        const int32_t remaining = (int32_t)climbTimeMs - (int32_t)cachedStateElapsedMs;
        return (remaining > 0) ? remaining : 0;
    }
    return 0;
}

wingLaunchState_e wingLaunchGetState(void)
{
    return launchState;
}

bool wingLaunchIsThrottleGatePassed(void)
{
    return throttleGatePassed;
}

#endif // USE_WING_LAUNCH
