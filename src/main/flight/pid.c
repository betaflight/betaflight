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
#include <math.h>

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/gps.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/pitotmeter.h"


typedef struct {
    float kP;   // Proportional gain
    float kI;   // Integral gain
    float kD;   // Derivative gain
    float kFF;  // Feed-forward gain
    float kT;   // Back-calculation tracking gain

    float gyroRate;
    float rateTarget;

    // Buffer for derivative calculation
#define PID_GYRO_RATE_BUF_LENGTH 5
    float gyroRateBuf[PID_GYRO_RATE_BUF_LENGTH];
    firFilter_t gyroRateFilter;

    // Rate integrator
    float errorGyroIf;
    float errorGyroIfLimit;

    // Used for ANGLE filtering
    pt1Filter_t angleFilterState;

    // Rate filtering
    rateLimitFilter_t axisAccelFilter;
    pt1Filter_t ptermLpfState;
    pt1Filter_t deltaLpfState;

    // Dterm notch filtering
#ifdef USE_DTERM_NOTCH
    biquadFilter_t deltaNotchFilter;
#endif
} pidState_t;

#ifdef USE_DTERM_NOTCH
    static filterApplyFnPtr notchFilterApplyFn;
#endif

extern float dT;

float headingHoldCosZLimit;
int16_t headingHoldTarget;
static pt1Filter_t headingHoldRateFilter;

// Thrust PID Attenuation factor. 0.0f means fully attenuated, 1.0f no attenuation is applied
static bool pidGainsUpdateRequired = false;
int16_t axisPID[FLIGHT_DYNAMICS_INDEX_COUNT];

#ifdef BLACKBOX
int32_t axisPID_P[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_I[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_D[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_Setpoint[FLIGHT_DYNAMICS_INDEX_COUNT];
#endif

static pidState_t pidState[FLIGHT_DYNAMICS_INDEX_COUNT];

PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(pidProfile_t, pidProfile, PG_PID_PROFILE, 3);

PG_RESET_TEMPLATE(pidProfile_t, pidProfile,
        .bank_mc = {
            .pid = {
                [PID_ROLL] =    { 40, 30, 23 },
                [PID_PITCH] =   { 40, 30, 23 },
                [PID_YAW] =     { 85, 45, 0 },
                [PID_LEVEL] = {
                    .P = 20,    // Self-level strength
                    .I = 15,    // Self-leveing low-pass frequency (0 - disabled)
                    .D = 75,    // 75% horizon strength
                },
                [PID_HEADING] = { 60, 0, 0 },
                [PID_POS_XY] = {
                    .P = 65,   // NAV_POS_XY_P * 100
                    .I = 120,  // posDecelerationTime * 100
                    .D = 10,   // posResponseExpo * 100
                },
                [PID_VEL_XY] = {
                    .P = 180,  // NAV_VEL_XY_P * 100
                    .I = 15,   // NAV_VEL_XY_I * 100
                    .D = 100,  // NAV_VEL_XY_D * 100
                },
                [PID_POS_Z] = {
                    .P = 50,    // NAV_POS_Z_P * 100
                    .I = 0,     // not used
                    .D = 0,     // not used
                },
                [PID_VEL_Z] = {
                    .P = 100,   // NAV_VEL_Z_P * 100
                    .I = 50,    // NAV_VEL_Z_I * 100
                    .D = 10,    // NAV_VEL_Z_D * 100
                }
            }
        },

        .bank_fw = {
            .pid = {
                [PID_ROLL] =    { 5, 7, 50 },
                [PID_PITCH] =   { 5, 7, 50 },
                [PID_YAW] =     { 6, 10, 60 },
                [PID_LEVEL] = {
                    .P = 20,    // Self-level strength
                    .I = 5,     // Self-leveing low-pass frequency (0 - disabled)
                    .D = 75,    // 75% horizon strength
                },
                [PID_HEADING] = { 60, 0, 0 },
                [PID_POS_Z] = {
                    .P = 50,    // FW_POS_Z_P * 100
                    .I = 0,     // not used
                    .D = 0,     // not used
                },
                [PID_POS_XY] = {
                    .P = 75,     // FW_NAV_P * 100
                    .I = 5,      // FW_NAV_I * 100
                    .D = 8,      // FW_NAV_D * 100
                }
            }
        },

        .acc_soft_lpf_hz = 15,
        .dterm_soft_notch_hz = 0,
        .dterm_soft_notch_cutoff = 1,
        .dterm_lpf_hz = 40,
        .yaw_lpf_hz = 30,
        .dterm_setpoint_weight = 0.0f,

        .rollPitchItermIgnoreRate = 200,     // dps
        .yawItermIgnoreRate = 50,            // dps

        .axisAccelerationLimitYaw = 10000,       // dps/s
        .axisAccelerationLimitRollPitch = 0,     // dps/s

        .yaw_p_limit = YAW_P_LIMIT_DEFAULT,

        .heading_hold_rate_limit = HEADING_HOLD_RATE_LIMIT_DEFAULT,

        .max_angle_inclination[FD_ROLL] = 300,    // 30 degrees
        .max_angle_inclination[FD_PITCH] = 300,    // 30 degrees
        .pidSumLimit = PID_SUM_LIMIT_DEFAULT,

        .fixedWingItermThrowLimit = FW_ITERM_THROW_LIMIT_DEFAULT,
        .fixedWingReferenceAirspeed = 1000,
        .fixedWingCoordinatedYawGain = 1.0f,
);

void pidInit(void)
{
    // Calculate derivative using 5-point noise-robust differentiators without time delay (one-sided or forward filters)
    // by Pavel Holoborodko, see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
    // h[0] = 5/8, h[-1] = 1/4, h[-2] = -1, h[-3] = -1/4, h[-4] = 3/8
    static const float dtermCoeffs[PID_GYRO_RATE_BUF_LENGTH] = {5.0f/8, 2.0f/8, -8.0f/8, -2.0f/8, 3.0f/8};
    for (int axis = 0; axis < 3; ++ axis) {
        firFilterInit(&pidState[axis].gyroRateFilter, pidState[axis].gyroRateBuf, PID_GYRO_RATE_BUF_LENGTH, dtermCoeffs);
    }

    // Calculate max overall tilt (max pitch + max roll combined) as a limit to heading hold
    headingHoldCosZLimit = cos_approx(DECIDEGREES_TO_RADIANS(pidProfile()->max_angle_inclination[FD_ROLL])) *
                           cos_approx(DECIDEGREES_TO_RADIANS(pidProfile()->max_angle_inclination[FD_PITCH]));
}

#ifdef USE_DTERM_NOTCH
bool pidInitFilters(void)
{
    const uint32_t refreshRate = getPidUpdateRate();
    notchFilterApplyFn = nullFilterApply;
    if (refreshRate != 0 && pidProfile()->dterm_soft_notch_hz != 0){
        notchFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < 3; ++ axis) {
            biquadFilterInitNotch(&pidState[axis].deltaNotchFilter, refreshRate, pidProfile()->dterm_soft_notch_hz, pidProfile()->dterm_soft_notch_cutoff);
        }
        return true;
    }
    return false;
}
#endif

void pidResetErrorAccumulators(void)
{
    // Reset R/P/Y integrator
    for (int axis = 0; axis < 3; axis++) {
        pidState[axis].errorGyroIf = 0.0f;
        pidState[axis].errorGyroIfLimit = 0.0f;
    }
}

static float pidRcCommandToAngle(int16_t stick, int16_t maxInclination)
{
    stick = constrain(stick, -500, 500);
    return scaleRangef((float) stick, -500.0f, 500.0f, (float) -maxInclination, (float) maxInclination);
}

int16_t pidAngleToRcCommand(float angleDeciDegrees, int16_t maxInclination)
{
    angleDeciDegrees = constrainf(angleDeciDegrees, (float) -maxInclination, (float) maxInclination);
    return scaleRangef((float) angleDeciDegrees, (float) -maxInclination, (float) maxInclination, -500.0f, 500.0f);
}

/*
Map stick positions to desired rotatrion rate in given axis.
Rotation rate in dps at full stick deflection is defined by axis rate measured in dps/10
Rate 20 means 200dps at full stick deflection
*/
float pidRateToRcCommand(float rateDPS, uint8_t rate)
{
    const float maxRateDPS = rate * 10.0f;
    return scaleRangef(rateDPS, -maxRateDPS, maxRateDPS, -500.0f, 500.0f);
}

float pidRcCommandToRate(int16_t stick, uint8_t rate)
{
    const float maxRateDPS = rate * 10.0f;
    return scaleRangef((float) stick, -500.0f, 500.0f, -maxRateDPS, maxRateDPS);
}

static float calculateFixedWingTPAFactor(void)
{
    float tpaFactor;

    // tpa_rate is amount of curve TPA applied to PIDs
    // tpa_breakpoint for fixed wing is cruise throttle value (value at which PIDs were tuned)
    if (currentControlRateProfile->dynThrPID != 0 && currentControlRateProfile->tpa_breakpoint > motorConfig()->minthrottle) {
        if (rcCommand[THROTTLE] > motorConfig()->minthrottle) {
            // Calculate TPA according to throttle
            tpaFactor = 0.5f + ((float)(currentControlRateProfile->tpa_breakpoint - motorConfig()->minthrottle) / (rcCommand[THROTTLE] - motorConfig()->minthrottle) / 2.0f);

            // Limit to [0.5; 2] range
            tpaFactor = constrainf(tpaFactor, 0.5f, 2.0f);
        }
        else {
            tpaFactor = 2.0f;
        }

        // Attenuate TPA curve according to configured amount
        tpaFactor = 1.0f + (tpaFactor - 1.0f) * (currentControlRateProfile->dynThrPID / 100.0f);
    }
    else {
        tpaFactor = 1.0f;
    }

    return tpaFactor;
}

static float calculateMultirotorTPAFactor(void)
{
    float tpaFactor;

    // TPA should be updated only when TPA is actually set
    if (currentControlRateProfile->dynThrPID == 0 || rcCommand[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        tpaFactor = 1.0f;
    } else if (rcCommand[THROTTLE] < motorConfig()->maxthrottle) {
        tpaFactor = (100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcCommand[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (motorConfig()->maxthrottle - currentControlRateProfile->tpa_breakpoint)) / 100.0f;
    } else {
        tpaFactor = (100 - currentControlRateProfile->dynThrPID) / 100.0f;
    }

    return tpaFactor;
}

void schedulePidGainsUpdate(void)
{
    pidGainsUpdateRequired = true;
}

void updatePIDCoefficients(void)
{
    static uint16_t prevThrottle = 0;

    // Check if throttle changed
    if (rcCommand[THROTTLE] != prevThrottle) {
        prevThrottle = rcCommand[THROTTLE];
        pidGainsUpdateRequired = true;
    }

    // If nothing changed - don't waste time recalculating coefficients
    if (!pidGainsUpdateRequired) {
        return;
    }

    const float tpaFactor = STATE(FIXED_WING) ? calculateFixedWingTPAFactor() : calculateMultirotorTPAFactor();

    // PID coefficients can be update only with THROTTLE and TPA or inflight PID adjustments
    //TODO: Next step would be to update those only at THROTTLE or inflight adjustments change
    for (int axis = 0; axis < 3; axis++) {
        if (STATE(FIXED_WING)) {
            // Airplanes - scale all PIDs according to TPA
            pidState[axis].kP  = pidBank()->pid[axis].P / FP_PID_RATE_P_MULTIPLIER  * tpaFactor;
            pidState[axis].kI  = pidBank()->pid[axis].I / FP_PID_RATE_I_MULTIPLIER  * tpaFactor;
            pidState[axis].kD  = 0.0f;
            pidState[axis].kFF = pidBank()->pid[axis].D / FP_PID_RATE_FF_MULTIPLIER * tpaFactor;
            pidState[axis].kT  = 0.0f;
        }
        else {
            const float axisTPA = (axis == FD_YAW) ? 1.0f : tpaFactor;
            pidState[axis].kP  = pidBank()->pid[axis].P / FP_PID_RATE_P_MULTIPLIER * axisTPA;
            pidState[axis].kI  = pidBank()->pid[axis].I / FP_PID_RATE_I_MULTIPLIER;
            pidState[axis].kD  = pidBank()->pid[axis].D / FP_PID_RATE_D_MULTIPLIER * axisTPA;
            pidState[axis].kFF = 0.0f;

            // Tracking anti-windup requires P/I/D to be all defined which is only true for MC
            if ((pidBank()->pid[axis].P != 0) && (pidBank()->pid[axis].I != 0)) {
                pidState[axis].kT = 2.0f / ((pidState[axis].kP / pidState[axis].kI) + (pidState[axis].kD / pidState[axis].kP));
            } else {
                pidState[axis].kT = 0;
            }
        }
    }

    pidGainsUpdateRequired = false;
}

static float calcHorizonRateMagnitude(void)
{
    // Figure out the raw stick positions
    const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, rxConfig()->midrc));
    const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, rxConfig()->midrc));
    const float mostDeflectedStickPos = constrain(MAX(stickPosAil, stickPosEle), 0, 500) / 500.0f;
    const float modeTransitionStickPos = constrain(pidBank()->pid[PID_LEVEL].D, 0, 100) / 100.0f;

    float horizonRateMagnitude;

    // Calculate transition point according to stick deflection
    if (mostDeflectedStickPos <= modeTransitionStickPos) {
        horizonRateMagnitude = mostDeflectedStickPos / modeTransitionStickPos;
    }
    else {
        horizonRateMagnitude = 1.0f;
    }

    return horizonRateMagnitude;
}

static void pidLevel(pidState_t *pidState, flight_dynamics_index_t axis, float horizonRateMagnitude)
{
    // This is ROLL/PITCH, run ANGLE/HORIZON controllers
    const float angleTarget = pidRcCommandToAngle(rcCommand[axis], pidProfile()->max_angle_inclination[axis]);
    const float angleErrorDeg = DECIDEGREES_TO_DEGREES(angleTarget - attitude.raw[axis]);

    float angleRateTarget = constrainf(angleErrorDeg * (pidBank()->pid[PID_LEVEL].P / FP_PID_LEVEL_P_MULTIPLIER), -currentControlRateProfile->rates[axis] * 10.0f, currentControlRateProfile->rates[axis] * 10.0f);

    // Apply simple LPF to angleRateTarget to make response less jerky
    // Ideas behind this:
    //  1) Attitude is updated at gyro rate, rateTarget for ANGLE mode is calculated from attitude
    //  2) If this rateTarget is passed directly into gyro-base PID controller this effectively doubles the rateError.
    //     D-term that is calculated from error tend to amplify this even more. Moreover, this tend to respond to every
    //     slightest change in attitude making self-leveling jittery
    //  3) Lowering LEVEL P can make the effects of (2) less visible, but this also slows down self-leveling.
    //  4) Human pilot response to attitude change in RATE mode is fairly slow and smooth, human pilot doesn't
    //     compensate for each slightest change
    //  5) (2) and (4) lead to a simple idea of adding a low-pass filter on rateTarget for ANGLE mode damping
    //     response to rapid attitude changes and smoothing out self-leveling reaction
    if (pidBank()->pid[PID_LEVEL].I) {
        // I8[PIDLEVEL] is filter cutoff frequency (Hz). Practical values of filtering frequency is 5-10 Hz
        angleRateTarget = pt1FilterApply4(&pidState->angleFilterState, angleRateTarget, pidBank()->pid[PID_LEVEL].I, dT);
    }

    // P[LEVEL] defines self-leveling strength (both for ANGLE and HORIZON modes)
    if (FLIGHT_MODE(HORIZON_MODE)) {
        pidState->rateTarget = (1.0f - horizonRateMagnitude) * angleRateTarget + horizonRateMagnitude * pidState->rateTarget;
    } else {
        pidState->rateTarget = angleRateTarget;
    }
}

/* Apply angular acceleration limit to rate target to limit extreme stick inputs to respect physical capabilities of the machine */
static void pidApplySetpointRateLimiting(pidState_t *pidState, flight_dynamics_index_t axis)
{
    const uint32_t axisAccelLimit = (axis == FD_YAW) ? pidProfile()->axisAccelerationLimitYaw : pidProfile()->axisAccelerationLimitRollPitch;

    if (axisAccelLimit > AXIS_ACCEL_MIN_LIMIT) {
        pidState->rateTarget = rateLimitFilterApply4(&pidState->axisAccelFilter, pidState->rateTarget, (float)axisAccelLimit, dT);
    }
}

#ifdef USE_SERVOS
static void pidApplyFixedWingRateController(pidState_t *pidState, flight_dynamics_index_t axis)
{
    const float rateError = pidState->rateTarget - pidState->gyroRate;

    // Calculate new P-term and FF-term
    float newPTerm = rateError * pidState->kP;
    float newFFTerm = pidState->rateTarget * pidState->kFF;

    // Additional P-term LPF on YAW axis
    if (axis == FD_YAW && pidProfile()->yaw_lpf_hz) {
        newPTerm = pt1FilterApply4(&pidState->ptermLpfState, newPTerm, pidProfile()->yaw_lpf_hz, dT);
    }

    // Calculate integral
    pidState->errorGyroIf += rateError * pidState->kI * dT;

    if (STATE(ANTI_WINDUP)) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidState->errorGyroIfLimit, pidState->errorGyroIfLimit);
    } else {
        pidState->errorGyroIfLimit = ABS(pidState->errorGyroIf);
    }

    if (pidProfile()->fixedWingItermThrowLimit != 0) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidProfile()->fixedWingItermThrowLimit, pidProfile()->fixedWingItermThrowLimit);
    }

#ifdef AUTOTUNE_FIXED_WING
    if (FLIGHT_MODE(AUTO_TUNE) && !FLIGHT_MODE(PASSTHRU_MODE)) {
        autotuneFixedWingUpdate(axis, pidState->rateTarget, pidState->gyroRate, newPTerm + newFFTerm);
    }
#endif

    axisPID[axis] = constrainf(newPTerm + newFFTerm + pidState->errorGyroIf, -pidProfile()->pidSumLimit, +pidProfile()->pidSumLimit);

#ifdef BLACKBOX
    axisPID_P[axis] = newPTerm;
    axisPID_I[axis] = pidState->errorGyroIf;
    axisPID_D[axis] = newFFTerm;
    axisPID_Setpoint[axis] = pidState->rateTarget;
#endif
}
#endif

static void pidApplyMulticopterRateController(pidState_t *pidState, flight_dynamics_index_t axis)
{
    const float rateError = pidState->rateTarget - pidState->gyroRate;

    // Calculate new P-term
    float newPTerm = rateError * pidState->kP;
    // Constrain YAW by yaw_p_limit value if not servo driven (in that case servo limits apply)
    if (axis == FD_YAW && (getMotorCount() >= 4 && pidProfile()->yaw_p_limit)) {
        newPTerm = constrain(newPTerm, -pidProfile()->yaw_p_limit, pidProfile()->yaw_p_limit);
    }

    // Additional P-term LPF on YAW axis
    if (axis == FD_YAW && pidProfile()->yaw_lpf_hz) {
        newPTerm = pt1FilterApply4(&pidState->ptermLpfState, newPTerm, pidProfile()->yaw_lpf_hz, dT);
    }

    // Calculate new D-term
    float newDTerm;
    if (pidBank()->pid[axis].D == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        newDTerm = 0;
    } else {
        firFilterUpdate(&pidState->gyroRateFilter, pidProfile()->dterm_setpoint_weight * pidState->rateTarget - pidState->gyroRate);
        newDTerm = firFilterApply(&pidState->gyroRateFilter) * (pidState->kD / dT);

        // Apply additional lowpass
        if (pidProfile()->dterm_lpf_hz) {
            newDTerm = pt1FilterApply4(&pidState->deltaLpfState, newDTerm, pidProfile()->dterm_lpf_hz, dT);
        }

#ifdef USE_DTERM_NOTCH
        newDTerm = notchFilterApplyFn(&pidState->deltaNotchFilter, newDTerm);
#endif

        // Additionally constrain D
        newDTerm = constrainf(newDTerm, -300.0f, 300.0f);
    }

    // TODO: Get feedback from mixer on available correction range for each axis
    const float newOutput = newPTerm + newDTerm + pidState->errorGyroIf;
    const float newOutputLimited = constrainf(newOutput, -pidProfile()->pidSumLimit, +pidProfile()->pidSumLimit);

    // Prevent strong Iterm accumulation during stick inputs
    const float integratorThreshold = (axis == FD_YAW) ? pidProfile()->yawItermIgnoreRate : pidProfile()->rollPitchItermIgnoreRate;
    const float antiWindupScaler = constrainf(1.0f - (ABS(pidState->rateTarget) / integratorThreshold), 0.0f, 1.0f);

    pidState->errorGyroIf += (rateError * pidState->kI * antiWindupScaler * dT) + ((newOutputLimited - newOutput) * pidState->kT * dT);

    // Don't grow I-term if motors are at their limit
    if (STATE(ANTI_WINDUP) || mixerIsOutputSaturated()) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidState->errorGyroIfLimit, pidState->errorGyroIfLimit);
    } else {
        pidState->errorGyroIfLimit = ABS(pidState->errorGyroIf);
    }

    axisPID[axis] = newOutputLimited;

#ifdef BLACKBOX
    axisPID_P[axis] = newPTerm;
    axisPID_I[axis] = pidState->errorGyroIf;
    axisPID_D[axis] = newDTerm;
    axisPID_Setpoint[axis] = pidState->rateTarget;
#endif
}

void updateHeadingHoldTarget(int16_t heading)
{
    headingHoldTarget = heading;
}

void resetHeadingHoldTarget(int16_t heading)
{
    updateHeadingHoldTarget(heading);
    pt1FilterReset(&headingHoldRateFilter, 0.0f);
}

int16_t getHeadingHoldTarget() {
    return headingHoldTarget;
}

static uint8_t getHeadingHoldState()
{
    // Don't apply heading hold if overall tilt is greater than maximum angle inclination
    if (calculateCosTiltAngle() < headingHoldCosZLimit) {
        return HEADING_HOLD_DISABLED;
    }

#if defined(NAV)
    int navHeadingState = navigationGetHeadingControlState();
    // NAV will prevent MAG_MODE from activating, but require heading control
    if (navHeadingState != NAV_HEADING_CONTROL_NONE) {
        // Apply maghold only if heading control is in auto mode
        if (navHeadingState == NAV_HEADING_CONTROL_AUTO) {
            return HEADING_HOLD_ENABLED;
        }
    }
    else
#endif
    if (ABS(rcCommand[YAW]) == 0 && FLIGHT_MODE(HEADING_MODE)) {
        return HEADING_HOLD_ENABLED;
    } else {
        return HEADING_HOLD_UPDATE_HEADING;
    }

    return HEADING_HOLD_UPDATE_HEADING;
}

/*
 * HEADING_HOLD P Controller returns desired rotation rate in dps to be fed to Rate controller
 */
float pidHeadingHold(void)
{
    float headingHoldRate;

    int16_t error = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headingHoldTarget;

    /*
     * Convert absolute error into relative to current heading
     */
    if (error <= -180) {
        error += 360;
    }

    if (error >= +180) {
        error -= 360;
    }

    /*
        New MAG_HOLD controller work slightly different that previous one.
        Old one mapped error to rotation speed in following way:
            - on rate 0 it gave about 0.5dps for each degree of error
            - error 0 = rotation speed of 0dps
            - error 180 = rotation speed of 96 degrees per second
            - output
            - that gives about 2 seconds to correct any error, no matter how big. Of course, usually more because of inertia.
        That was making him quite "soft" for small changes and rapid for big ones that started to appear
        when iNav introduced real RTH and WAYPOINT that might require rapid heading changes.

        New approach uses modified principle:
            - manual yaw rate is not used. MAG_HOLD is decoupled from manual input settings
            - instead, mag_hold_rate_limit is used. It defines max rotation speed in dps that MAG_HOLD controller can require from RateController
            - computed rotation speed is capped at -mag_hold_rate_limit and mag_hold_rate_limit
            - Default mag_hold_rate_limit = 40dps and default MAG_HOLD P-gain is 40
            - With those values, maximum rotation speed will be required from Rate Controller when error is greater that 30 degrees
            - For smaller error, required rate will be proportional.
            - It uses LPF filter set at 2Hz to additionally smoothen out any rapid changes
            - That makes correction of smaller errors stronger, and those of big errors softer

        This make looks as very slow rotation rate, but please remember this is automatic mode.
        Manual override with YAW input when MAG_HOLD is enabled will still use "manual" rates, not MAG_HOLD rates.
        Highest possible correction is 180 degrees and it will take more less 4.5 seconds. It is even more than sufficient
        to run RTH or WAYPOINT missions. My favourite rate range here is 20dps - 30dps that gives nice and smooth turns.

        Correction for small errors is much faster now. For example, old contrioller for 2deg errors required 1dps (correction in 2 seconds).
        New controller for 2deg error requires 2,6dps. 4dps for 3deg and so on up until mag_hold_rate_limit is reached.
    */

    headingHoldRate = error * pidBank()->pid[PID_HEADING].P / 30;
    headingHoldRate = constrainf(headingHoldRate, -pidProfile()->heading_hold_rate_limit, pidProfile()->heading_hold_rate_limit);
    headingHoldRate = pt1FilterApply4(&headingHoldRateFilter, headingHoldRate, HEADING_HOLD_ERROR_LPF_FREQ, dT);

    return headingHoldRate;
}

#ifdef USE_FLM_TURN_ASSIST
/*
 * TURN ASSISTANT mode is an assisted mode to do a Yaw rotation on a ground plane, allowing one-stick turn in RATE more
 * and keeping ROLL and PITCH attitude though the turn.
 */
static void pidTurnAssistant(pidState_t *pidState)
{
    t_fp_vector targetRates;
    targetRates.V.X = 0.0f;
    targetRates.V.Y = 0.0f;

    if (STATE(FIXED_WING)) {
        if (calculateCosTiltAngle() >= 0.173648f) {
            // Ideal banked turn follow the equations:
            //      forward_vel^2 / radius = Gravity * tan(roll_angle)
            //      yaw_rate = forward_vel / radius
            // If we solve for roll angle we get:
            //      tan(roll_angle) = forward_vel * yaw_rate / Gravity
            // If we solve for yaw rate we get:
            //      yaw_rate = tan(roll_angle) * Gravity / forward_vel

#if defined(PITOT)
            float airspeedForCoordinatedTurn = sensors(SENSOR_PITOT) ?
                    pitot.airSpeed :
                    pidProfile()->fixedWingReferenceAirspeed;
#else
            float airspeedForCoordinatedTurn = pidProfile()->fixedWingReferenceAirspeed;
#endif

            // Constrain to somewhat sane limits - 10km/h - 216km/h
            airspeedForCoordinatedTurn = constrainf(airspeedForCoordinatedTurn, 300, 6000);

            // Calculate rate of turn in Earth frame according to FAA's Pilot's Handbook of Aeronautical Knowledge
            float bankAngle = DECIDEGREES_TO_RADIANS(attitude.values.roll);
            float coordinatedTurnRateEarthFrame = GRAVITY_CMSS * tan_approx(-bankAngle) / airspeedForCoordinatedTurn;

            targetRates.V.Z = RADIANS_TO_DEGREES(coordinatedTurnRateEarthFrame);
        }
        else {
            // Don't allow coordinated turn calculation if airplane is in hard bank or steep climb/dive
            return;
        }
    }
    else {
        targetRates.V.Z = pidState[YAW].rateTarget;
    }

    // Transform calculated rate offsets into body frame and apply
    imuTransformVectorEarthToBody(&targetRates);

    // Add in roll and pitch
    pidState[ROLL].rateTarget = constrainf(pidState[ROLL].rateTarget + targetRates.V.X, -currentControlRateProfile->rates[ROLL] * 10.0f, currentControlRateProfile->rates[ROLL] * 10.0f);
    pidState[PITCH].rateTarget = constrainf(pidState[PITCH].rateTarget + targetRates.V.Y, -currentControlRateProfile->rates[PITCH] * 10.0f, currentControlRateProfile->rates[PITCH] * 10.0f);

    // Replace YAW on quads - add it in on airplanes
    if (STATE(FIXED_WING)) {
        pidState[YAW].rateTarget = constrainf(pidState[YAW].rateTarget + targetRates.V.Z * pidProfile()->fixedWingCoordinatedYawGain, -currentControlRateProfile->rates[YAW] * 10.0f, currentControlRateProfile->rates[YAW] * 10.0f);
    }
    else {
        pidState[YAW].rateTarget = constrainf(targetRates.V.Z, -currentControlRateProfile->rates[YAW] * 10.0f, currentControlRateProfile->rates[YAW] * 10.0f);
    }
}
#endif

void pidController(void)
{
    uint8_t headingHoldState = getHeadingHoldState();

    if (headingHoldState == HEADING_HOLD_UPDATE_HEADING) {
        updateHeadingHoldTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    }

    for (int axis = 0; axis < 3; axis++) {
        // Step 1: Calculate gyro rates
        pidState[axis].gyroRate = gyro.gyroADCf[axis];

        // Step 2: Read target
        float rateTarget;

        if (axis == FD_YAW && headingHoldState == HEADING_HOLD_ENABLED) {
            rateTarget = pidHeadingHold();
        } else {
            rateTarget = pidRcCommandToRate(rcCommand[axis], currentControlRateProfile->rates[axis]);
        }

        // Limit desired rate to something gyro can measure reliably
        pidState[axis].rateTarget = constrainf(rateTarget, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);
    }

    // Step 3: Run control for ANGLE_MODE, HORIZON_MODE, and HEADING_LOCK
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        const float horizonRateMagnitude = calcHorizonRateMagnitude();
        pidLevel(&pidState[FD_ROLL], FD_ROLL, horizonRateMagnitude);
        pidLevel(&pidState[FD_PITCH], FD_PITCH, horizonRateMagnitude);
    }

#ifdef USE_FLM_TURN_ASSIST
    if (FLIGHT_MODE(TURN_ASSISTANT) || navigationRequiresTurnAssistance()) {
        pidTurnAssistant(pidState);
    }
#endif

    // Apply setpoint rate of change limits
    for (int axis = 0; axis < 3; axis++) {
        pidApplySetpointRateLimiting(&pidState[axis], axis);
    }

    // Step 4: Run gyro-driven control
    for (int axis = 0; axis < 3; axis++) {
        // Apply PID setpoint controller
#ifdef USE_SERVOS
        if (STATE(FIXED_WING)) {
            pidApplyFixedWingRateController(&pidState[axis], axis);
        }
        else {
            pidApplyMulticopterRateController(&pidState[axis], axis);
        }
#else
        pidApplyMulticopterRateController(&pidState[axis], axis);
#endif
    }
}
