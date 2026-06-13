/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include <math.h>
#include <string.h>

#ifdef USE_AIRPLANE_SAS

#include "common/maths.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"
#include "fc/rc_modes.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "io/gps.h"

#include "build/debug.h"

#include "flight/mixer.h"
#include "flight/airplane_sas.h"

FAST_DATA_ZERO_INIT psas_data_t psasData;
FAST_DATA_ZERO_INIT psasRuntime_t psasRuntime;

static bool isActivePSAS = false;
static bool isLiftCoefValid = false;
static float validLiftCoefTime = 0.0f;

static bool isEnabledAccelZController = false;
static bool isEnabledLiftCoefEstimation = false;
static bool isEnabledAoALimiter = false;

static pt1Filter_t psasPitchDampingLowpass;
static pt1Filter_t psasYawDampingLowpass;
static pt1Filter_t psasLiftCoefLowpass;
static pt1Filter_t psasAccelZLowpass;
static pt1Filter_t psasAccelYLowpass;

void psasInit(const pidProfile_t *pidProfile)
{
    pt1FilterInit(&psasPitchDampingLowpass, pt1FilterGain(pidProfile->psas_pitch_damping_filter_freq * 0.01f, pidRuntime.dT));
    pt1FilterInit(&psasYawDampingLowpass, pt1FilterGain(pidProfile->psas_yaw_damping_filter_freq * 0.01f, pidRuntime.dT));
    pt1FilterInit(&psasLiftCoefLowpass, pt1FilterGain(pidProfile->psas_lift_coef_filter_freq * 0.1f, pidRuntime.dT));
    pt1FilterInit(&psasAccelZLowpass, pt1FilterGain(pidProfile->psas_accel_z_filter_freq * 0.1f, pidRuntime.dT));
    pt1FilterInit(&psasAccelYLowpass, pt1FilterGain(pidProfile->psas_accel_y_filter_freq * 0.1f, pidRuntime.dT));

    for (int i=0; i < XYZ_AXIS_COUNT; i++) {
        psasRuntime.stick_gain[i] = pidProfile->psas_stick_gain[i];
        psasRuntime.damping_gain[i] = pidProfile->psas_damping_gain[i] * 0.001f;
    }
    psasRuntime.pitch_stability_gain = pidProfile->psas_pitch_stability_gain * 0.1f;
    psasRuntime.pitch_accel_p_gain = pidProfile->psas_pitch_accel_p_gain * 0.1f;
    psasRuntime.pitch_accel_i_gain = pidProfile->psas_pitch_accel_i_gain;
    psasRuntime.pitch_accel_min = pidProfile->psas_pitch_accel_min * 0.1f;
    psasRuntime.pitch_accel_max = pidProfile->psas_pitch_accel_max * 0.1f;
    psasRuntime.yaw_stability_gain = pidProfile->psas_yaw_stability_gain * 0.1f;
    psasRuntime.wing_load = pidProfile->psas_wing_load * 0.01f;
    psasRuntime.air_density = pidProfile->psas_air_density * 0.001f;
    psasRuntime.lift_c_limit = pidProfile->psas_lift_c_limit * 0.1f;
    psasRuntime.aoa_limiter_gain = pidProfile->psas_aoa_limiter_gain * 10.0f;
    psasRuntime.aoa_limiter_forecast_time = pidProfile->psas_aoa_limiter_forecast_time * 0.01f;
    psasRuntime.aoa_limiter_tau_return = pidProfile->psas_aoa_limiter_tau_return * 0.1f;
    psasRuntime.servoVelocityLimit = 100.0f / (pidProfile->psas_servo_time * 0.001f); // Limit servo velocity %/s. The psas_servo_time can not be zero - the CLI minimum value is 5.
    psasRuntime.roll_yaw_clift_start = pidProfile->psas_roll_yaw_clift_start * 0.1f;
    psasRuntime.roll_yaw_clift_stop = pidProfile->psas_roll_yaw_clift_stop * 0.1f;
    psasRuntime.roll_to_yaw_link = pidProfile->psas_roll_to_yaw_link * 0.1f;

    isEnabledAccelZController = sensors(SENSOR_ACC) && pidProfile->psas_pitch_accel_i_gain != 0; // Enable controller for non zero I. The P (psas_pitch_accel_p_gain) is an additional option
    isEnabledLiftCoefEstimation = sensors(SENSOR_ACC) && pidProfile->psas_wing_load != 0;
    isEnabledAoALimiter = isEnabledLiftCoefEstimation && pidProfile->psas_aoa_limiter_gain != 0;
}

static void FAST_CODE_NOINLINE computeLiftCoefficient(const pidProfile_t *pidProfile, float accelZ, float *liftCoef, float *liftCoefVelocity)
{
    static float liftCoefLast = 0.0f; // liftCoefLast is full defined after timeForValid time, its first value does not matter after any re-init
    const float timeForValid = 3.0f;
    *liftCoef = 0.0f;
    *liftCoefVelocity = 0.0f;

    if (ARMING_FLAG(ARMED) &&
        STATE(GPS_FIX) &&
        gpsSol.numSat > GPS_MIN_SAT_COUNT) {
        const float speedThreshold = 1.5f;    // GPS speed threshold (m/s)
        float speed = 0.01f * gpsSol.speed3d;
        if (speed > speedThreshold) {
            const float airSpeedPressure = psasRuntime.air_density * sq(speed) / 2.0f;
            *liftCoef = accelZ * psasRuntime.wing_load * G_ACCELERATION / airSpeedPressure;
            if (pidProfile->psas_lift_coef_filter_freq != 0) {
                *liftCoef = pt1FilterApply(&psasLiftCoefLowpass, *liftCoef);
            }
            *liftCoefVelocity = (*liftCoef - liftCoefLast) / pidRuntime.dT;
            liftCoefLast = *liftCoef;
            // Enable AoA limiter after ~3s of stable lift to avoid triggering during launch
            if (!isLiftCoefValid) {
                if (*liftCoef < psasRuntime.lift_c_limit && *liftCoef > -psasRuntime.lift_c_limit) {
                    validLiftCoefTime += pidRuntime.dT;
                    if (validLiftCoefTime > timeForValid) {
                        isLiftCoefValid = true;
                    }
                } else {
                    validLiftCoefTime = 0.0f;
                }
                if (!isLiftCoefValid) {
                    *liftCoef = 0.0f;
                    *liftCoefVelocity = 0.0f;
                }
            }
        } else {
            isLiftCoefValid = false;
            validLiftCoefTime = 0.0f;
        }
    } else {
        isLiftCoefValid = false;
        liftCoefLast = 0.0f;
        validLiftCoefTime = 0.0f;
    }
}

// Accel Z (G load) error integrator with weak P-term: stick → desired G, I-term eliminates offset (astatic).
static float FAST_CODE_NOINLINE updateAccelZHoldingController(float pitchStick, float accelZ)
{
    float deltaAccP = 0.0f;
    float accelReq = pitchStick < 0.0f ? (1.0f - psasRuntime.pitch_accel_max) * pitchStick + 1.0f
                                           : -(1.0f + psasRuntime.pitch_accel_min) * pitchStick + 1.0f;
    float accelDelta = accelZ - accelReq;
    float servoVelocity = accelDelta * psasRuntime.pitch_accel_i_gain;
    servoVelocity = constrainf(servoVelocity, -psasRuntime.servoVelocityLimit, psasRuntime.servoVelocityLimit);

    psasData.pitch.I += servoVelocity * pidRuntime.dT;
    deltaAccP = accelDelta * psasRuntime.pitch_accel_p_gain;

    DEBUG_SET(DEBUG_PSAS, 3, lrintf(accelReq * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 4, lrintf(accelDelta * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 5, lrintf(deltaAccP * 10.0f));

    return deltaAccP;
}

// The angle of attack limiter. The aerodynamics lift force coefficient depends by angle of attack. Therefore it possible to use this coef instead of AoA value.
static bool FAST_CODE_NOINLINE updateAngleOfAttackLimiter(float liftCoef, float liftCoefVelocity)
{
    bool isLimitAoA = false;

    float liftCoefDiff = 0.0f;
    float servoVelocity = 0.0f;

    if (IS_RC_MODE_ACTIVE(BOXAOALIMITER)) {
        if (isLiftCoefValid) {
            psasData.pitch.aoaLimiterState = LIMITER_ON;
            const float liftCoefForecastChange = liftCoefVelocity * psasRuntime.aoa_limiter_forecast_time;
            if (liftCoef > 0.0f) {
                if (liftCoefForecastChange > 0.0f) {
                    liftCoef += liftCoefForecastChange;
                }
                liftCoefDiff = liftCoef - psasRuntime.lift_c_limit;
                if (liftCoefDiff > 0.0f) {
                    isLimitAoA = true;
                    servoVelocity = liftCoefDiff * psasRuntime.aoa_limiter_gain;
                    servoVelocity = constrainf(servoVelocity, -psasRuntime.servoVelocityLimit, psasRuntime.servoVelocityLimit);
                }
            } else {
                if (liftCoefForecastChange < 0.0f) {
                    liftCoef += liftCoefForecastChange;
                }
                liftCoefDiff = liftCoef + psasRuntime.lift_c_limit;
                if (liftCoefDiff < 0.0f) {
                    isLimitAoA = true;
                    servoVelocity = liftCoefDiff * psasRuntime.aoa_limiter_gain;
                    servoVelocity = constrainf(servoVelocity, -psasRuntime.servoVelocityLimit, psasRuntime.servoVelocityLimit);
                }
            }
        } else {
            psasData.pitch.aoaLimiterState = LIMITER_NOT_READY;
        }
    }

    if (isLimitAoA) {
        psasData.pitch.I += servoVelocity * pidRuntime.dT;
        psasData.pitch.aoaLimiterState = LIMITER_ACTIVE;
    } else if (!isEnabledAccelZController) {
        // Decay the AoA limiter I value when the limiter is off or lift coeff is not valid
        psasData.pitch.I -= psasData.pitch.I / psasRuntime.aoa_limiter_tau_return * pidRuntime.dT; // The psas_aoa_limiter_tau_return can not be zero - the CLI minimum value is 1.
    }

    DEBUG_SET(DEBUG_PSAS, 6, lrintf(liftCoefDiff * 100.0f));
    DEBUG_SET(DEBUG_PSAS, 7, isLimitAoA ? 1 : 0);

    return isLimitAoA;
}

// Roll to yaw control cross link to improve roll rotation at high angle of attack
static float FAST_CODE_NOINLINE rollToYawCrossLinkControl(const pidProfile_t *pidProfile, float rollPilotControl, float liftCoef)
{
    if (!isLiftCoefValid || pidProfile->psas_roll_to_yaw_link == 0) {
        return 0.0f;
    }

    float crossYawControl = 0.0f;
    const float denom = (psasRuntime.roll_yaw_clift_stop - psasRuntime.roll_yaw_clift_start);
    if (liftCoef > psasRuntime.roll_yaw_clift_start && denom > 1e-6f) {
        float k = (liftCoef - psasRuntime.roll_yaw_clift_start) / denom;
        k = constrainf(k, 0.0f, 1.0f);
        crossYawControl = k * (0.01f * rollPilotControl) * psasRuntime.roll_to_yaw_link;
    }

    return crossYawControl;
}

static void FAST_CODE_NOINLINE psasUpdate(const pidProfile_t *pidProfile)
{
    // Pitch channel
    // Pilot pitch control
    // The positive stick and servos deflection gives positive pitch rotation (the nose moves down)
    const float maxRcRatePitch = MAX(getMaxRcRate(FD_PITCH), 1.0f);
    float pitchStick = getSetpointRate(FD_PITCH) / maxRcRatePitch;  // pitch stick [-1 ... +1]
    psasData.pitch.pilot = pitchStick * psasRuntime.stick_gain[FD_PITCH];

    // Plane pitch damping improvement
    float gyroPitch = !gyroOverflowDetected() ? gyro.gyroADCf[FD_PITCH] : 0.0f;
    if (pidProfile->psas_pitch_damping_filter_freq != 0) {
        float gyroPitchLow = pt1FilterApply(&psasPitchDampingLowpass, gyroPitch);
        gyroPitch -= gyroPitchLow;      // Damping the pitch gyro high freq part only
    }
    // On the positive pitch gyro rotation (nose to down direction) it needs to turn nose up, therefore it needs to use negative sign
    psasData.pitch.damping = -1.0f * gyroPitch * (psasRuntime.damping_gain[FD_PITCH]);

    // Plane pitch stability improvement
    float accelZ = 1.0f;
    float accelZFiltered = 1.0f;
    if (sensors(SENSOR_ACC)) {
        accelZ = acc.accADC.z * acc.dev.acc_1G_rec;
        if (pidProfile->psas_accel_z_filter_freq != 0) {
            accelZFiltered = pt1FilterApply(&psasAccelZLowpass, accelZ);
        } else {
            accelZFiltered = accelZ;
        }
    }
    // On the positive accel Z value (up direction) it needs to turn nose down, therefore it needs to use positive sign
    psasData.pitch.stability = (accelZFiltered - 1.0f) * psasRuntime.pitch_stability_gain;

    psasData.pitch.Sum = psasData.pitch.pilot + psasData.pitch.damping + psasData.pitch.stability;

    // Additional features
    // We have not got the angle of attack (AoA) sensor
    // Therefore to use lift coefficient instead of AoA. It is proportional AoA in the linear region
    float liftCoef = 0.0f;
    float liftCoefVelocity = 0.0f;
    if (isEnabledLiftCoefEstimation) {
        computeLiftCoefficient(pidProfile, accelZ, &liftCoef, &liftCoefVelocity);
    }

    // If the lift coefficent (angle of attack) is valid and its value is over limit, then limit value.
    bool isLimitAoA = false;
    psasData.pitch.aoaLimiterState = LIMITER_DISABLED;
    if (isEnabledAoALimiter) {
        isLimitAoA = updateAngleOfAttackLimiter(liftCoef, liftCoefVelocity);
    }

    // Else, if the lift coefficent (angle of attack) value is normal then hold required G load (accel z) value.
    psasData.pitch.accelP = 0.0f;
    if (isEnabledAccelZController && !isLimitAoA) {
        psasData.pitch.accelP = updateAccelZHoldingController(pitchStick, accelZ);
        psasData.pitch.Sum += psasData.pitch.accelP;
    }

    psasData.pitch.Sum = constrainf(psasData.pitch.Sum, -100.0f, 100.0f);

    // The AoA limiter and Accel Z controller accumulate pitch I value
    // limit integrator output and add it to Sum
    if (isEnabledAoALimiter || isEnabledAccelZController) {
        float output = psasData.pitch.Sum + psasData.pitch.I;
        if (output > 100.0f) {
            psasData.pitch.I = 100.0f - psasData.pitch.Sum;
        } else if (output < -100.0f) {
            psasData.pitch.I = -100.0f - psasData.pitch.Sum;
        }

        // Add integrator output to Sum value
        psasData.pitch.Sum += psasData.pitch.I;
    }

    // Roll channel
    // Pilot roll control
    // The positive stick and servos deflection gives positive roll rotation (on the right side)
    const float maxRcRateRoll = MAX(getMaxRcRate(FD_ROLL), 1.0f);
    psasData.roll.pilot = getSetpointRate(FD_ROLL) / maxRcRateRoll * (psasRuntime.stick_gain[FD_ROLL]);

    // Plane roll damping improvement
    // On the positive roll gyro rotation (on the right side direction) it needs to turn plane on the left, therefore it needs to use negative sign
    if (!gyroOverflowDetected()) {
        psasData.roll.damping = -1.0f * gyro.gyroADCf[FD_ROLL] * (psasRuntime.damping_gain[FD_ROLL]);
    } else {
        psasData.roll.damping = 0.0f;
    }

    psasData.roll.Sum = psasData.roll.pilot + psasData.roll.damping;
    psasData.roll.Sum = constrainf(psasData.roll.Sum, -100.0f, 100.0f);

    // Yaw channel
    // Pilot yaw control
    // The positive rudder deflection gives positive yaw rotation (the nose moves to the left side)
    const float maxRcRateYaw = MAX(getMaxRcRate(FD_YAW), 1.0f);
    psasData.yaw.pilot = getSetpointRate(FD_YAW) / maxRcRateYaw * (psasRuntime.stick_gain[FD_YAW]);

    // Plane yaw damping improvement
    float gyroYaw = !gyroOverflowDetected() ? gyro.gyroADCf[FD_YAW] : 0.0f;
    if (pidProfile->psas_yaw_damping_filter_freq != 0) {
        float gyroYawLow = pt1FilterApply(&psasYawDampingLowpass, gyroYaw);
        gyroYaw -= gyroYawLow;      // Damping the yaw gyro high freq part only
    }
    // On the positive yaw gyro rotation (left side direction) it needs to turn nose on the right, therefore it needs to use negative sign
    psasData.yaw.damping = -gyroYaw * (psasRuntime.damping_gain[FD_YAW]);

    // Plane yaw stability improvement
    float accelYFiltered = 0.0f;
    if (sensors(SENSOR_ACC)) {
        float accelY = acc.accADC.y * acc.dev.acc_1G_rec;
        if (pidProfile->psas_accel_y_filter_freq != 0) {
            accelYFiltered = pt1FilterApply(&psasAccelYLowpass, accelY);
        } else {
            accelYFiltered = accelY;
        }
    }
    // On the positive accel Y value (left side direction) it needs to turn nose on the right, therefore it needs to use negative sign
    psasData.yaw.stability = -accelYFiltered * psasRuntime.yaw_stability_gain;

    // The roll rotation to yaw channel cross link to improve roll rotation on the high angle of attack flight
    // On the right roll rotation it needs the nose on the right yaw rotation, therefore it needs to use negative sign
    psasData.yaw.rollToYawCrossLink = -rollToYawCrossLinkControl(pidProfile, psasData.roll.pilot, liftCoef);

    psasData.yaw.Sum = psasData.yaw.pilot + psasData.yaw.damping + psasData.yaw.stability + psasData.yaw.rollToYawCrossLink;
    psasData.yaw.Sum = constrainf(psasData.yaw.Sum, -100.0f, 100.0f);

    DEBUG_SET(DEBUG_PSAS, 0, lrintf(psasData.pitch.Sum * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 1, lrintf(psasData.pitch.I * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 2, lrintf(liftCoef * 100.0f));
}

bool FAST_CODE_NOINLINE psasHandleMode(const pidProfile_t *pidProfile)
{
    bool isPSAS = isFixedWing() && FLIGHT_MODE(AIRPLANE_SAS_MODE);
    if (isPSAS) {
        if (!isActivePSAS) {
            for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
                pidData[axis].P = 0;
                pidData[axis].I = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
                pidData[axis].S = 0;
                pidData[axis].Sum = 0;
            }
            isActivePSAS = true;
        }

        if (pidRuntime.pidStabilisationEnabled) {
            psasUpdate(pidProfile);
        } else {
            memset(&psasData, 0, sizeof(psasData));
        }

        return true;
    } else if (isActivePSAS) {
        memset(&psasData, 0, sizeof(psasData));
        isActivePSAS = false;
        isLiftCoefValid = false;
        validLiftCoefTime = 0.0f;
        return false;
    }
    return false;
}
#endif
