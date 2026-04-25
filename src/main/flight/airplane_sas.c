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

#ifdef USE_AIRPLANE_SAS

#include "fc/rc.h"
#include "fc/runtime_config.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "io/gps.h"
#include <math.h>
#include "common/maths.h"
#include "build/debug.h"
#include "flight/mixer.h"
#include "flight/airplane_sas.h"

FAST_DATA_ZERO_INIT psas_data_t psasData;

static bool isReadyPSAS;
static pt1Filter_t psasPitchDampingLowpass;
static pt1Filter_t psasYawDampingLowpass;
static pt1Filter_t psasLiftCoefLowpass;
static pt1Filter_t psasAccelZLowpass;
static pt1Filter_t psasAccelYLowpass;

static bool isLiftCoefValid = false;
static float validLiftCoefTime = 0.0f;

static bool isEnabledAccelZController = false,
            isEnabledAoALimiter = false;

void psasInit(const pidProfile_t *pidProfile)
{
    pt1FilterInit(&psasPitchDampingLowpass, pt1FilterGain(pidProfile->psas_pitch_damping_filter_freq * 0.01f, pidRuntime.dT));
    pt1FilterInit(&psasYawDampingLowpass, pt1FilterGain(pidProfile->psas_yaw_damping_filter_freq * 0.01f, pidRuntime.dT));
    pt1FilterInit(&psasLiftCoefLowpass, pt1FilterGain(pidProfile->psas_lift_coef_filter_freq * 0.1f, pidRuntime.dT));
    pt1FilterInit(&psasAccelZLowpass, pt1FilterGain(pidProfile->psas_accel_z_filter_freq * 0.1f, pidRuntime.dT));
    pt1FilterInit(&psasAccelYLowpass, pt1FilterGain(pidProfile->psas_accel_y_filter_freq * 0.1f, pidRuntime.dT));

    isReadyPSAS = false;
    isLiftCoefValid = false;
    validLiftCoefTime = 0.0f;
    isEnabledAccelZController = sensors(SENSOR_ACC) && (pidProfile->psas_pitch_accel_i_gain != 0);
    isEnabledAoALimiter = pidProfile->psas_aoa_limiter_gain != 0;
}

static void computeLiftCoefficient(const pidProfile_t *pidProfile, float accelZ, float *liftCoef, float *liftCoefVelocity)
{
    static float liftCoefLast = 0.0f;
    const float timeForValid = 3.0f;
    *liftCoef = 0.0f;
    *liftCoefVelocity = 0.0f;

    if (ARMING_FLAG(ARMED) &&
        sensors(SENSOR_ACC) &&
        sensors(SENSOR_GPS) &&
        STATE(GPS_FIX) &&
        gpsSol.numSat > GPS_MIN_SAT_COUNT) {
        const float limitLiftC = 0.1f * pidProfile->psas_lift_c_limit;
        const float speedThreshold = 2.5f;    // GPS speed threshold (m/s)
        float speed = 0.01f * gpsSol.speed3d;
        if (speed > speedThreshold) {
            const float airSpeedPressure = (0.001f * pidProfile->psas_air_density) * sq(speed) / 2.0f;
            *liftCoef = accelZ * (0.01f * pidProfile->psas_wing_load) * G_ACCELERATION / airSpeedPressure;
            if (pidProfile->psas_lift_coef_filter_freq != 0) {
                *liftCoef = pt1FilterApply(&psasLiftCoefLowpass, *liftCoef);
            }
            *liftCoefVelocity = (*liftCoef - liftCoefLast) / pidRuntime.dT;
            liftCoefLast = *liftCoef;
            // Enable AoA limiter after ~3s of stable lift to avoid triggering during launch
            if (!isLiftCoefValid) {
                if (*liftCoef < limitLiftC && *liftCoef > -limitLiftC) {
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
        validLiftCoefTime = 0.0f;
    }
}

//The astatic accel Z controller by stick position
static float updateAstaticAccelZController(const pidProfile_t *pidProfile, float pitchStick, float accelZ)
{
    float deltaAccP = 0.0f;
    const float servoVelocityLimit = 100.0f / (pidProfile->psas_servo_time * 0.001f); // Limit servo velocity %/s
    float accelReq = pitchStick < 0.0f ? (1.0f - 0.1f * pidProfile->psas_pitch_accel_max) * pitchStick + 1.0f
                                           : -(1.0f + 0.1f * pidProfile->psas_pitch_accel_min) * pitchStick + 1.0f;
    float accelDelta = accelZ - accelReq;
    float servoVelocity = accelDelta * pidProfile->psas_pitch_accel_i_gain;
    servoVelocity = constrainf(servoVelocity, -servoVelocityLimit, servoVelocityLimit);

    psasData.pitch.I += servoVelocity * pidRuntime.dT;
    deltaAccP = accelDelta * pidProfile->psas_pitch_accel_p_gain * 0.1f;

    DEBUG_SET(DEBUG_PSAS, 3, lrintf(accelReq * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 4, lrintf(accelDelta * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 5, lrintf(deltaAccP * 10.0f));

    return deltaAccP;
}

// The angle of attack limiter. The aerodynamics lift force coefficient depends by angle of attack. Therefore it possible to use this coef instead of AoA value.
static bool updateAngleOfAttackLimiter(const pidProfile_t *pidProfile, float liftCoef, float liftCoefVelocity)
{
    bool isLimitAoA = false;

    float liftCoefDiff = 0.0f,
          servoVelocity = 0.0f;
    if (isLiftCoefValid) {
        const float liftCoefForecastChange = liftCoefVelocity * (pidProfile->psas_aoa_limiter_forecast_time * 0.01f);
        const float limitLiftC = 0.1f * pidProfile->psas_lift_c_limit;

        const float servoVelocityLimit = 100.0f / (pidProfile->psas_servo_time * 0.001f); // Limit servo velocity %/s
        if (liftCoef > 0.0f) {
            if (liftCoefForecastChange > 0.0f) {
                liftCoef += liftCoefForecastChange;
            }
            liftCoefDiff = liftCoef - limitLiftC;
            if (liftCoefDiff > 0.0f) {
                isLimitAoA = true;
                servoVelocity = liftCoefDiff * (pidProfile->psas_aoa_limiter_gain * 10.0f);
                servoVelocity = constrainf(servoVelocity, -servoVelocityLimit, servoVelocityLimit);
            }
        } else {
            if (liftCoefForecastChange < 0.0f) {
                liftCoef += liftCoefForecastChange;
            }
            liftCoefDiff = liftCoef + limitLiftC;
            if (liftCoefDiff < 0.0f) {
                isLimitAoA = true;
                servoVelocity = liftCoefDiff * (pidProfile->psas_aoa_limiter_gain * 10.0f);
                servoVelocity = constrainf(servoVelocity, -servoVelocityLimit, servoVelocityLimit);
            }
        }
    }

    if (isLimitAoA) {
        psasData.pitch.I += servoVelocity * pidRuntime.dT;
    } else if (!isEnabledAccelZController) {
        // Decay the AoA limiter I value when the limiter is off or lift coeff is not valid
        psasData.pitch.I -= psasData.pitch.I / (pidProfile->psas_aoa_limiter_tau_return * 0.1f) * pidRuntime.dT;
    }

    DEBUG_SET(DEBUG_PSAS, 6, lrintf(liftCoefDiff * 100.0f));
    DEBUG_SET(DEBUG_PSAS, 7, isLimitAoA ? 1 : 0);

    return isLimitAoA;
}

// Roll to yaw control cross link to improve roll rotation at high angle of attack
static float rollToYawCrossLinkControl(const pidProfile_t *pidProfile, float rollPilotControl, float liftCoef)
{
    if (!isLiftCoefValid || pidProfile->psas_roll_to_yaw_link == 0) {
        return 0.0f;
    }

    float crossYawControl = 0.0f,
          roll_yaw_clift_start = 0.1f * pidProfile->psas_roll_yaw_clift_start,
          roll_yaw_clift_stop = 0.1f * pidProfile->psas_roll_yaw_clift_stop;

    const float denom = (roll_yaw_clift_stop - roll_yaw_clift_start);
    if (pidProfile->psas_roll_to_yaw_link && liftCoef > roll_yaw_clift_start && denom > 1e-6f) {
        float k = (liftCoef - roll_yaw_clift_start) / denom;
        k = constrainf(k, 0.0f, 1.0f);
        crossYawControl = k * (0.01f * rollPilotControl) * (pidProfile->psas_roll_to_yaw_link * 0.1f);
    }

    return crossYawControl;
}

void FAST_CODE_NOINLINE psasUpdate(const pidProfile_t *pidProfile)
{
    // Pitch channel
    // Pilot pitch control
    const float maxRcRatePitch = fmaxf(getMaxRcRate(FD_PITCH), 1.0f);
    float pitchStick = getSetpointRate(FD_PITCH) / maxRcRatePitch;  // pitch stick [-1 ... +1]
    psasData.pitch.pilot = pitchStick * pidProfile->psas_stick_gain[FD_PITCH];

    // Plane pitch damping improvement
    float gyroPitch = gyro.gyroADCf[FD_PITCH];
    if (pidProfile->psas_pitch_damping_filter_freq != 0) {
        float gyroPitchLow = pt1FilterApply(&psasPitchDampingLowpass, gyroPitch);
        gyroPitch -= gyroPitchLow;      // Damping the pitch gyro high freq part only
    }
    psasData.pitch.damping = -1.0f * gyroPitch * (pidProfile->psas_damping_gain[FD_PITCH] * 0.001f);

    // Plane pitch stability improvement
    float accelZ = 1.0f,
          accelZ_filtered = 1.0f;
    if (sensors(SENSOR_ACC)) {
        accelZ =  acc.accADC.z * acc.dev.acc_1G_rec;
        if (pidProfile->psas_accel_z_filter_freq != 0) {
            accelZ_filtered = pt1FilterApply(&psasAccelZLowpass, accelZ);
        } else {
            accelZ_filtered = accelZ;
        }
    }
    psasData.pitch.stability = (accelZ_filtered - 1.0f) * (pidProfile->psas_pitch_stability_gain * 0.1f);

    psasData.pitch.Sum = psasData.pitch.pilot + psasData.pitch.damping + psasData.pitch.stability;

    // Additional features
    // We have not got the angle of attack (AoA) sensor
    // Therefore to use lift coefficient instead of AoA. It is proportional AoA in the linear region
    float liftCoef = 0.0f;
    float liftCoefVelocity = 0.0f;
    computeLiftCoefficient(pidProfile, accelZ, &liftCoef, &liftCoefVelocity);

    // If the lift coefficent (angle of attack) is valid and its value is over limit, then limit value.
    bool isLimitAoA = false;
    if (isEnabledAoALimiter) {
        isLimitAoA = updateAngleOfAttackLimiter(pidProfile, liftCoef, liftCoefVelocity);
    }

    // Else, if the lift coefficent (angle of attack) value is normal then hold required G load (accel z) value.
    psasData.pitch.accelP = 0.0f;
    if (isEnabledAccelZController && isLimitAoA == false) {
        psasData.pitch.accelP = updateAstaticAccelZController(pidProfile, pitchStick, accelZ);
        psasData.pitch.Sum += psasData.pitch.accelP;
    }

    psasData.pitch.Sum = constrainf(psasData.pitch.Sum, -100.0f, 100.0f);

    // The AoA limiter and Accel Z controller accumulate pitch I value
    // limit integrator output and add it to Sum
    if (isEnabledAoALimiter || isEnabledAccelZController) {
        float output = psasData.pitch.Sum + psasData.pitch.I;
        if ( output > 100.0f) {
            psasData.pitch.I = 100.0f - psasData.pitch.Sum;
        } else if (output < -100.0f) {
            psasData.pitch.I = -100.0f - psasData.pitch.Sum;
        }

        // Add integrator output to Sum value
        psasData.pitch.Sum += psasData.pitch.I;
    }

    // Roll channel
    // Pilot roll control
    const float maxRcRateRoll = fmaxf(getMaxRcRate(FD_ROLL), 1.0f);
    psasData.roll.pilot = getSetpointRate(FD_ROLL) / maxRcRateRoll * (pidProfile->psas_stick_gain[FD_ROLL]);

    // Plane roll damping improvement
    psasData.roll.damping = -1.0f * gyro.gyroADCf[FD_ROLL] * (pidProfile->psas_damping_gain[FD_ROLL] * 0.001f);

    psasData.roll.Sum = psasData.roll.pilot + psasData.roll.damping;
    psasData.roll.Sum = constrainf(psasData.roll.Sum, -100.0f, 100.0f);

    // Yaw channel
    // Pilot yaw control
    const float maxRcRateYaw = fmaxf(getMaxRcRate(FD_YAW), 1.0f);
    psasData.yaw.pilot = getSetpointRate(FD_YAW) / maxRcRateYaw * (pidProfile->psas_stick_gain[FD_YAW]);

    // Plane yaw damping improvement
    float gyroYaw = gyro.gyroADCf[FD_YAW];
    if (pidProfile->psas_yaw_damping_filter_freq != 0) {
        float gyroYawLow = pt1FilterApply(&psasYawDampingLowpass, gyroYaw);
        gyroYaw -= gyroYawLow;      // Damping the yaw gyro high freq part only
    }
    psasData.yaw.damping = gyroYaw * (pidProfile->psas_damping_gain[FD_YAW] * 0.001f);

    // Plane yaw stability improvement
    float accelY_filtered = 1.0f;
    if (sensors(SENSOR_ACC)) {
        float accelY =  acc.accADC.y * acc.dev.acc_1G_rec;
        if (pidProfile->psas_accel_y_filter_freq != 0) {
            accelY_filtered = pt1FilterApply(&psasAccelYLowpass, accelY);
        } else {
            accelY_filtered = accelY;
        }
    }
    psasData.yaw.stability = accelY_filtered * (pidProfile->psas_yaw_stability_gain * 0.1f);

    // The roll rotation to yaw channel cross link to improve roll rotation on the high angle of attack flight
    psasData.yaw.rollToYawCrossLink = rollToYawCrossLinkControl(pidProfile,  psasData.roll.pilot, liftCoef);

    psasData.yaw.Sum = psasData.yaw.pilot + psasData.yaw.damping + psasData.yaw.stability + psasData.yaw.rollToYawCrossLink;
    psasData.yaw.Sum = constrainf(psasData.yaw.Sum, -100.0f, 100.0f);

    DEBUG_SET(DEBUG_PSAS, 0, lrintf(psasData.pitch.Sum * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 1, lrintf(psasData.pitch.I * 10.0f));
    DEBUG_SET(DEBUG_PSAS, 2, lrintf(liftCoef * 100.0f));
}

bool FAST_CODE_NOINLINE psasHandleMode(const pidProfile_t *pidProfile) {
    bool isPSAS = isFixedWing() && FLIGHT_MODE(AIRPLANE_SAS_MODE);
    if (isPSAS) {
        const bool psasUnsafe =
            !pidRuntime.pidStabilisationEnabled ||
            gyroOverflowDetected();
        if (psasUnsafe) {
            memset(&psasData, 0, sizeof(psasData));
            isReadyPSAS = false;
            return true;
        }

        // Clear all PID values and reset the all PSAS filters by first PSAS run
        if (!isReadyPSAS) {
            psasInit(pidProfile);
            memset(&psasData, 0, sizeof(psasData));
            isReadyPSAS = true;
        }

        psasUpdate(pidProfile);
        return true;
    } else if (isReadyPSAS) {      // Clear the all PID values after PSAS work
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;
            pidData[axis].S = 0;
            pidData[axis].Sum = 0;
        }
        isReadyPSAS = false;
        return false;
    }
    return false;
}
#endif
