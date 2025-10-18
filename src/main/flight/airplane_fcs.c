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

#ifdef USE_AIRPLANE_FCS

#include "fc/rc.h"
#include "fc/runtime_config.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "io/gps.h"
#include <math.h>
#include "build/debug.h"

void afcsInit(const pidProfile_t *pidProfile)
{
    pt1FilterInit(&pidRuntime.afcsPitchDampingLowpass, pt1FilterGain(pidProfile->afcs_pitch_damping_filter_freq * 0.01, pidRuntime.dT));
    pt1FilterInit(&pidRuntime.afcsYawDampingLowpass, pt1FilterGain(pidProfile->afcs_yaw_damping_filter_freq * 0.01f, pidRuntime.dT));
    pt1FilterInit(&pidRuntime.afcsLiftCoefLowpass, pt1FilterGain(pidProfile->afcs_aoa_limiter_filter_freq * 0.1f, pidRuntime.dT));
    pidRuntime.isReadyAFCS = false;
}

static bool computeLiftCoefficient(const pidProfile_t *pidProfile, float accelZ, float *liftCoef)
{
    static bool isLiftCoefValid = false;
    static float validLiftCoefTime = 0.0f;
    const float timeForValid = 3.0f;
    *liftCoef = 0.0f;
    if (ARMING_FLAG(ARMED) && gpsSol.numSat > 5) {
        const float limitLiftC = 0.1f * pidProfile->afcs_lift_c_limit;
        const float speedThreshold = 2.5f;    //gps speed thresold
        float speed = 0.01f * gpsSol.speed3d;
        if (speed > speedThreshold) {
            const float airSpeedPressure = (0.001f * pidProfile->afcs_air_density) * sq(speed) / 2.0f;
            *liftCoef = accelZ * (0.01f * pidProfile->afcs_wing_load) * G_ACCELERATION / airSpeedPressure;

            // Enable AoA limiter after 3s flight with good lift coef. It prevents issues during plane launch
            if (isLiftCoefValid == false) {
                if (*liftCoef < limitLiftC && *liftCoef > -limitLiftC) {
                    validLiftCoefTime += pidRuntime.dT;
                    if (validLiftCoefTime > timeForValid) {
                        isLiftCoefValid = true;
                    }
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

    return isLiftCoefValid;
}

//The astatic accel Z controller by stick position
static void updateAstaticAccelZController(const pidProfile_t *pidProfile, float pitchPilotCtrl, float accelZ)
{
    if (pidProfile->afcs_pitch_accel_i_gain != 0) {
        const float servoVelocityLimit = 100.0f / (pidProfile->afcs_servo_time * 0.001f); // Limit servo velocity %/s
        float accelReq = pitchPilotCtrl < 0.0f ? (1.0f - 0.1f * pidProfile->afcs_pitch_accel_max) * pitchPilotCtrl * 0.01f + 1.0f
                                               : -(1.0f + 0.1f * pidProfile->afcs_pitch_accel_min) * pitchPilotCtrl * 0.01f + 1.0f;
        float accelDelta = accelZ - accelReq;
        float servoVelocity = accelDelta * (pidProfile->afcs_pitch_accel_i_gain * 0.1f);
        servoVelocity = constrainf(servoVelocity, -servoVelocityLimit, servoVelocityLimit);
        pidData[FD_PITCH].I += servoVelocity * pidRuntime.dT;

        // limit integrator output
        float output = pidData[FD_PITCH].Sum + pidData[FD_PITCH].I;
        if ( output > 100.0f) {
            pidData[FD_PITCH].I = 100.0f - pidData[FD_PITCH].Sum;
        } else if (output < -100.0f) {
            pidData[FD_PITCH].I = -100.0f - pidData[FD_PITCH].Sum;
        }

        DEBUG_SET(DEBUG_AFCS, 0, lrintf(accelReq * 10.0f));
        DEBUG_SET(DEBUG_AFCS, 1, lrintf(accelDelta * 10.0f));
    }
}

// The angle of attack limiter. The aerodynamics lift force coefficient depends by angle of attack. Therefore it possible to use this coef instead of AoA value.
static bool updateAngleOfAttackLimiter(const pidProfile_t *pidProfile, float liftCoef)
{
    bool isLimitAoA = false;
    static float liftCoefLast = 0.0f;
    float liftCoefF = pt1FilterApply(&pidRuntime.afcsLiftCoefLowpass, liftCoef);
    float liftCoefVelocity = (liftCoefF - liftCoefLast) / pidRuntime.dT;
    liftCoefLast = liftCoefF;
    float liftCoefForcastChange = liftCoefVelocity * (pidProfile->afcs_aoa_limiter_forcast_time * 0.1f);

    if (pidProfile->afcs_aoa_limiter_gain != 0) {
        const float limitLiftC = 0.1f * pidProfile->afcs_lift_c_limit;

        const float servoVelocityLimit = 100.0f / (pidProfile->afcs_servo_time * 0.001f); // Limit servo velocity %/s
        float liftCoefDiff = 0.0f,
              servoVelocity = 0.0f;
        if (liftCoefF > 0.0f) {
            liftCoefDiff = liftCoefF - limitLiftC;
            if (liftCoefForcastChange > 0.0f) {
                liftCoefDiff += liftCoefForcastChange;
            }
            if (liftCoefDiff > 0.0f) {
                isLimitAoA = true;
                servoVelocity = liftCoefDiff * (pidProfile->afcs_aoa_limiter_gain * 0.1f);
                servoVelocity = constrainf(servoVelocity, -servoVelocityLimit, servoVelocityLimit);
                pidData[FD_PITCH].I += servoVelocity * pidRuntime.dT;
            }
        } else {
            liftCoefDiff = liftCoefF + limitLiftC;
            if (liftCoefForcastChange < 0.0f) {
                liftCoefDiff += liftCoefForcastChange;
            }
            if (liftCoefDiff < 0.0f) {
                isLimitAoA = true;
                servoVelocity = liftCoefDiff * (pidProfile->afcs_aoa_limiter_gain * 0.1f);
                servoVelocity = constrainf(servoVelocity, -servoVelocityLimit, servoVelocityLimit);
                pidData[FD_PITCH].I += servoVelocity * pidRuntime.dT;
            }
        }
        DEBUG_SET(DEBUG_AFCS, 3, lrintf(liftCoefF * 100.0f));
        DEBUG_SET(DEBUG_AFCS, 4, lrintf(liftCoefDiff * 100.0f));
    }

    return isLimitAoA;
}

// Roll to yaw control cross link to improve roll rotation at high angle of attack
static float rollToYawCrossLinkControl(const pidProfile_t *pidProfile, float rollPilotControl, float liftCoef)
{
    float crossYawControl = 0.0f,
          roll_yaw_clift_start = 0.1f * pidProfile->afcs_roll_yaw_clift_start,
          roll_yaw_clift_stop = 0.1f * pidProfile->afcs_roll_yaw_clift_stop;

    if (pidProfile->afcs_roll_to_yaw_link && liftCoef > roll_yaw_clift_start) {
        float k = (liftCoef - roll_yaw_clift_start) / (roll_yaw_clift_stop - roll_yaw_clift_start);
        k = constrainf(k, 0.0f, 1.0f);
        crossYawControl = k * (0.01f * rollPilotControl) * (pidProfile->afcs_roll_to_yaw_link * 0.1f);
    }

    return crossYawControl;
}

void FAST_CODE afcsUpdate(const pidProfile_t *pidProfile)
{
    // Clear all PID values by first AFCS run
    if (!pidRuntime.isReadyAFCS) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;
            pidData[axis].S = 0;
            pidData[axis].Sum = 0;
        }
        pidRuntime.isReadyAFCS = true;
    }

    // Pitch channel
    pidData[FD_PITCH].I /= 10.0f;   //restore % last value
    float pitchPilotCtrl = getSetpointRate(FD_PITCH) / getMaxRcRate(FD_PITCH) * pidProfile->afcs_stick_gain[FD_PITCH];
    float gyroPitch = gyro.gyroADCf[FD_PITCH];
    if (pidProfile->afcs_pitch_damping_filter_freq != 0) {
        float gyroPitchLow = pt1FilterApply(&pidRuntime.afcsPitchDampingLowpass, gyroPitch);
        gyroPitch -= gyroPitchLow;      // Damping the pitch gyro high freq part only
    }
    float pitchDampingCtrl = -1.0f * gyroPitch * (pidProfile->afcs_damping_gain[FD_PITCH] * 0.001f);
    float accelZ = acc.accADC.z * acc.dev.acc_1G_rec;
    float pitchStabilityCtrl = (accelZ - 1.0f) * (pidProfile->afcs_pitch_stability_gain * 0.01f);

    pidData[FD_PITCH].Sum = pitchPilotCtrl + pitchDampingCtrl + pitchStabilityCtrl;
    pidData[FD_PITCH].Sum = constrainf(pidData[FD_PITCH].Sum, -100.0f, 100.0f);

    // Hold required accel z value. If it is unpossible due of big angle of attack value, then limit angle of attack
    float liftCoef = 0.0f;
    bool isValidLiftCoef = computeLiftCoefficient(pidProfile, accelZ, &liftCoef);
    bool isLimitAoA = false;
    if (isValidLiftCoef) {
        isLimitAoA = updateAngleOfAttackLimiter(pidProfile, liftCoef);
    }
    if (isLimitAoA == false) {
        updateAstaticAccelZController(pidProfile, pitchPilotCtrl, accelZ);
    }
    pidData[FD_PITCH].Sum += pidData[FD_PITCH].I;

    pidData[FD_PITCH].Sum = pidData[FD_PITCH].Sum / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_PITCH].F = 10.0f * pitchPilotCtrl;
    pidData[FD_PITCH].D = 10.0f * pitchDampingCtrl;
    pidData[FD_PITCH].S = 10.0f * pitchStabilityCtrl;
    pidData[FD_PITCH].I *= 10.0f;  // Store *10 value

    // Roll channel
    float rollPilotCtrl = getSetpointRate(FD_ROLL) / getMaxRcRate(FD_ROLL) * (pidProfile->afcs_stick_gain[FD_ROLL]);
    float rollDampingCtrl = -1.0f * gyro.gyroADCf[FD_ROLL] * (pidProfile->afcs_damping_gain[FD_ROLL] * 0.001f);
    pidData[FD_ROLL].Sum = rollPilotCtrl + rollDampingCtrl;
    pidData[FD_ROLL].Sum = constrainf(pidData[FD_ROLL].Sum, -100.0f, 100.0f) / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_ROLL].F = 10.0f * rollPilotCtrl;
    pidData[FD_ROLL].D = 10.0f * rollDampingCtrl;

    // Yaw channel
    float yawPilotCtrl = getSetpointRate(FD_YAW) / getMaxRcRate(FD_YAW) * (pidProfile->afcs_stick_gain[FD_YAW]);
    float gyroYaw = gyro.gyroADCf[FD_YAW];
    if (pidProfile->afcs_yaw_damping_filter_freq != 0) {
        float gyroYawLow = pt1FilterApply(&pidRuntime.afcsYawDampingLowpass, gyroYaw);
        gyroYaw -= gyroYawLow;      // Damping the yaw gyro high freq part only
    }
    float yawDampingCtrl = gyroYaw * (pidProfile->afcs_damping_gain[FD_YAW] * 0.001f);
    float yawStabilityCtrl = acc.accADC.y * acc.dev.acc_1G_rec * (pidProfile->afcs_yaw_stability_gain * 0.01f);
    float rollToYawCrossControl = 0.0f;
    if (isValidLiftCoef) {
        rollToYawCrossControl = rollToYawCrossLinkControl(pidProfile, rollPilotCtrl, liftCoef);
    }
    pidData[FD_YAW].Sum = yawPilotCtrl + yawDampingCtrl + yawStabilityCtrl + rollToYawCrossControl;
    pidData[FD_YAW].Sum = constrainf(pidData[FD_YAW].Sum, -100.0f, 100.0f) / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_YAW].F = 10.0f * yawPilotCtrl;
    pidData[FD_YAW].D = 10.0f * yawDampingCtrl;
    pidData[FD_YAW].S = 10.0f * yawStabilityCtrl;
    pidData[FD_YAW].P = 10.0f * rollToYawCrossControl;

    DEBUG_SET(DEBUG_AFCS, 2, lrintf(liftCoef * 100.0f));
    DEBUG_SET(DEBUG_AFCS, 5, lrintf(pidData[FD_PITCH].I));
    DEBUG_SET(DEBUG_AFCS, 6, lrintf(pidData[FD_YAW].P));
}
#endif
