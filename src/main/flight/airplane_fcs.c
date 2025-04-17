#ifdef USE_AIRPLANE_FCS

#include "fc/rc.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "io/gps.h"
#include <math.h>
#include "build/debug.h"

void afcsInit(const pidProfile_t *pidProfile)
{
    pt1FilterInit(&pidRuntime.afcsPitchDampingLowpass, pt1FilterGain(pidProfile->afcs_pitch_damping_filter_freq * 0.01, pidRuntime.dT));
    pt1FilterInit(&pidRuntime.afcsYawDampingLowpass, pt1FilterGain(pidProfile->afcs_yaw_damping_filter_freq * 0.01f, pidRuntime.dT));
    pidRuntime.afcsPitchControlErrorSum = 0.0f;
}

static float computeLiftCoefficient(const pidProfile_t *pidProfile, float speed, float accelZ)
{
    float liftC = 0.0f;
    const float speedThreshold = 2.0f;    //gps speed thresold
    const float limitLiftC = 2.0f;
    if (speed > speedThreshold) {
        const float airSpeedPressure = (0.001f * pidProfile->afcs_air_density) * sq(speed) / 2.0f;
        liftC = accelZ * (0.001f * pidProfile->afcs_wing_load) * G_ACCELERATION / airSpeedPressure;
        liftC = constrainf(liftC, -limitLiftC, limitLiftC); //limit lift force coef value for small speed to prevent unreal AoA
    }

    return liftC;
}

void FAST_CODE afcsUpdate(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        pidData[axis].P = 0;
        pidData[axis].I = 0;
        pidData[axis].D = 0;
        pidData[axis].F = 0;
        pidData[axis].S = 0;
        pidData[axis].Sum = 0;
    }

    // Pitch channel
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

    bool isLimitAoA = false;
    if (gpsSol.numSat > 5 && pidProfile->afcs_aoa_limiter_gain != 0) {
        float speed = 0.01f * gpsSol.speed3d;
        float liftCoef = computeLiftCoefficient(pidProfile, speed, accelZ);
        float limitLiftC = 0.1f * pidProfile->afcs_lift_c_limit;
        float delta;
        if (liftCoef > 0.5f) {
            delta = limitLiftC - liftCoef;
            if (delta < 0.0f) {
                isLimitAoA = true;
                pidRuntime.afcsPitchControlErrorSum += delta * (pidProfile->afcs_aoa_limiter_gain * 0.1f) * pidRuntime.dT;
            }
        } else if (liftCoef < -0.5f) {
            delta = -limitLiftC - liftCoef;
            if (delta > 0.0f) {
                isLimitAoA = true;
                pidRuntime.afcsPitchControlErrorSum += delta * (pidProfile->afcs_aoa_limiter_gain * 0.1f) * pidRuntime.dT;
            }
        }
    }

    if (isLimitAoA == false && pidProfile->afcs_pitch_accel_i_gain != 0) {
        float accelReq = pitchPilotCtrl > 0.0f ? (0.1f * pidProfile->afcs_pitch_accel_max - 1.0f) * pitchPilotCtrl * 0.01f + 1.0f
                                               : (0.1f * pidProfile->afcs_pitch_accel_min + 1.0f) * pitchPilotCtrl * 0.01f + 1.0f;
        float accelDelta = accelReq - accelZ;
        pidRuntime.afcsPitchControlErrorSum += accelDelta * (pidProfile->afcs_pitch_accel_i_gain * 0.1f) * pidRuntime.dT;
        float output = pidData[FD_PITCH].Sum + pidRuntime.afcsPitchControlErrorSum;
        if ( output > 100.0f) {
            pidRuntime.afcsPitchControlErrorSum = 100.0f - pidData[FD_PITCH].Sum;
        } else if (output < -100.0f) {
            pidRuntime.afcsPitchControlErrorSum = -100.0f - pidData[FD_PITCH].Sum;
        }

        DEBUG_SET(DEBUG_AFCS, 3, lrintf(pidRuntime.afcsPitchControlErrorSum));
        DEBUG_SET(DEBUG_AFCS, 4, lrintf(pidData[FD_PITCH].Sum));
        DEBUG_SET(DEBUG_AFCS, 5, lrintf(accelReq * 10.0f));
        DEBUG_SET(DEBUG_AFCS, 6, lrintf(accelZ * 10.0f));
        DEBUG_SET(DEBUG_AFCS, 7, lrintf(accelDelta * 10.0f));
    }
    
    pidData[FD_PITCH].Sum += pidRuntime.afcsPitchControlErrorSum;
    
    pidData[FD_PITCH].Sum = pidData[FD_PITCH].Sum / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_PITCH].F = 10.0f * pitchPilotCtrl;
    pidData[FD_PITCH].D = 10.0f * pitchDampingCtrl;
    pidData[FD_PITCH].P = 10.0f * pitchStabilityCtrl;

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
    float yawStabilityCtrl = acc.accADC.y *  acc.dev.acc_1G_rec * (pidProfile->afcs_yaw_stability_gain * 0.01f);
    pidData[FD_YAW].Sum = yawPilotCtrl + yawDampingCtrl + yawStabilityCtrl;
    pidData[FD_YAW].Sum = constrainf(pidData[FD_YAW].Sum, -100.0f, 100.0f) / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_YAW].F = 10.0f * yawPilotCtrl;
    pidData[FD_YAW].D = 10.0f * yawDampingCtrl;
    pidData[FD_YAW].P = 10.0f * yawStabilityCtrl;

    DEBUG_SET(DEBUG_AFCS, 0, lrintf(pitchPilotCtrl));
    DEBUG_SET(DEBUG_AFCS, 1, lrintf(pitchDampingCtrl));
    DEBUG_SET(DEBUG_AFCS, 2, lrintf(pitchStabilityCtrl));

}
#endif