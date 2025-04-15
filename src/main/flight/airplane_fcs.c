#ifdef USE_AIRPLANE_FCS

#include "fc/rc.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include <math.h>
#include "build/debug.h"

void afcsInit(const pidProfile_t *pidProfile)
{
    pt1FilterInit(&pidRuntime.afcsPitchDampingLowpass, pt1FilterGain(pidProfile->afcs_pitch_damping_filter_freq * 0.01, pidRuntime.dT));
    pt1FilterInit(&pidRuntime.afcsYawDampingLowpass, pt1FilterGain(pidProfile->afcs_yaw_damping_filter_freq * 0.01f, pidRuntime.dT));
    pidRuntime.afcsAccelError = 0.0f;
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
    if (pidProfile->afcs_pitch_accel_i_gain != 0) {
        float accelReq = pitchPilotCtrl > 0.0f ? (0.1f * pidProfile->afcs_pitch_accel_max - 1.0f) * pitchPilotCtrl * 0.01f + 1.0f
                                               : (0.1f * pidProfile->afcs_pitch_accel_min + 1.0f) * pitchPilotCtrl * 0.01f + 1.0f;
        float accelDelta = accelReq - accelZ;
        pidRuntime.afcsAccelError += accelDelta * (pidProfile->afcs_pitch_accel_i_gain * 0.1f) * pidRuntime.dT;
        float output = pidData[FD_PITCH].Sum + pidRuntime.afcsAccelError;
        if ( output > 100.0f) {
            pidRuntime.afcsAccelError = 100.0f - pidData[FD_PITCH].Sum;
        } else if (output < -100.0f) {
            pidRuntime.afcsAccelError = -100.0f - pidData[FD_PITCH].Sum;
        }
        pidData[FD_PITCH].Sum += pidRuntime.afcsAccelError;
        DEBUG_SET(DEBUG_AFCS, 3, lrintf(pidRuntime.afcsAccelError));
        DEBUG_SET(DEBUG_AFCS, 4, lrintf(pidData[FD_PITCH].Sum));
        DEBUG_SET(DEBUG_AFCS, 5, lrintf(accelReq * 10.0f));
        DEBUG_SET(DEBUG_AFCS, 6, lrintf(accelZ * 10.0f));
        DEBUG_SET(DEBUG_AFCS, 7, lrintf(accelDelta * 10.0f));
    }

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