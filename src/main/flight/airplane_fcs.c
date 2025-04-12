#ifdef USE_AIRPLANE_FCS
#include "fc/rc.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

void afcsInit(const pidProfile_t *pidProfile)
{
    pt1FilterInit(&pidRuntime.afcsPitchDampingLowpass, pt1FilterGain(pidProfile->afcs_pitch_damping_filter_freq * 0.01, pidRuntime.dT));
    pt1FilterInit(&pidRuntime.afcsYawDampingLowpass, pt1FilterGain(pidProfile->afcs_yaw_damping_filter_freq * 0.01f, pidRuntime.dT));
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
    float gyroPitchLow = pt1FilterApply(&pidRuntime.afcsPitchDampingLowpass, gyroPitch);
    float gyroPitchHigh = gyroPitch - gyroPitchLow;
    float pitchDampingCtrl = -1.0f * gyroPitchHigh * (pidProfile->afcs_damping_gain[FD_PITCH] * 0.001f);
    float pitchStabilityCtrl = (acc.accADC.z - 1.0f) * (pidProfile->afcs_pitch_stability_gain * 0.01f);
    pidData[FD_PITCH].Sum = pitchPilotCtrl + pitchDampingCtrl + pitchStabilityCtrl;
    pidData[FD_PITCH].Sum = constrainf(pidData[FD_PITCH].Sum, -100.0f, 100.0f) / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_PITCH].F = 5.0f * (pitchPilotCtrl + pidRuntime.afcsAutoTrimPosition[FD_PITCH]);
    pidData[FD_PITCH].D = 5.0f * pitchDampingCtrl;
    pidData[FD_PITCH].P = 5.0f * pitchStabilityCtrl;

    // Roll channel
    float rollPilotCtrl = getSetpointRate(FD_ROLL) / getMaxRcRate(FD_ROLL) * (pidProfile->afcs_stick_gain[FD_ROLL]);
    float rollDampingCtrl = -1.0f * gyro.gyroADCf[FD_ROLL] * (pidProfile->afcs_damping_gain[FD_ROLL] * 0.001f);
    pidData[FD_ROLL].Sum = rollPilotCtrl + rollDampingCtrl;
    pidData[FD_ROLL].Sum = constrainf(pidData[FD_ROLL].Sum, -100.0f, 100.0f) / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_ROLL].F = 5.0f * (rollPilotCtrl + pidRuntime.afcsAutoTrimPosition[FD_ROLL]);
    pidData[FD_ROLL].D = 5.0f * rollDampingCtrl;

    // Yaw channel
    float yawPilotCtrl = getSetpointRate(FD_YAW) / getMaxRcRate(FD_YAW) * (pidProfile->afcs_stick_gain[FD_YAW]);
    float gyroYaw = gyro.gyroADCf[FD_YAW];
    float gyroYawLow = pt1FilterApply(&pidRuntime.afcsYawDampingLowpass, gyroYaw);
    float gyroYawHigh = gyroYaw - gyroYawLow;
    float yawDampingCtrl = gyroYawHigh * (pidProfile->afcs_damping_gain[FD_YAW] * 0.001f);
    float yawStabilityCtrl = acc.accADC.y * (pidProfile->afcs_yaw_stability_gain * 0.01f);
    pidData[FD_YAW].Sum = yawPilotCtrl + yawDampingCtrl + yawStabilityCtrl;
    pidData[FD_YAW].Sum = constrainf(pidData[FD_YAW].Sum, -100.0f, 100.0f) / 100.0f * 500.0f;

    // Save control components instead of PID to get logging without additional variables
    pidData[FD_YAW].F = 5.0f * (yawPilotCtrl + pidRuntime.afcsAutoTrimPosition[FD_YAW]);
    pidData[FD_YAW].D = 5.0f * yawDampingCtrl;
    pidData[FD_YAW].P = 5.0f * yawStabilityCtrl;
}
#endif