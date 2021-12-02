/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include "platform.h"

#ifdef USE_SIMPLIFIED_TUNING

#include "common/axis.h"
#include "common/maths.h"

#include "config/simplified_tuning.h"

#include "sensors/gyro.h"

static void calculateNewPidValues(pidProfile_t *pidProfile)
{
    const pidf_t pidDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = {
            [PID_ROLL] = PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] = PID_YAW_DEFAULT,
        };

#ifdef USE_D_MIN
    const int dMinDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = D_MIN_DEFAULT;
#endif
    const float masterMultiplier = pidProfile->simplified_master_multiplier / 100.0f;
    const float piGain = pidProfile->simplified_pi_gain / 100.0f;
    const float dGain = pidProfile->simplified_d_gain / 100.0f;
    const float feedforwardGain = pidProfile->simplified_feedforward_gain / 100.0f;
    const float iGain = pidProfile->simplified_i_gain / 100.0f;

    for (int axis = FD_ROLL; axis <= pidProfile->simplified_pids_mode; ++axis) {
        const float pitchDGain = (axis == FD_PITCH) ? pidProfile->simplified_roll_pitch_ratio / 100.0f : 1.0f;
        const float pitchPiGain = (axis == FD_PITCH) ? pidProfile->simplified_pitch_pi_gain / 100.0f : 1.0f;
        pidProfile->pid[axis].P = constrain(pidDefaults[axis].P * masterMultiplier * piGain * pitchPiGain, 0, PID_GAIN_MAX);
        pidProfile->pid[axis].I = constrain(pidDefaults[axis].I * masterMultiplier * piGain * iGain * pitchPiGain, 0, PID_GAIN_MAX);
#ifdef USE_D_MIN
        const float dminRatio = (dMinDefaults[axis] > 0) ? 1.0f + (((float)pidDefaults[axis].D - dMinDefaults[axis]) / dMinDefaults[axis]) * (pidProfile->simplified_dmin_ratio / 100.0f) : 1.0f;
        pidProfile->pid[axis].D = constrain(dMinDefaults[axis] * masterMultiplier * dGain * pitchDGain * dminRatio, 0, PID_GAIN_MAX);
        pidProfile->d_min[axis] = constrain(dMinDefaults[axis] * masterMultiplier * dGain * pitchDGain, 0, PID_GAIN_MAX);
#else
        pidProfile->pid[axis].D = constrain(dMinDefaults[axis] * masterMultiplier * dGain * pitchDGain, 0, PID_GAIN_MAX);
#endif
        pidProfile->pid[axis].F = constrain(pidDefaults[axis].F * masterMultiplier * pitchPiGain * feedforwardGain, 0, F_GAIN_MAX);
    }
}

static void calculateNewDTermFilterValues(pidProfile_t *pidProfile)
{
    if (pidProfile->dterm_lpf1_dyn_min_hz) {
        pidProfile->dterm_lpf1_dyn_min_hz = constrain(DTERM_LPF1_DYN_MIN_HZ_DEFAULT * pidProfile->simplified_dterm_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
        pidProfile->dterm_lpf1_dyn_max_hz = constrain(DTERM_LPF1_DYN_MAX_HZ_DEFAULT * pidProfile->simplified_dterm_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
    }

    if (pidProfile->dterm_lpf1_static_hz) {
        pidProfile->dterm_lpf1_static_hz = constrain(DTERM_LPF1_DYN_MIN_HZ_DEFAULT * pidProfile->simplified_dterm_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
    }

    if (pidProfile->dterm_lpf2_static_hz) {
        pidProfile->dterm_lpf2_static_hz = constrain(DTERM_LPF2_HZ_DEFAULT * pidProfile->simplified_dterm_filter_multiplier / 100, 0, LPF_MAX_HZ);
    }

    if (!pidProfile->dterm_lpf1_type) {
        pidProfile->dterm_lpf1_type = FILTER_PT1;
    }

    if (!pidProfile->dterm_lpf2_type) {
        pidProfile->dterm_lpf2_type = FILTER_PT1;
    }
}

static void calculateNewGyroFilterValues()
{
    if (gyroConfigMutable()->gyro_lpf1_dyn_min_hz) {
        gyroConfigMutable()->gyro_lpf1_dyn_min_hz = constrain(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * gyroConfig()->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
        gyroConfigMutable()->gyro_lpf1_dyn_max_hz = constrain(GYRO_LPF1_DYN_MAX_HZ_DEFAULT * gyroConfig()->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
    }

    if (gyroConfigMutable()->gyro_lpf1_static_hz) {
        gyroConfigMutable()->gyro_lpf1_static_hz = constrain(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * gyroConfig()->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
    }

    if (gyroConfigMutable()->gyro_lpf2_static_hz) {
        gyroConfigMutable()->gyro_lpf2_static_hz = constrain(GYRO_LPF2_HZ_DEFAULT * gyroConfig()->simplified_gyro_filter_multiplier / 100, 0, LPF_MAX_HZ);
    }

    if (!gyroConfigMutable()->gyro_lpf1_type) {
        gyroConfigMutable()->gyro_lpf1_type = FILTER_PT1;
    }

    if (!gyroConfigMutable()->gyro_lpf2_type) {
        gyroConfigMutable()->gyro_lpf2_type = FILTER_PT1;
    }
}

void applySimplifiedTuning(pidProfile_t *pidProfile)
{
    if (pidProfile->simplified_pids_mode != PID_SIMPLIFIED_TUNING_OFF) {
        calculateNewPidValues(pidProfile);
    }

    if (pidProfile->simplified_dterm_filter) {
        calculateNewDTermFilterValues(pidProfile);
    }

    if (gyroConfig()->simplified_gyro_filter) {
        calculateNewGyroFilterValues();
    }
}

void disableSimplifiedTuning(pidProfile_t *pidProfile)
{
    pidProfile->simplified_pids_mode = PID_SIMPLIFIED_TUNING_OFF;

    pidProfile->simplified_dterm_filter = false;

    gyroConfigMutable()->simplified_gyro_filter = false;
}
#endif // USE_SIMPLIFIED_TUNING
