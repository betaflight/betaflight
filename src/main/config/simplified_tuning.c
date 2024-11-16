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

static void calculateNewPidValues(pidProfile_t *pidProfile)
{
    const pidf_t pidDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = {
            [PID_ROLL] = PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] = PID_YAW_DEFAULT,
        };

#ifdef USE_D_MAX
    const int dMaxDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = D_MAX_DEFAULT;
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
        pidProfile->pid[axis].D = constrain(pidDefaults[axis].D * masterMultiplier * dGain * pitchDGain, 0, PID_GAIN_MAX);
        pidProfile->pid[axis].F = constrain(pidDefaults[axis].F * masterMultiplier * pitchPiGain * feedforwardGain, 0, F_GAIN_MAX);

#ifdef USE_D_MAX
        const float dMaxGain = (dMaxDefaults[axis] > 0)
            ? pidProfile->simplified_d_max_gain / 100.0f + (1 - pidProfile->simplified_d_max_gain / 100.0f) * pidDefaults[axis].D / dMaxDefaults[axis]
            : 1.0f;
        pidProfile->d_max[axis] = constrain(dMaxDefaults[axis] * masterMultiplier * dGain * pitchDGain * dMaxGain, 0, PID_GAIN_MAX);
#endif
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
}

static void calculateNewGyroFilterValues(gyroConfig_t *gyroConfig)
{
    if (gyroConfig->gyro_lpf1_dyn_min_hz) {
        gyroConfig->gyro_lpf1_dyn_min_hz = constrain(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * gyroConfig->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
        gyroConfig->gyro_lpf1_dyn_max_hz = constrain(GYRO_LPF1_DYN_MAX_HZ_DEFAULT * gyroConfig->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
    }

    if (gyroConfig->gyro_lpf1_static_hz) {
        gyroConfig->gyro_lpf1_static_hz = constrain(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * gyroConfig->simplified_gyro_filter_multiplier / 100, 0, DYN_LPF_MAX_HZ);
    }

    if (gyroConfig->gyro_lpf2_static_hz) {
        gyroConfig->gyro_lpf2_static_hz = constrain(GYRO_LPF2_HZ_DEFAULT * gyroConfig->simplified_gyro_filter_multiplier / 100, 0, LPF_MAX_HZ);
    }
}

void applySimplifiedTuningPids(pidProfile_t *pidProfile)
{
    if (pidProfile->simplified_pids_mode != PID_SIMPLIFIED_TUNING_OFF) {
        calculateNewPidValues(pidProfile);
    }
}

void applySimplifiedTuningDtermFilters(pidProfile_t *pidProfile)
{
    if (pidProfile->simplified_dterm_filter) {
        calculateNewDTermFilterValues(pidProfile);
    }
}

void applySimplifiedTuningGyroFilters(gyroConfig_t *gyroConfig)
{
    if (gyroConfig->simplified_gyro_filter) {
        calculateNewGyroFilterValues(gyroConfig);
    }
}

void applySimplifiedTuning(pidProfile_t *pidProfile, gyroConfig_t *gyroConfig)
{
    applySimplifiedTuningPids(pidProfile);
    applySimplifiedTuningDtermFilters(pidProfile);
    applySimplifiedTuningGyroFilters(gyroConfig);
}

void disableSimplifiedTuning(pidProfile_t *pidProfile, gyroConfig_t *gyroConfig)
{
    pidProfile->simplified_pids_mode = PID_SIMPLIFIED_TUNING_OFF;

    pidProfile->simplified_dterm_filter = false;

    gyroConfig->simplified_gyro_filter = false;
}
#endif // USE_SIMPLIFIED_TUNING
