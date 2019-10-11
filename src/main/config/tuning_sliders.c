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

#ifdef USE_TUNING_SLIDERS

#include "common/axis.h"
#include "common/maths.h"

#include "config/tuning_sliders.h"

#include "sensors/gyro.h"

static void calculateNewPidValues(pidProfile_t *pidProfile)
{
    const pidf_t pidDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = {
            [PID_ROLL] = PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_ROLL_DEFAULT, // using the same defaults as roll
            [PID_YAW] = PID_YAW_DEFAULT,
        };
    const int dMinDefaults[FLIGHT_DYNAMICS_INDEX_COUNT] = D_MIN_DEFAULT;

    // MASTER MULTIPLIER
    for (int axis = FD_ROLL; axis <= pidProfile->slider_pids_mode; ++axis) {
        pidProfile->pid[axis].P = constrain(pidDefaults[axis].P * pidProfile->slider_master_multiplier / 100, 0, PID_GAIN_MAX);
        pidProfile->pid[axis].I = constrain(pidDefaults[axis].I * pidProfile->slider_master_multiplier / 100, 0, PID_GAIN_MAX);
        pidProfile->pid[axis].D = constrain(pidDefaults[axis].D * pidProfile->slider_master_multiplier / 100, 0, PID_GAIN_MAX);
        pidProfile->d_min[axis] = constrain(dMinDefaults[axis] * pidProfile->slider_master_multiplier / 100, 0, D_MIN_GAIN_MAX);
        pidProfile->pid[axis].F = constrain(pidDefaults[axis].F * pidProfile->slider_master_multiplier / 100, 0, F_GAIN_MAX);
    }

    // ROLL PITCH RATIO
    pidProfile->pid[FD_ROLL].P = constrain(pidProfile->pid[FD_ROLL].P * pidProfile->slider_roll_pitch_ratio / 100, 0, PID_GAIN_MAX);
    pidProfile->pid[FD_ROLL].I = constrain(pidProfile->pid[FD_ROLL].I * pidProfile->slider_roll_pitch_ratio / 100, 0, PID_GAIN_MAX);
    pidProfile->pid[FD_ROLL].D = constrain(pidProfile->pid[FD_ROLL].D * pidProfile->slider_roll_pitch_ratio / 100, 0, PID_GAIN_MAX);
    pidProfile->d_min[FD_ROLL] = constrain(pidProfile->d_min[FD_ROLL] * pidProfile->slider_roll_pitch_ratio / 100, 0, D_MIN_GAIN_MAX);
    pidProfile->pid[FD_ROLL].F = constrain(pidProfile->pid[FD_ROLL].F * pidProfile->slider_roll_pitch_ratio / 100, 0, F_GAIN_MAX);

    // I GAIN
    for (int axis = FD_ROLL; axis <= pidProfile->slider_pids_mode; ++axis) {
        pidProfile->pid[axis].I = constrain(pidProfile->pid[axis].I * pidProfile->slider_i_gain / 100, 0, PID_GAIN_MAX);
    }

    // PD RATIO - applied only on roll and pitch because on yaw default D gain is 0
    const float defaultPDRatio = pidDefaults[FD_ROLL].P / (float)pidDefaults[FD_ROLL].D;
    for (int axis = FD_ROLL; axis <= FD_PITCH; ++axis) {
        pidProfile->pid[axis].P = constrain(pidProfile->pid[axis].D * defaultPDRatio * pidProfile->slider_pd_ratio / 100, 0, PID_GAIN_MAX);
    }

    // PD GAIN
    for (int axis = FD_ROLL; axis <= pidProfile->slider_pids_mode; ++axis) {
        pidProfile->pid[axis].P = constrain(pidProfile->pid[axis].P * pidProfile->slider_pd_gain / 100, 0, PID_GAIN_MAX);
        pidProfile->pid[axis].D = constrain(pidProfile->pid[axis].D * pidProfile->slider_pd_gain / 100, 0, PID_GAIN_MAX);
        pidProfile->d_min[axis] = constrain(pidProfile->d_min[axis] * pidProfile->slider_pd_gain / 100, 0, D_MIN_GAIN_MAX);
    }

    // D MIN RATIO
    const float defaultDMinRatio = dMinDefaults[FD_ROLL] / (float)pidDefaults[FD_ROLL].D;
    const float scaledDminRatioSliderValue = scaleRangef(pidProfile->slider_dmin_ratio, SLIDER_MIN, SLIDER_MAX, SLIDER_MIN + SLIDER_MAX - 100 / defaultDMinRatio, 100 / defaultDMinRatio);
    for (int axis = FD_ROLL; axis <= pidProfile->slider_pids_mode; ++axis) {
        if (pidProfile->slider_dmin_ratio == 200) {
            pidProfile->d_min[axis] = 0; // disable d_min if slider maxed out
        } else {
            pidProfile->d_min[axis] = constrain(constrain(pidProfile->pid[axis].D * defaultDMinRatio * scaledDminRatioSliderValue / 100, 0, pidProfile->pid[axis].D - 1), 0, D_MIN_GAIN_MAX);
        }
    }

    // FF GAIN
    for (int axis = FD_ROLL; axis <= pidProfile->slider_pids_mode; ++axis) {
        pidProfile->pid[axis].F = constrain(pidProfile->pid[axis].F * pidProfile->slider_ff_gain / 100, 0, F_GAIN_MAX);
    }
}

static void calculateNewDTermFilterValues(pidProfile_t *pidProfile)
{
    pidProfile->dyn_lpf_dterm_min_hz = constrain(DYN_LPF_DTERM_MIN_HZ_DEFAULT * pidProfile->slider_dterm_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    pidProfile->dyn_lpf_dterm_max_hz = constrain(DYN_LPF_DTERM_MAX_HZ_DEFAULT * pidProfile->slider_dterm_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    pidProfile->dterm_lowpass2_hz = constrain(DTERM_LOWPASS_2_HZ_DEFAULT * pidProfile->slider_dterm_filter_multiplier / 100, 0, FILTER_FREQUENCY_MAX);
    pidProfile->dterm_filter_type = FILTER_PT1;
    pidProfile->dterm_filter2_type = FILTER_PT1;
}

static void calculateNewGyroFilterValues()
{
    gyroConfigMutable()->dyn_lpf_gyro_min_hz = constrain(DYN_LPF_GYRO_MIN_HZ_DEFAULT * gyroConfig()->slider_gyro_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    gyroConfigMutable()->dyn_lpf_gyro_max_hz = constrain(DYN_LPF_GYRO_MAX_HZ_DEFAULT * gyroConfig()->slider_gyro_filter_multiplier / 100, 0, DYN_LPF_FILTER_FREQUENCY_MAX);
    gyroConfigMutable()->gyro_lowpass2_hz = constrain(GYRO_LOWPASS_2_HZ_DEFAULT * gyroConfig()->slider_gyro_filter_multiplier / 100, 0, FILTER_FREQUENCY_MAX);
    gyroConfigMutable()->gyro_lowpass_type = FILTER_PT1;
    gyroConfigMutable()->gyro_lowpass2_type = FILTER_PT1;
}

void applyTuningSliders(pidProfile_t *pidProfile)
{
    if (pidProfile->slider_pids_mode != PID_TUNING_SLIDERS_OFF) {
        calculateNewPidValues(pidProfile);
    }

    if (pidProfile->slider_dterm_filter) {
        calculateNewDTermFilterValues(pidProfile);
    }

    if (gyroConfig()->slider_gyro_filter) {
        calculateNewGyroFilterValues();
    }
}
#endif // USE_TUNING_SLIDERS
