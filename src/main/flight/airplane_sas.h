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

#pragma once

#include "pid.h"

typedef enum {
    LIMITER_DISABLED,
    LIMITER_NOT_READY,
    LIMITER_ON,
    LIMITER_ACTIVE,
} psasLimiterState_e;

typedef struct psas_pitch_ctrl_s {
    float pilot;
    float damping;
    float stability;
    float I;
    float accelP;
    float Sum;
    psasLimiterState_e aoaLimiterState;
} psas_pitch_ctrl_t;

typedef struct psas_roll_ctrl_s {
    float pilot;
    float damping;
    float Sum;
} psas_roll_ctrl_t;

typedef struct psas_yaw_ctrl_s {
    float pilot;
    float damping;
    float stability;
    float rollToYawCrossLink;
    float Sum;
} psas_yaw_ctrl_t;

typedef struct psas_control_s {
    psas_pitch_ctrl_t pitch;
    psas_roll_ctrl_t roll;
    psas_yaw_ctrl_t yaw;
} psas_data_t;

extern psas_data_t psasData;

typedef struct psasRuntime_s {
    float stick_gain[XYZ_AXIS_COUNT];
    float stick_speed_scale[XYZ_AXIS_COUNT];
    float damping_gain[XYZ_AXIS_COUNT];
    float damping_speed_scale[XYZ_AXIS_COUNT];
    float pitch_stability_speed_scale;
    float pitch_stability_gain;
    float pitch_accel_p_gain;
    float pitch_accel_i_gain;
    float pitch_accel_max;
    float pitch_accel_min;
    float yaw_stability_gain;
    float yaw_stability_speed_scale;
    float wing_load;
    float air_density;
    float lift_c_limit;
    float aoa_limiter_gain;
    float aoa_limiter_forecast_time;
    float aoa_limiter_tau_return;
    float servoVelocityLimit;
    float roll_yaw_clift_start;
    float roll_yaw_clift_stop;
    float roll_to_yaw_link;
} psasRuntime_t;

void psasInit(const pidProfile_t *pidProfile);
bool psasHandleMode(const pidProfile_t *pidProfile);
