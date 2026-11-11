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
#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "config/config.h"
#include "config/config_reset.h"

#define PSAS_PROFILE_COUNT 4
#define MAX_PSAS_PROFILE_NAME_LENGTH 8u

typedef struct psasProfile_s {
    char profileName[MAX_PSAS_PROFILE_NAME_LENGTH + 1]; // Descriptive name for profile
    uint8_t psas_stick_gain[XYZ_AXIS_COUNT];    // Percent control output by pilot stick deflection
    uint16_t psas_damping_gain[XYZ_AXIS_COUNT]; // Percent control range addition by 1 degree per second angle rate *0.001
    uint16_t psas_pitch_damping_filter_freq;    // pitch damping filter cut freq Hz *0.01
    uint8_t psas_accel_z_filter_freq;           // accel Z filter cut freq Hz *0.1
    uint16_t psas_pitch_stability_gain;         // percent control range addition by 1g accel z change *0.1
    uint16_t psas_pitch_accel_p_gain;           // elevator for 1g Z accel difference in % *0.1
    uint8_t psas_pitch_accel_i_gain;            // elevator speed for 1g Z accel difference in %/sec
    uint8_t psas_pitch_accel_max;               // maximal positive Z accel value *0.1
    uint8_t psas_pitch_accel_min;               // maximal negative Z accel value *0.1
    uint16_t psas_yaw_damping_filter_freq;      // yaw damping filter cut freq Hz *0.01
    uint8_t psas_accel_y_filter_freq;           // accel Y filter cut freq Hz *0.1
    uint16_t psas_yaw_stability_gain;           // percent control by 1g Y accel change *0.1
    uint16_t psas_wing_load;                    // wing load (mass / WingArea) g/decimeter^2 *0.1. Set yours plane proper value to use lift coef estimation, AoA limiter, roll to yaw cross link
    uint16_t psas_air_density;                  // The current atmosphere air density [g/m^3], the MSA 1225 g/m^3 value is default. TODO: Dynamical air density computing by using baro sensors data
    uint8_t psas_lift_c_limit;                  // Limit aerodynamics lift force coefficient value *0.1
    uint8_t psas_aoa_limiter_gain;              // elevator speed for 0.1 lift force coef difference in %/sec
    uint8_t psas_lift_coef_filter_freq;         // aoa limiter lift coef filter cut freq Hz *0.1
    uint8_t psas_aoa_limiter_forecast_time;     // aoa limiter lift coef forecast time, s *0.01
    uint8_t psas_aoa_limiter_tau_return;        // aoa limiter tau value, to return zero I value output s *0.1
    uint16_t psas_servo_time;                   // minimal time of servo movement from neutral to maximum, ms
    uint8_t psas_roll_yaw_clift_start;          // Aerodynamics lift force coef to start yaw control for roll rotation  *0.1
    uint8_t psas_roll_yaw_clift_stop;           // Aerodynamics lift force coef to maximum yaw control for roll rotation  *0.1
    uint8_t psas_roll_to_yaw_link;              // The maximal yaw control value to support roll rotation, % *0.1
    uint8_t psas_speed_curve_enabled_mask;      // Enable speed curves for the damping+stability and stick gains
    uint8_t psas_speed_optimum_vref;            // Reference speed value which has optimal plane settins m/s
    uint8_t psas_speed_main_curve_power;        // Speed gain curves power for damping, stability, pitch and yaw sticks *0.1
    uint8_t psas_speed_roll_stick_curve_power;  // Speed gain curves power for roll stick *0.1
    uint16_t psas_speed_main_curve_min;         // Speed gain minimum for damping and stability *0.01
    uint16_t psas_speed_main_curve_max;         // Speed gain maximum for damping and stability *0.01
    uint16_t psas_speed_stick_curve_min;        // Speed gain minimum for sticks *0.01
    uint16_t psas_speed_stick_curve_max;        // Speed gain maximum for sticks *0.01
    uint8_t psas_speed_curve_mode;              // Speed curves mode (psasSpeedCurveMode_e enum values)
    uint8_t psas_trim_enabled_mask;             // Trimming enabled mask for roll, pitch, yaw
    uint8_t psas_trim_deadband;                 // Trim activations stick limit, *0.1 %
    uint16_t psas_trim_lowpass_time;            // Stick LowPass filters time, ms
    uint8_t psas_trim_rate;                     // Trim I term, *0.01 1/s
    uint8_t psas_trim_limit;                    // Trim output limit, %
    uint16_t psas_trim_tau_reset;               // Trim reset time, ms
} psasProfile_t;

PG_DECLARE_ARRAY(psasProfile_t, PSAS_PROFILE_COUNT, psasProfiles);
void resetPsasProfile(psasProfile_t *profile);

typedef enum {
    PSAS_OFF,
    PSAS_WORK,
    PSAS_FINISHED,
} psasState_e;

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

typedef struct psas_speed_gains_s {
    float stick[XYZ_AXIS_COUNT];
    float main[XYZ_AXIS_COUNT];
} psas_speed_gains_t;

typedef struct psasRuntime_s {
    float stick_gain[XYZ_AXIS_COUNT];
    float damping_gain[XYZ_AXIS_COUNT];
    float pitch_stability_gain;
    float pitch_accel_p_gain;
    float pitch_accel_i_gain;
    float pitch_accel_max;
    float pitch_accel_min;
    float yaw_stability_gain;
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
    psas_speed_gains_t speed_gains;
} psasRuntime_t;

void psasInit(const psasProfile_t *psasProfile);
uint8_t psasHandleMode(void);
void psasCopyProfile(uint8_t dstPsasProfileIndex, uint8_t srcPsasProfileIndex);
