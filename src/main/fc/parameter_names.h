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

#pragma once

#define PARAM_NAME_GYRO_HARDWARE_LPF "gyro_hardware_lpf"
#define PARAM_NAME_GYRO_LPF1_TYPE "gyro_lpf1_type"
#define PARAM_NAME_GYRO_LPF1_STATIC_HZ "gyro_lpf1_static_hz"
#define PARAM_NAME_GYRO_LPF2_TYPE "gyro_lpf2_type"
#define PARAM_NAME_GYRO_LPF2_STATIC_HZ "gyro_lpf2_static_hz"
#define PARAM_NAME_GYRO_TO_USE "gyro_to_use"
#define PARAM_NAME_DYN_NOTCH_MAX_HZ "dyn_notch_max_hz"
#define PARAM_NAME_DYN_NOTCH_COUNT "dyn_notch_count"
#define PARAM_NAME_DYN_NOTCH_Q "dyn_notch_q"
#define PARAM_NAME_DYN_NOTCH_MIN_HZ "dyn_notch_min_hz"
#define PARAM_NAME_ACC_HARDWARE "acc_hardware"
#define PARAM_NAME_ACC_LPF_HZ "acc_lpf_hz"
#define PARAM_NAME_MAG_HARDWARE "mag_hardware"
#define PARAM_NAME_BARO_HARDWARE "baro_hardware"
#define PARAM_NAME_RC_SMOOTHING "rc_smoothing"
#define PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR "rc_smoothing_auto_factor"
#define PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR_THROTTLE "rc_smoothing_auto_factor_throttle"
#define PARAM_NAME_RC_SMOOTHING_SETPOINT_CUTOFF "rc_smoothing_setpoint_cutoff"
#define PARAM_NAME_RC_SMOOTHING_FEEDFORWARD_CUTOFF "rc_smoothing_feedforward_cutoff"
#define PARAM_NAME_RC_SMOOTHING_THROTTLE_CUTOFF "rc_smoothing_throttle_cutoff"
#define PARAM_NAME_RC_SMOOTHING_DEBUG_AXIS "rc_smoothing_debug_axis"
#define PARAM_NAME_RC_SMOOTHING_ACTIVE_CUTOFFS "rc_smoothing_active_cutoffs_ff_sp_thr"
#define PARAM_NAME_SERIAL_RX_PROVIDER "serialrx_provider"
#define PARAM_NAME_DSHOT_IDLE_VALUE "dshot_idle_value"
#define PARAM_NAME_DSHOT_BIDIR "dshot_bidir"
#define PARAM_NAME_USE_UNSYNCED_PWM "use_unsynced_pwm"
#define PARAM_NAME_MOTOR_PWM_PROTOCOL "motor_pwm_protocol"
#define PARAM_NAME_MOTOR_PWM_RATE "motor_pwm_rate"
#define PARAM_NAME_MOTOR_POLES "motor_poles"
#define PARAM_NAME_THR_MID "thr_mid"
#define PARAM_NAME_THR_EXPO "thr_expo"
#define PARAM_NAME_RATES_TYPE "rates_type"
#define PARAM_NAME_TPA_RATE "tpa_rate"
#define PARAM_NAME_TPA_BREAKPOINT "tpa_breakpoint"
#define PARAM_NAME_TPA_MODE "tpa_mode"
#define PARAM_NAME_THROTTLE_LIMIT_TYPE "throttle_limit_type"
#define PARAM_NAME_THROTTLE_LIMIT_PERCENT "throttle_limit_percent"
#define PARAM_NAME_GYRO_CAL_ON_FIRST_ARM "gyro_cal_on_first_arm"
#define PARAM_NAME_DEADBAND "deadband"
#define PARAM_NAME_YAW_DEADBAND "yaw_deadband"
#define PARAM_NAME_PID_PROCESS_DENOM "pid_process_denom"
#define PARAM_NAME_DTERM_LPF1_TYPE "dterm_lpf1_type"
#define PARAM_NAME_DTERM_LPF1_STATIC_HZ "dterm_lpf1_static_hz"
#define PARAM_NAME_DTERM_LPF2_TYPE "dterm_lpf2_type"
#define PARAM_NAME_DTERM_LPF2_STATIC_HZ "dterm_lpf2_static_hz"
#define PARAM_NAME_DTERM_NOTCH_HZ "dterm_notch_hz"
#define PARAM_NAME_DTERM_NOTCH_CUTOFF "dterm_notch_cutoff"
#define PARAM_NAME_VBAT_SAG_COMPENSATION "vbat_sag_compensation"
#define PARAM_NAME_PID_AT_MIN_THROTTLE "pid_at_min_throttle"
#define PARAM_NAME_ANTI_GRAVITY_GAIN "anti_gravity_gain"
#define PARAM_NAME_ANTI_GRAVITY_CUTOFF_HZ "anti_gravity_cutoff_hz"
#define PARAM_NAME_ANTI_GRAVITY_P_GAIN "anti_gravity_p_gain"
#define PARAM_NAME_ACC_LIMIT_YAW "acc_limit_yaw"
#define PARAM_NAME_ACC_LIMIT "acc_limit"
#define PARAM_NAME_ITERM_RELAX "iterm_relax"
#define PARAM_NAME_ITERM_RELAX_TYPE "iterm_relax_type"
#define PARAM_NAME_ITERM_RELAX_CUTOFF "iterm_relax_cutoff"
#define PARAM_NAME_ITERM_WINDUP "iterm_windup"
#define PARAM_NAME_PIDSUM_LIMIT "pidsum_limit"
#define PARAM_NAME_PIDSUM_LIMIT_YAW "pidsum_limit_yaw"
#define PARAM_NAME_YAW_LOWPASS_HZ "yaw_lowpass_hz"
#define PARAM_NAME_THROTTLE_BOOST "throttle_boost"
#define PARAM_NAME_THROTTLE_BOOST_CUTOFF "throttle_boost_cutoff"
#define PARAM_NAME_ABS_CONTROL_GAIN "abs_control_gain"
#define PARAM_NAME_USE_INTEGRATED_YAW "use_integrated_yaw"
#define PARAM_NAME_D_MAX_GAIN "d_max_gain"
#define PARAM_NAME_D_MAX_ADVANCE "d_max_advance"
#define PARAM_NAME_MOTOR_OUTPUT_LIMIT "motor_output_limit"
#define PARAM_NAME_FEEDFORWARD_TRANSITION "feedforward_transition"
#define PARAM_NAME_FEEDFORWARD_AVERAGING "feedforward_averaging"
#define PARAM_NAME_FEEDFORWARD_SMOOTH_FACTOR "feedforward_smooth_factor"
#define PARAM_NAME_FEEDFORWARD_JITTER_FACTOR "feedforward_jitter_factor"
#define PARAM_NAME_FEEDFORWARD_BOOST "feedforward_boost"
#define PARAM_NAME_FEEDFORWARD_MAX_RATE_LIMIT "feedforward_max_rate_limit"
#define PARAM_NAME_DYN_IDLE_MIN_RPM "dyn_idle_min_rpm"
#define PARAM_NAME_DYN_IDLE_P_GAIN "dyn_idle_p_gain"
#define PARAM_NAME_DYN_IDLE_I_GAIN "dyn_idle_i_gain"
#define PARAM_NAME_DYN_IDLE_D_GAIN "dyn_idle_d_gain"
#define PARAM_NAME_DYN_IDLE_MAX_INCREASE "dyn_idle_max_increase"
#define PARAM_NAME_SIMPLIFIED_PIDS_MODE "simplified_pids_mode"
#define PARAM_NAME_SIMPLIFIED_MASTER_MULTIPLIER "simplified_master_multiplier"
#define PARAM_NAME_SIMPLIFIED_I_GAIN "simplified_i_gain"
#define PARAM_NAME_SIMPLIFIED_D_GAIN "simplified_d_gain"
#define PARAM_NAME_SIMPLIFIED_PI_GAIN "simplified_pi_gain"
#define PARAM_NAME_SIMPLIFIED_DMAX_GAIN "simplified_dmax_gain"
#define PARAM_NAME_SIMPLIFIED_FEEDFORWARD_GAIN "simplified_feedforward_gain"
#define PARAM_NAME_SIMPLIFIED_PITCH_D_GAIN "simplified_pitch_d_gain"
#define PARAM_NAME_SIMPLIFIED_PITCH_PI_GAIN "simplified_pitch_pi_gain"
#define PARAM_NAME_SIMPLIFIED_DTERM_FILTER "simplified_dterm_filter"
#define PARAM_NAME_SIMPLIFIED_DTERM_FILTER_MULTIPLIER "simplified_dterm_filter_multiplier"
#define PARAM_NAME_SIMPLIFIED_GYRO_FILTER "simplified_gyro_filter"
#define PARAM_NAME_SIMPLIFIED_GYRO_FILTER_MULTIPLIER "simplified_gyro_filter_multiplier"
#define PARAM_NAME_DEBUG_MODE "debug_mode"
#define PARAM_NAME_RPM_FILTER_HARMONICS "rpm_filter_harmonics"
#define PARAM_NAME_RPM_FILTER_Q "rpm_filter_q"
#define PARAM_NAME_RPM_FILTER_MIN_HZ "rpm_filter_min_hz"
#define PARAM_NAME_RPM_FILTER_FADE_RANGE_HZ "rpm_filter_fade_range_hz"
#define PARAM_NAME_RPM_FILTER_LPF_HZ "rpm_filter_lpf_hz"
#define PARAM_NAME_POSITION_ALTITUDE_SOURCE "altitude_source"
#define PARAM_NAME_POSITION_ALTITUDE_PREFER_BARO "altitude_prefer_baro"
#define PARAM_NAME_POSITION_ALTITUDE_LPF "altitude_lpf"
#define PARAM_NAME_POSITION_ALTITUDE_D_LPF "altitude_d_lpf"

#ifdef USE_GPS
#define PARAM_NAME_GPS_PROVIDER "gps_provider"
#define PARAM_NAME_GPS_SBAS_MODE "gps_sbas_mode"
#define PARAM_NAME_GPS_SBAS_INTEGRITY "gps_sbas_integrity"
#define PARAM_NAME_GPS_AUTO_CONFIG "gps_auto_config"
#define PARAM_NAME_GPS_AUTO_BAUD "gps_auto_baud"
#define PARAM_NAME_GPS_UBLOX_USE_GALILEO "gps_ublox_use_galileo"
#define PARAM_NAME_GPS_UBLOX_MODE "gps_ublox_mode"
#define PARAM_NAME_GPS_SET_HOME_POINT_ONCE "gps_set_home_point_once"
#define PARAM_NAME_GPS_USE_3D_SPEED "gps_use_3d_speed"

#ifdef USE_GPS_RESCUE
#define PARAM_NAME_GPS_RESCUE_MIN_START_DIST "gps_rescue_min_start_dist"
#define PARAM_NAME_GPS_RESCUE_ALT_MODE "gps_rescue_alt_mode"
#define PARAM_NAME_GPS_RESCUE_INITIAL_CLIMB "gps_rescue_initial_climb"
#define PARAM_NAME_GPS_RESCUE_ASCEND_RATE "gps_rescue_ascend_rate"

#define PARAM_NAME_GPS_RESCUE_RETURN_ALT "gps_rescue_return_alt"
#define PARAM_NAME_GPS_RESCUE_RETURN_SPEED "gps_rescue_ground_speed"
#define PARAM_NAME_GPS_RESCUE_PITCH_ANGLE_MAX "gps_rescue_pitch_angle_max"
#define PARAM_NAME_GPS_RESCUE_ROLL_MIX "gps_rescue_roll_mix"

#define PARAM_NAME_GPS_RESCUE_DESCENT_DIST "gps_rescue_descent_dist"
#define PARAM_NAME_GPS_RESCUE_DESCEND_RATE "gps_rescue_descend_rate"
#define PARAM_NAME_GPS_RESCUE_LANDING_ALT "gps_rescue_landing_alt"

#define PARAM_NAME_GPS_RESCUE_THROTTLE_MIN "gps_rescue_throttle_min"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_MAX "gps_rescue_throttle_max"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_HOVER "gps_rescue_throttle_hover"

#define PARAM_NAME_GPS_RESCUE_SANITY_CHECKS "gps_rescue_sanity_checks"
#define PARAM_NAME_GPS_RESCUE_MIN_SATS "gps_rescue_min_sats"
#define PARAM_NAME_GPS_RESCUE_ALLOW_ARMING_WITHOUT_FIX "gps_rescue_allow_arming_without_fix"

#define PARAM_NAME_GPS_RESCUE_THROTTLE_P "gps_rescue_throttle_p"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_I "gps_rescue_throttle_i"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_D "gps_rescue_throttle_d"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_P "gps_rescue_velocity_p"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_I "gps_rescue_velocity_i"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_D "gps_rescue_velocity_d"
#define PARAM_NAME_GPS_RESCUE_YAW_P "gps_rescue_yaw_p"

#ifdef USE_MAG
#define PARAM_NAME_GPS_RESCUE_USE_MAG "gps_rescue_use_mag"
#endif
#endif
#endif
