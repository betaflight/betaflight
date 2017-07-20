/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// FC configuration
#define PG_CF_START 1
#define PG_FAILSAFE_CONFIG 1
#define PG_BOARD_ALIGNMENT 2
#define PG_GIMBAL_CONFIG 3
#define PG_MOTOR_MIXER 4
#define PG_BLACKBOX_CONFIG 5
#define PG_MOTOR_CONFIG 6
//#define PG_SENSOR_SELECTION_CONFIG 7 -- NOT USED in iNav
//#define PG_SENSOR_ALIGNMENT_CONFIG 8 -- NOT USED in iNav
//#define PG_SENSOR_TRIMS 9 -- NOT USED in iNav
#define PG_GYRO_CONFIG 10
#define PG_BATTERY_CONFIG 11
#define PG_CONTROL_RATE_PROFILES 12
#define PG_SERIAL_CONFIG 13
#define PG_PID_PROFILE 14
//#define PG_GTUNE_CONFIG 15
#define PG_ARMING_CONFIG 16
//#define PG_TRANSPONDER_CONFIG 17
#define PG_SYSTEM_CONFIG 18
#define PG_FEATURE_CONFIG 19
#define PG_MIXER_CONFIG 20
#define PG_SERVO_MIXER 21
#define PG_IMU_CONFIG 22
//#define PG_PROFILE_SELECTION 23
#define PG_RX_CONFIG 24
#define PG_RC_CONTROLS_CONFIG 25
#define PG_MOTOR_3D_CONFIG 26
#define PG_LED_STRIP_CONFIG 27
//#define PG_COLOR_CONFIG 28
//#define PG_AIRPLANE_ALT_HOLD_CONFIG 29
#define PG_GPS_CONFIG 30
#define PG_TELEMETRY_CONFIG 31
//#define PG_FRSKY_TELEMETRY_CONFIG 32
//#define PG_HOTT_TELEMETRY_CONFIG 33
//#define PG_NAVIGATION_CONFIG 34
#define PG_ACCELEROMETER_CONFIG 35
//#define PG_RATE_PROFILE_SELECTION 36
//#define PG_ADJUSTMENT_PROFILE 37
#define PG_ADJUSTMENT_RANGE_CONFIG 37
#define PG_BAROMETER_CONFIG 38
//#define PG_THROTTLE_CORRECTION_CONFIG 39
#define PG_COMPASS_CONFIG 40
#define PG_MODE_ACTIVATION_PROFILE 41
//#define PG_SERVO_PROFILE 42
#define PG_SERVO_PARAMS 42
//#define PG_RX_FAILSAFE_CHANNEL_CONFIG 43
#define PG_RX_CHANNEL_RANGE_CONFIG 44
//#define PG_MODE_COLOR_CONFIG 45
//#define PG_SPECIAL_COLOR_CONFIG 46
//#define PG_PILOT_CONFIG 47
//#define PG_MSP_SERVER_CONFIG 48
//#define PG_VOLTAGE_METER_CONFIG 49
//#define PG_AMPERAGE_METER_CONFIG 50
//#define PG_DEBUG_CONFIG 51
#define PG_SERVO_CONFIG 52
//#define PG_IBUS_TELEMETRY_CONFIG 53
//#define PG_VTX_CONFIG 54
#define PG_ELERES_CONFIG 55
#define PG_CF_END 56

// Driver configuration
//#define PG_DRIVER_PWM_RX_CONFIG 100
//#define PG_DRIVER_FLASHCHIP_CONFIG 101

// iNav specific parameter group ids start at 1000
#define PG_INAV_START 1000
#define PG_PITOTMETER_CONFIG 1000
#define PG_POSITION_ESTIMATION_CONFIG 1001
#define PG_NAV_CONFIG 1002
#define PG_MODE_ACTIVATION_OPERATOR_CONFIG 1003
#define PG_OSD_CONFIG 1004
#define PG_BEEPER_CONFIG 1005
#define PG_RANGEFINDER_CONFIG 1006
#define PG_WAYPOINT_MISSION_STORAGE 1007
#define PG_PID_AUTOTUNE_CONFIG 1008
#define PG_STATS_CONFIG 1009
#define PG_ADC_CHANNEL_CONFIG 1010
#define PG_INAV_END 1010

// OSD configuration (subject to change)
//#define PG_OSD_FONT_CONFIG 2047
//#define PG_OSD_VIDEO_CONFIG 2046
//#define PG_OSD_ELEMENT_CONFIG 2045

// 4095 is currently the highest number that can be used for a PGN due to the top 4 bits of the 16 bit value being reserved for the version when the PG is stored in an EEPROM.
#define PG_RESERVED_FOR_TESTING_1 4095
#define PG_RESERVED_FOR_TESTING_2 4094
#define PG_RESERVED_FOR_TESTING_3 4093

