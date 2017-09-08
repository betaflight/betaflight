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

#pragma once

#define I2C1_OVERCLOCK false
#define I2C2_OVERCLOCK false
#define USE_I2C_PULLUP          // Enable built-in pullups on all boards in case external ones are too week

#define USE_SERVOS
#define USE_CLI

#define USE_RX_PWM
#define USE_RX_PPM
#define SERIAL_RX
#define USE_SERIALRX_SPEKTRUM   // Cheap and fairly common protocol
#define USE_SERIALRX_SBUS       // Very common protocol
#define USE_SERIALRX_IBUS       // Cheap FlySky & Turnigy receivers

#if defined(STM32F1) || defined(STM32F3)
#define USE_UNDERCLOCK
#endif

#if (FLASH_SIZE > 64)
#define USE_64BIT_TIME
#define BLACKBOX
#define GPS
#define GPS_PROTO_UBLOX
#define NAV
#define USE_FLM_TURN_ASSIST     // This is mandatory for fixed-wing navigation
#define TELEMETRY
#define TELEMETRY_LTM
#define TELEMETRY_FRSKY
#endif

#if defined(STM_FAST_TARGET)
#define SCHEDULER_DELAY_LIMIT           10
#else
#define SCHEDULER_DELAY_LIMIT           100
#endif

#if (FLASH_SIZE > 128)
#define FIXED_WING_LANDING
#define AUTOTUNE_FIXED_WING
#define ASYNC_GYRO_PROCESSING
#define BOOTLOG
#define BOOTLOG_DESCRIPTIONS
#define STATS
#define USE_64BIT_TIME
#define USE_GYRO_NOTCH_1
#define USE_GYRO_NOTCH_2
#define USE_DTERM_NOTCH
#define USE_ACC_NOTCH
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define DASHBOARD_ARMED_BITMAP
#define GPS_PROTO_NMEA
#define GPS_PROTO_I2C_NAV
#define GPS_PROTO_NAZA
#define GPS_PROTO_UBLOX_NEO7PLUS
#define GPS_PROTO_MTK
#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION
#define NAV_NON_VOLATILE_WAYPOINT_STORAGE
#define TELEMETRY_HOTT
#define TELEMETRY_IBUS
#define TELEMETRY_MAVLINK
#define TELEMETRY_SMARTPORT
#define TELEMETRY_CRSF
// These are rather exotic serial protocols
#define USE_RX_MSP
#define USE_SERIALRX_SUMD
#define USE_SERIALRX_SUMH
#define USE_SERIALRX_XBUS
#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_CRSF
#define USE_PMW_SERVO_DRIVER
#define USE_SERIAL_PASSTHROUGH
#define PWM_DRIVER_PCA9685
#define NAV_MAX_WAYPOINTS       60
#define MAX_BOOTLOG_ENTRIES     64
#define USE_RCSPLIT
#define PITOT
#define USE_PITOT_ADC
#else
#define CLI_MINIMAL_VERBOSITY
#define SKIP_TASK_STATISTICS
#define SKIP_CLI_COMMAND_HELP
#define SKIP_CLI_RESOURCES
#define DISABLE_UNCOMMON_MIXERS
#define NAV_MAX_WAYPOINTS       30
#define MAX_BOOTLOG_ENTRIES     32
#endif
