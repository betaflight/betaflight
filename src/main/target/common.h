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

#define SERIAL_RX
#define USE_SERIALRX_SPEKTRUM   // Cheap and fairly common protocol
#define USE_SERIALRX_SBUS       // Very common protocol
#define USE_SERIALRX_IBUS       // Cheap FlySky & Turnigy receivers

#if (FLASH_SIZE > 64)
#define ASYNC_GYRO_PROCESSING
#define BOOTLOG
#define BLACKBOX
#define GPS
#define GPS_PROTO_UBLOX
#define NAV
#define TELEMETRY
#define TELEMETRY_LTM
#else
#define SKIP_TASK_STATISTICS
#define SKIP_CLI_COMMAND_HELP
#endif

#if (FLASH_SIZE > 128)
#define CMS
#define USE_DASHBOARD
#define USE_MSP_DISPLAYPORT
#define BOOTLOG_DESCRIPTIONS
#define DASHBOARD_ARMED_BITMAP
#define GPS_PROTO_NMEA
#define GPS_PROTO_I2C_NAV
#define GPS_PROTO_NAZA
#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_IBUS
#define TELEMETRY_MAVLINK
#define TELEMETRY_SMARTPORT
// These are rather exotic serial protocols
#define USE_SERIALRX_SUMD
#define USE_SERIALRX_SUMH
#define USE_SERIALRX_XBUS
#define USE_SERIALRX_JETIEXBUS
#define USE_PMW_SERVO_DRIVER
#define PWM_DRIVER_PCA9685
#define NAV_MAX_WAYPOINTS       60
#else
#define SKIP_CLI_COMMAND_HELP
#define SKIP_CLI_RESOURCES
#define SKIP_RX_MSP
#define DISABLE_UNCOMMON_MIXERS
#define NAV_MAX_WAYPOINTS       30
#endif
