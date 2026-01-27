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

/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

#pragma once

// Protocol numbers used both by the wire format, config system, and field setters.
#define MSP_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1
#define API_VERSION_MINOR                   48
#define API_VERSION_LENGTH                  2

#define MULTIWII_IDENTIFIER "MWII";
#define BASEFLIGHT_IDENTIFIER "BAFL";
//#define BETAFLIGHT_IDENTIFIER "BTFL" Actual value stored in FC_FIRMWARE_IDENTIFIER in build/version.h
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define INAV_IDENTIFIER "INAV"
#define RACEFLIGHT_IDENTIFIER "RCFL"

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

// These are baseflight specific flags but they are useless now since MW 2.3 uses the upper 4 bits for the navigation version.
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)

// MW 2.3 stores NAVI_VERSION in the top 4 bits of the capability mask.
#define CAP_NAVI_VERSION_BIT_4_MSB  ((uint32_t)1 << 31)
#define CAP_NAVI_VERSION_BIT_3      ((uint32_t)1 << 30)
#define CAP_NAVI_VERSION_BIT_2      ((uint32_t)1 << 29)
#define CAP_NAVI_VERSION_BIT_1_LSB  ((uint32_t)1 << 28)

#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FLAPS                   ((uint32_t)1 << 3)
#define CAP_NAVCAP                  ((uint32_t)1 << 4)
#define CAP_EXTAUX                  ((uint32_t)1 << 5)

#define MSP_API_VERSION                 1    // out message: Get API version
#define MSP_FC_VARIANT                  2    // out message: Get flight controller variant
#define MSP_FC_VERSION                  3    // out message: Get flight controller version
#define MSP_BOARD_INFO                  4    // out message: Get board information
#define MSP_BUILD_INFO                  5    // out message: Get build information

#define MSP_NAME                        10   // out message: Returns user set board name - betaflight
#define MSP_SET_NAME                    11   // in message:  Sets board name - betaflight

// Cleanflight original features (32-62)
#define MSP_BATTERY_CONFIG              32   // out message: Get battery configuration
#define MSP_SET_BATTERY_CONFIG          33   // in message:  Set battery configuration
#define MSP_MODE_RANGES                 34   // out message: Returns all mode ranges
#define MSP_SET_MODE_RANGE              35   // in message:  Sets a single mode range
#define MSP_FEATURE_CONFIG              36   // out message: Get feature configuration
#define MSP_SET_FEATURE_CONFIG          37   // in message:  Set feature configuration
#define MSP_BOARD_ALIGNMENT_CONFIG      38   // out message: Get board alignment configuration
#define MSP_SET_BOARD_ALIGNMENT_CONFIG  39   // in message:  Set board alignment configuration
#define MSP_CURRENT_METER_CONFIG        40   // out message: Get current meter configuration
#define MSP_SET_CURRENT_METER_CONFIG    41   // in message:  Set current meter configuration
#define MSP_MIXER_CONFIG                42   // out message: Get mixer configuration
#define MSP_SET_MIXER_CONFIG            43   // in message:  Set mixer configuration
#define MSP_RX_CONFIG                   44   // out message: Get RX configuration
#define MSP_SET_RX_CONFIG               45   // in message:  Set RX configuration
#define MSP_LED_COLORS                  46   // out message: Get LED colors
#define MSP_SET_LED_COLORS              47   // in message:  Set LED colors
#define MSP_LED_STRIP_CONFIG            48   // out message: Get LED strip configuration
#define MSP_SET_LED_STRIP_CONFIG        49   // in message:  Set LED strip configuration
#define MSP_RSSI_CONFIG                 50   // out message: Get RSSI configuration
#define MSP_SET_RSSI_CONFIG             51   // in message:  Set RSSI configuration
#define MSP_ADJUSTMENT_RANGES           52   // out message: Get adjustment ranges
#define MSP_SET_ADJUSTMENT_RANGE        53   // in message:  Set adjustment range
#define MSP_CF_SERIAL_CONFIG            54   // out message: Get Cleanflight serial configuration
#define MSP_SET_CF_SERIAL_CONFIG        55   // in message:  Set Cleanflight serial configuration
#define MSP_VOLTAGE_METER_CONFIG        56   // out message: Get voltage meter configuration
#define MSP_SET_VOLTAGE_METER_CONFIG    57   // in message:  Set voltage meter configuration
#define MSP_SONAR_ALTITUDE              58   // out message: Get sonar altitude [cm]
#define MSP_PID_CONTROLLER              59   // out message: Get PID controller
#define MSP_SET_PID_CONTROLLER          60   // in message:  Set PID controller
#define MSP_ARMING_CONFIG               61   // out message: Get arming configuration
#define MSP_SET_ARMING_CONFIG           62   // in message:  Set arming configuration

// Baseflight MSP commands (64-89)
#define MSP_RX_MAP                      64   // out message: Get RX map (also returns number of channels total)
#define MSP_SET_RX_MAP                  65   // in message:  Set RX map, numchannels to set comes from MSP_RX_MAP
#define MSP_REBOOT                      68   // in message:  Reboot settings
#define MSP_DATAFLASH_SUMMARY           70   // out message: Get description of dataflash chip
#define MSP_DATAFLASH_READ              71   // out message: Get content of dataflash chip
#define MSP_DATAFLASH_ERASE             72   // in message:  Erase dataflash chip
#define MSP_FAILSAFE_CONFIG             75   // out message: Get failsafe settings
#define MSP_SET_FAILSAFE_CONFIG         76   // in message:  Set failsafe settings
#define MSP_RXFAIL_CONFIG               77   // out message: Get RX failsafe settings
#define MSP_SET_RXFAIL_CONFIG           78   // in message:  Set RX failsafe settings
#define MSP_SDCARD_SUMMARY              79   // out message: Get SD card state
#define MSP_BLACKBOX_CONFIG             80   // out message: Get blackbox settings
#define MSP_SET_BLACKBOX_CONFIG         81   // in message:  Set blackbox settings
#define MSP_TRANSPONDER_CONFIG          82   // out message: Get transponder settings
#define MSP_SET_TRANSPONDER_CONFIG      83   // in message:  Set transponder settings
#define MSP_OSD_CONFIG                  84   // out message: Get OSD settings
#define MSP_SET_OSD_CONFIG              85   // in message:  Set OSD settings
#define MSP_OSD_CHAR_READ               86   // out message: Get OSD characters
#define MSP_OSD_CHAR_WRITE              87   // in message:  Set OSD characters
#define MSP_VTX_CONFIG                  88   // out message: Get VTX settings
#define MSP_SET_VTX_CONFIG              89   // in message:  Set VTX settings

// Betaflight Additional Commands (90-99)
#define MSP_ADVANCED_CONFIG             90   // out message: Get advanced configuration
#define MSP_SET_ADVANCED_CONFIG         91   // in message:  Set advanced configuration
#define MSP_FILTER_CONFIG               92   // out message: Get filter configuration
#define MSP_SET_FILTER_CONFIG           93   // in message:  Set filter configuration
#define MSP_PID_ADVANCED                94   // out message: Get advanced PID settings
#define MSP_SET_PID_ADVANCED            95   // in message:  Set advanced PID settings
#define MSP_SENSOR_CONFIG               96   // out message: Get sensor configuration
#define MSP_SET_SENSOR_CONFIG           97   // in message:  Set sensor configuration
#define MSP_CAMERA_CONTROL              98   // in/out message: Camera control
#define MSP_SET_ARMING_DISABLED         99   // in message:  Enable/disable arming

// Multiwii original MSP commands (101-139)
#define MSP_STATUS                      101  // out message: Cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU                     102  // out message: 9 DOF
#define MSP_SERVO                       103  // out message: Servos
#define MSP_MOTOR                       104  // out message: Motors
#define MSP_RC                          105  // out message: RC channels and more
#define MSP_RAW_GPS                     106  // out message: Fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS                    107  // out message: Distance home, direction home
#define MSP_ATTITUDE                    108  // out message: 2 angles 1 heading
#define MSP_ALTITUDE                    109  // out message: Altitude, variometer
#define MSP_ANALOG                      110  // out message: Vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING                   111  // out message: RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                         112  // out message: P I D coeff (9 are used currently)
#define MSP_BOXNAMES                    116  // out message: The aux switch names
#define MSP_PIDNAMES                    117  // out message: The PID names
#define MSP_WP                          118  // out message: Get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS                      119  // out message: Get the permanent IDs associated to BOXes
#define MSP_SERVO_CONFIGURATIONS        120  // out message: All servo configurations
#define MSP_NAV_STATUS                  121  // out message: Returns navigation status
#define MSP_NAV_CONFIG                  122  // out message: Returns navigation parameters
#define MSP_MOTOR_3D_CONFIG             124  // out message: Settings needed for reversible ESCs
#define MSP_RC_DEADBAND                 125  // out message: Deadbands for yaw alt pitch roll
#define MSP_SENSOR_ALIGNMENT            126  // out message: Orientation of acc,gyro,mag
#define MSP_LED_STRIP_MODECOLOR         127  // out message: Get LED strip mode_color settings
#define MSP_VOLTAGE_METERS              128  // out message: Voltage (per meter)
#define MSP_CURRENT_METERS              129  // out message: Amperage (per meter)
#define MSP_BATTERY_STATE               130  // out message: Connected/Disconnected, Voltage, Current Used
#define MSP_MOTOR_CONFIG                131  // out message: Motor configuration (min/max throttle, etc)
#define MSP_GPS_CONFIG                  132  // out message: GPS configuration
#define MSP_COMPASS_CONFIG              133  // out message: Compass configuration
#define MSP_ESC_SENSOR_DATA             134  // out message: Extra ESC data from 32-Bit ESCs (Temperature, RPM)
#define MSP_GPS_RESCUE                  135  // out message: GPS Rescue angle, returnAltitude, descentDistance, groundSpeed, sanityChecks and minSats
#define MSP_GPS_RESCUE_PIDS             136  // out message: GPS Rescue throttleP and velocity PIDS + yaw P
#define MSP_VTXTABLE_BAND               137  // out message: VTX table band/channel data
#define MSP_VTXTABLE_POWERLEVEL         138  // out message: VTX table powerLevel data
#define MSP_MOTOR_TELEMETRY             139  // out message: Per-motor telemetry data (RPM, packet stats, ESC temp, etc.)

// Simplified tuning commands (140-145)
#define MSP_SIMPLIFIED_TUNING           140  // out message: Get simplified tuning values and enabled state
#define MSP_SET_SIMPLIFIED_TUNING       141  // in message:  Set simplified tuning positions and apply calculated tuning
#define MSP_CALCULATE_SIMPLIFIED_PID    142  // out message: Calculate PID values based on sliders without saving
#define MSP_CALCULATE_SIMPLIFIED_GYRO   143  // out message: Calculate gyro filter values based on sliders without saving
#define MSP_CALCULATE_SIMPLIFIED_DTERM  144  // out message: Calculate D term filter values based on sliders without saving
#define MSP_VALIDATE_SIMPLIFIED_TUNING  145  // out message: Returns array of true/false showing which simplified tuning groups match values

// Additional non-MultiWii commands (150-166)
#define MSP_STATUS_EX                   150  // out message: Cycletime, errors_count, CPU load, sensor present etc
#define MSP_UID                         160  // out message: Unique device ID
#define MSP_GPSSVINFO                   164  // out message: Get Signal Strength (only U-Blox)
#define MSP_GPSSTATISTICS               166  // out message: Get GPS debugging data

// OSD specific commands (180-189)
#define MSP_OSD_VIDEO_CONFIG            180  // out message: Get OSD video settings
#define MSP_SET_OSD_VIDEO_CONFIG        181  // in message:  Set OSD video settings
#define MSP_DISPLAYPORT                 182  // out message: External OSD displayport mode
#define MSP_COPY_PROFILE                183  // in message:  Copy settings between profiles
#define MSP_BEEPER_CONFIG               184  // out message: Get beeper configuration
#define MSP_SET_BEEPER_CONFIG           185  // in message:  Set beeper configuration
#define MSP_SET_TX_INFO                 186  // in message:  Set runtime information from TX lua scripts
#define MSP_TX_INFO                     187  // out message: Get runtime information for TX lua scripts
#define MSP_SET_OSD_CANVAS              188  // in message:  Set OSD canvas size COLSxROWS
#define MSP_OSD_CANVAS                  189  // out message: Get OSD canvas size COLSxROWS

// Set commands (200-229)
#define MSP_SET_RAW_RC                  200  // in message:  8 rc chan
#define MSP_SET_RAW_GPS                 201  // in message:  Fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID                     202  // in message:  P I D coeff (9 are used currently)
#define MSP_SET_RC_TUNING               204  // in message:  RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
#define MSP_ACC_CALIBRATION             205  // in message:  No param - calibrate accelerometer
#define MSP_MAG_CALIBRATION             206  // in message:  No param - calibrate magnetometer
#define MSP_RESET_CONF                  208  // in message:  No param - reset settings
#define MSP_SET_WP                      209  // in message:  Sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING              210  // in message:  Select setting number (0-2)
#define MSP_SET_HEADING                 211  // in message:  Define a new heading hold direction
#define MSP_SET_SERVO_CONFIGURATION     212  // in message:  Servo settings
#define MSP_SET_MOTOR                   214  // in message:  PropBalance function
#define MSP_SET_NAV_CONFIG              215  // in message:  Sets nav config parameters
#define MSP_SET_MOTOR_3D_CONFIG         217  // in message:  Settings needed for reversible ESCs
#define MSP_SET_RC_DEADBAND             218  // in message:  Deadbands for yaw alt pitch roll
#define MSP_SET_RESET_CURR_PID          219  // in message:  Reset current PID profile to defaults
#define MSP_SET_SENSOR_ALIGNMENT        220  // in message:  Set the orientation of acc,gyro,mag
#define MSP_SET_LED_STRIP_MODECOLOR     221  // in message:  Set LED strip mode_color settings
#define MSP_SET_MOTOR_CONFIG            222  // in message:  Motor configuration (min/max throttle, etc)
#define MSP_SET_GPS_CONFIG              223  // in message:  GPS configuration
#define MSP_SET_COMPASS_CONFIG          224  // in message:  Compass configuration
#define MSP_SET_GPS_RESCUE              225  // in message:  Set GPS Rescue parameters
#define MSP_SET_GPS_RESCUE_PIDS         226  // in message:  Set GPS Rescue PID values
#define MSP_SET_VTXTABLE_BAND           227  // in message:  Set vtxTable band/channel data
#define MSP_SET_VTXTABLE_POWERLEVEL     228  // in message:  Set vtxTable powerLevel data

// Multiple MSP and special commands (230-255)
#define MSP_MULTIPLE_MSP                230  // out message: Request multiple MSPs in one request
#define MSP_MODE_RANGES_EXTRA           238  // out message: Extra mode range data
#define MSP_SET_ACC_TRIM                239  // in message:  Set acc angle trim values
#define MSP_ACC_TRIM                    240  // out message: Get acc angle trim values
#define MSP_SERVO_MIX_RULES             241  // out message: Get servo mixer configuration
#define MSP_SET_SERVO_MIX_RULE          242  // in message:  Set servo mixer configuration
#define MSP_SET_PASSTHROUGH             245  // in message:  Set passthrough to peripherals
#define MSP_SET_RTC                     246  // in message:  Set the RTC clock
#define MSP_RTC                         247  // out message: Get the RTC clock
#define MSP_SET_BOARD_INFO              248  // in message:  Set the board information
#define MSP_SET_SIGNATURE               249  // in message:  Set the signature of the board and serial number
#define MSP_EEPROM_WRITE                250  // in message:  Write settings to EEPROM
#define MSP_RESERVE_1                   251  // reserved for system usage
#define MSP_RESERVE_2                   252  // reserved for system usage
#define MSP_DEBUGMSG                    253  // out message: debug string buffer
#define MSP_DEBUG                       254  // out message: debug1,debug2,debug3,debug4
#define MSP_V2_FRAME                    255  // MSPv2 payload indicator
