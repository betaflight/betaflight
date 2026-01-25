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

/*

    The purpose of this file is to enable the firmware "gates" for features and drivers
    prior to entering the target.h.

    CLOUD_BUILD is used to signify that the build is a user requested build and that the
    features to be enabled will be defined ALREADY.

    CORE_BUILD is used to signify that the build is a user requested build and that the
    features to be enabled will be the minimal set, and all the drivers should be present.

    If neither of the above are present then the build should simply be a baseline build
    for continuous integration, i.e. the compilation of the majority of features and drivers
    dependent on the size of the flash available.

    NOTE: for 4.5 we will be removing any conditions related to specific MCU types, instead
    these should be defined in the target.h or in a file that is imported by target.h (in the
    case of common settings for a given MCU group)

*/

#define USE_PARAMETER_GROUPS
// type conversion warnings.
// -Wconversion can be turned on to enable the process of eliminating these warnings
//#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
// -Wpadded can be turned on to check padding of structs
//#pragma GCC diagnostic warning "-Wpadded"

#if !defined(CLOUD_BUILD) && !defined(SITL)
#define USE_DSHOT
#endif

#ifdef USE_DSHOT
#define USE_DSHOT_BITBANG
#define USE_DSHOT_TELEMETRY
#define USE_DSHOT_TELEMETRY_STATS
#endif

#define USE_MOTOR
#define USE_DMA
#define USE_TIMER

#define USE_CLI
#define USE_SERIAL_PASSTHROUGH
#define USE_GYRO_REGISTER_DUMP  // Adds gyroregisters command to cli to dump configured register values
#define USE_IMU_CALC

// all the settings for classic build
#if !defined(CLOUD_BUILD) && !defined(SITL)

// if no board config is provided, include all drivers
#if !defined(USE_CONFIG)

#define USE_MAG

#if !defined(USE_BARO) && !defined(USE_VIRTUAL_BARO)
#define USE_BARO

#define USE_BARO_MS5611
#define USE_BARO_SPI_MS5611
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define USE_BARO_BMP388
#define USE_BARO_SPI_BMP388
#define USE_BARO_LPS
#define USE_BARO_SPI_LPS
#define USE_BARO_QMP6988
#define USE_BARO_SPI_QMP6988
#define USE_BARO_DPS310
#define USE_BARO_SPI_DPS310
#define USE_BARO_BMP085
#define USE_BARO_2SMBP_02B
#define USE_BARO_SPI_2SMBP_02B
#define USE_BARO_LPS22DF
#define USE_BARO_SPI_LPS22DF
#endif

#if !defined(USE_GYRO) && !defined(USE_ACC)
#define USE_ACC
#define USE_GYRO

#define USE_ACC_MPU6500
#define USE_GYRO_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_ICM20689
#define USE_GYRO_SPI_ICM20689
#define USE_ACCGYRO_LSM6DSO
#define USE_ACCGYRO_BMI270
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#define USE_ACCGYRO_ICM45686
#define USE_ACCGYRO_ICM45605
#define USE_ACCGYRO_IIM42652
#define USE_ACCGYRO_IIM42653
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P
#define USE_ACCGYRO_LSM6DSV16X
#define USE_ACCGYRO_ICM40609D

#if TARGET_FLASH_SIZE > 512
#define USE_ACC_MPU6050
#define USE_GYRO_MPU6050
#define USE_ACCGYRO_BMI160
#endif
#endif // ACC GYRO inclusion

#if !defined(USE_FLASH_CHIP)

#if !defined(USE_EXST) && !defined(USE_FLASH)
#define USE_FLASH
#endif

#if defined(USE_FLASH)

#if !defined(USE_EXST)
#define USE_FLASHFS
#define USE_FLASH_TOOLS
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G    // 1Gb NAND flash support
#define USE_FLASH_W25N02K    // 2Gb NAND flash support
#define USE_FLASH_W25M       // Stacked die support
#define USE_FLASH_W25M512    // 512Kb (256Kb x 2 stacked) NOR flash support
#define USE_FLASH_W25M02G    // 2Gb (1Gb x 2 stacked) NAND flash support
#define USE_FLASH_W25Q128FV  // 16MB Winbond 25Q128
#define USE_FLASH_PY25Q128HA // 16MB PUYA SEMI 25Q128
#endif // USE_EXST

#endif // USE_FLASH
#endif // USE_FLASH_CHIP

#if !defined(USE_MAX7456)
#define USE_MAX7456
#endif

#if !defined(USE_RX_SPI)
#define USE_RX_SPI

#define USE_RX_CC2500
#define USE_RX_EXPRESSLRS
#define USE_RX_SX1280
#define USE_RX_SX127X
#endif // !USE_RX_SPI

#if !defined(USE_EXST) && !defined(USE_SDCARD)
#define USE_SDCARD
#endif

#endif // !defined(USE_CONFIG)

#define USE_RX_PPM
#define USE_RX_PWM

#define USE_PINIO

#if !defined(USE_SERIAL_RX)

#define USE_SERIALRX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_GHST       // ImmersionRC Ghost Protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_FPORT      // FrSky FPort
#define USE_SERIALRX_XBUS       // JR
#define USE_SERIALRX_SRXL2      // Spektrum SRXL2 protocol

#endif // !defined(USE_SERIAL_RX)

#if !defined(USE_TELEMETRY)
#define USE_TELEMETRY

#define USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_SMARTPORT
#define USE_TELEMETRY_CRSF
#define USE_TELEMETRY_GHST
#define USE_TELEMETRY_SRXL

#endif // !defined(USE_TELEMETRY)

#define USE_SERVOS

#define USE_VTX
#define USE_OSD
#if !defined(USE_OSD_SD) && !defined(USE_OSD_HD)
#define USE_OSD_SD
#define USE_OSD_HD
#endif
#define USE_BLACKBOX

#if TARGET_FLASH_SIZE >= 1024

#if defined(USE_SERIALRX)

#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_MAVLINK    // MAVLink protocol for serial RX
#endif // USE_SERIALRX

#if defined(USE_TELEMETRY)

#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_IBUS_EXTENDED
#define USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_MAVLINK
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_LTM

#endif // USE_TELEMETRY

#ifdef USE_DSHOT_TELEMETRY
#define USE_RPM_LIMIT
#endif

#ifdef USE_OSD
// Dependency for CMS is defined outside this block.
#define USE_OSD_QUICK_MENU
#define USE_RC_STATS
#define USE_SPEC_PREARM_SCREEN
#endif

#define USE_BATTERY_CONTINUE
#define USE_DASHBOARD
#define USE_EMFAT_AUTORUN
#define USE_EMFAT_ICON
#define USE_ESCSERIAL_SIMONK
#define USE_ALTITUDE_HOLD
#define USE_POSITION_HOLD

#if !defined(USE_GPS)
#define USE_GPS
#endif

#if !defined(USE_GPS_PLUS_CODES)
#define USE_GPS_PLUS_CODES
#endif

#if !defined(USE_LED_STRIP)
#define USE_LED_STRIP
#endif

#define USE_SERIAL_4WAY_SK_BOOTLOADER

#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI

#define USE_TRANSPONDER

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define USE_RANGEFINDER_TF
#define USE_OPTICALFLOW_MT

#endif // TARGET_FLASH_SIZE >= 1024

#endif // !defined(CLOUD_BUILD)

#if !defined(LED_STRIP_MAX_LENGTH)
#ifdef USE_LED_STRIP_64
#define LED_STRIP_MAX_LENGTH           64
#else
#define LED_STRIP_MAX_LENGTH           32
#endif
#endif // # !defined(LED_STRIP_MAX_LENGTH)

#if defined(USE_LED_STRIP)
#define USE_LED_STRIP_STATUS_MODE
#endif

#if defined(USE_VTX)
#define USE_VTX_COMMON
#define USE_VTX_CONTROL
#define USE_VTX_SMARTAUDIO
#define USE_VTX_TRAMP
#define USE_VTX_MSP
#define USE_VTX_TABLE
#endif // USE_VTX

#define USE_HUFFMAN

#define PID_PROFILE_COUNT 4
#ifndef CONTROL_RATE_PROFILE_COUNT
#define CONTROL_RATE_PROFILE_COUNT 4 // or maybe 6
#endif

#define USE_CLI_BATCH
#define USE_RESOURCE_MGMT

#define USE_RUNAWAY_TAKEOFF     // Runaway Takeoff Prevention (anti-taz)

#define USE_GYRO_OVERFLOW_CHECK
#define USE_YAW_SPIN_RECOVERY

#ifdef USE_DSHOT
#define USE_DSHOT_DMAR
#endif

#define USE_MSP_OVER_TELEMETRY

#define USE_VIRTUAL_CURRENT_METER
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_RCDEVICE

#define USE_GYRO_LPF2
#define USE_DYN_LPF
#define USE_D_MAX

#define USE_THROTTLE_BOOST
#define USE_INTEGRATED_YAW_CONTROL

#define USE_ITERM_RELAX
#define USE_RC_SMOOTHING_FILTER
#define USE_THRUST_LINEARIZATION

#ifdef USE_SERIALRX_SPEKTRUM
#define USE_SPEKTRUM_BIND
#define USE_SPEKTRUM_BIND_PLUG
#define USE_SPEKTRUM_REAL_RSSI
#define USE_SPEKTRUM_VIRTUAL_RSSI
#define USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#define USE_SPEKTRUM_VTX_CONTROL
#define USE_SPEKTRUM_VTX_TELEMETRY
#define USE_SPEKTRUM_CMS_TELEMETRY
#endif // USE_SERIALRX_SPEKTRUM

#ifdef USE_TELEMETRY_SRXL
#ifndef USE_SERIALRX_SPEKTRUM
#define USE_SERIALRX_SPEKTRUM
#define USE_SPEKTRUM_VTX_CONTROL
#define USE_SPEKTRUM_VTX_TELEMETRY
#define USE_SPEKTRUM_CMS_TELEMETRY
#endif // USE_SERIALRX_SPEKTRUM
#endif // USE_TELEMETRY_SRXL

#define USE_BOARD_INFO
#define USE_RTC_TIME
#define USE_ESC_SENSOR_INFO

#define USE_RX_MSP
#define USE_RX_RSSI_DBM
#define USE_RX_RSNR
#define USE_RX_LINK_QUALITY_INFO
#define USE_RX_MSP_OVERRIDE
#define USE_RX_LINK_UPLINK_POWER

#define USE_AIRMODE_LPF
#define USE_GYRO_DLPF_EXPERIMENTAL
#define USE_SENSOR_NAMES
#define USE_UNCOMMON_MIXERS
#define USE_SIGNATURE
#define USE_ABSOLUTE_CONTROL
#define USE_HOTT_TEXTMODE
#define USE_ESC_SENSOR_TELEMETRY
#define USE_TELEMETRY_SENSORS_DISABLED_DETAILS
#define USE_PERSISTENT_STATS
#define USE_PROFILE_NAMES
#define USE_FEEDFORWARD
#define USE_CUSTOM_BOX_NAMES
#define USE_BATTERY_VOLTAGE_SAG_COMPENSATION
#define USE_SIMPLIFIED_TUNING
#define USE_CRAFTNAME_MSGS

#if !defined(CORE_BUILD)
// CORE_BUILD is only hardware drivers, and the bare minimum
// any thing defined here will be in the standard (git hub actions)
// builds or included in CLOUD_BUILD by default.

#if !defined(USE_LAUNCH_CONTROL)
#define USE_LAUNCH_CONTROL
#endif

#endif // !defined(CORE_BUILD)

#ifdef USE_GPS
#define USE_GPS_NMEA
#define USE_GPS_UBLOX
#define USE_GPS_RESCUE
#endif // USE_GPS

#if (defined(USE_OSD_HD) || defined(USE_OSD_SD) || defined(USE_FRSKYOSD)) && !defined(USE_OSD)
// If either USE_OSD_SD for USE_OSD_HD are defined, ensure that USE_OSD is also defined
#define USE_OSD
#endif

#if defined(USE_OSD)

#if !defined(USE_OSD_HD) && !defined(USE_OSD_SD)
// If USE_OSD is defined without specifying SD or HD, then support both
#define USE_OSD_SD
#define USE_OSD_HD
#endif

#if !defined(USE_OSD_SD) && defined(USE_MAX7456)
// If USE_OSD_SD isn't defined then explicitly exclude MAX7456 support
#undef USE_MAX7456
#endif

#define USE_CANVAS
#define USE_CMS
#define USE_CMS_FAILSAFE_MENU
#define USE_EXTENDED_CMS_MENUS
#define USE_MSP_DISPLAYPORT
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_OSD_ADJUSTMENTS
#define USE_OSD_PROFILES
#define USE_OSD_STICK_OVERLAY
#define USE_OSD_CUSTOM_TEXT

#if defined(USE_GPS)
#define USE_CMS_GPS_RESCUE_MENU
#endif

#endif // defined(USE_OSD)

#if defined(USE_SERIALRX_CRSF)

#define USE_CRSF_V3

#if defined(USE_TELEMETRY_CRSF) && defined(USE_CMS)
#define USE_CRSF_CMS_TELEMETRY
#define USE_CRSF_LINK_STATISTICS
#endif

#endif // defined(USE_SERIALRX_CRSF)

// The SERIALRX_MAVLINK provider can not work without TELEMETRY_MAVLINK
#if defined(USE_SERIALRX_MAVLINK) && !defined(USE_TELEMETRY_MAVLINK)
#define USE_TELEMETRY_MAVLINK
#endif

// USE_RACE_PRO feature pack
#ifdef USE_RACE_PRO

#ifdef USE_DSHOT_TELEMETRY
#ifndef USE_RPM_LIMIT
#define USE_RPM_LIMIT
#endif
#endif

#ifdef USE_OSD
#ifndef USE_OSD_QUICK_MENU
#define USE_OSD_QUICK_MENU
#endif
#ifndef USE_RC_STATS
#define USE_RC_STATS
#endif
#ifndef USE_SPEC_PREARM_SCREEN
#define USE_SPEC_PREARM_SCREEN
#endif
#endif

#endif // USE_RACE_PRO

#ifdef USE_WING

#ifndef USE_SERVOS
#define USE_SERVOS
#endif

#ifndef USE_ADVANCED_TPA
#define USE_ADVANCED_TPA
#endif

#undef USE_YAW_SPIN_RECOVERY
#undef USE_LAUNCH_CONTROL
#undef USE_ABSOLUTE_CONTROL
#undef USE_INTEGRATED_YAW_CONTROL
#undef USE_RUNAWAY_TAKEOFF

#endif // USE_WING

#if defined(USE_POSITION_HOLD) && !defined(USE_GPS)
#error "USE_POSITION_HOLD requires USE_GPS to be defined"
#endif

// backwards compatibility for older config.h targets
#ifndef GYRO_CONFIG_USE_GYRO_1
#define GYRO_CONFIG_USE_GYRO_1 0
#endif

#ifndef GYRO_CONFIG_USE_GYRO_2
#define GYRO_CONFIG_USE_GYRO_2 1
#endif

#ifndef GYRO_CONFIG_USE_GYRO_BOTH
#define GYRO_CONFIG_USE_GYRO_BOTH 2
#endif

#ifdef DEFAULT_GYRO_TO_USE
  #ifndef DEFAULT_GYRO_ENABLED
    #if DEFAULT_GYRO_TO_USE == GYRO_CONFIG_USE_GYRO_1
      #define DEFAULT_GYRO_ENABLED GYRO_MASK(0)
    #elif DEFAULT_GYRO_TO_USE == GYRO_CONFIG_USE_GYRO_2
      #define DEFAULT_GYRO_ENABLED GYRO_MASK(1)
    #elif DEFAULT_GYRO_TO_USE == GYRO_CONFIG_USE_GYRO_BOTH
      #define DEFAULT_GYRO_ENABLED (GYRO_MASK(0) | GYRO_MASK(1))
    #endif
  #endif
#elif !defined(DEFAULT_GYRO_ENABLED)
  // assume one gyro
  #define DEFAULT_GYRO_ENABLED GYRO_MASK(0)
#endif

#ifndef GYRO_COUNT
  #ifdef GYRO_1_CS_PIN
    #define GYRO_1_DEFINED 1
  #else
    #define GYRO_1_DEFINED 0
  #endif

  #ifdef GYRO_2_CS_PIN
    #define GYRO_2_DEFINED 1
  #else
    #define GYRO_2_DEFINED 0
  #endif

  #ifdef GYRO_3_CS_PIN
    #define GYRO_3_DEFINED 1
  #else
    #define GYRO_3_DEFINED 0
  #endif

  #ifdef GYRO_4_CS_PIN
    #define GYRO_4_DEFINED 1
  #else
    #define GYRO_4_DEFINED 0
  #endif

  #define GYRO_COUNT_RAW (GYRO_1_DEFINED + GYRO_2_DEFINED + GYRO_3_DEFINED + GYRO_4_DEFINED)

  // Ensure GYRO_COUNT is at least 1
  #if GYRO_COUNT_RAW > 0
    #define GYRO_COUNT GYRO_COUNT_RAW
  #else
    #define GYRO_COUNT 1
  #endif
#endif
