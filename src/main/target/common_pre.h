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

#ifdef STM32F4
#if defined(STM32F40_41xxx)
#define USE_FAST_DATA
#endif

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#define USE_PERSISTENT_OBJECTS
#define USE_CUSTOM_DEFAULTS_ADDRESS
#define USE_LATE_TASK_STATISTICS

#if defined(STM32F40_41xxx) || defined(STM32F411xE)
#define USE_OVERCLOCK
#endif
#endif // STM32F4

#ifdef STM32F7
#define USE_ITCM_RAM
#define ITCM_RAM_OPTIMISATION "-O2", "-freorder-blocks-algorithm=simple"
#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_OVERCLOCK
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#define USE_PERSISTENT_OBJECTS
#define USE_CUSTOM_DEFAULTS_ADDRESS
#define USE_LATE_TASK_STATISTICS
#endif // STM32F7

#ifdef STM32H7

#ifdef USE_DSHOT
#define USE_DSHOT_CACHE_MGMT
#endif

#define USE_ITCM_RAM
#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#define USE_PERSISTENT_OBJECTS
#define USE_DMA_RAM
#define USE_USB_MSC
#define USE_RTC_TIME
#define USE_PERSISTENT_MSC_RTC
#define USE_LATE_TASK_STATISTICS
#endif

#ifdef STM32G4
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_OVERCLOCK
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_MSC
#define USE_USB_CDC_HID
#define USE_MCO
#define USE_DMA_SPEC
#define USE_TIMER_MGMT
#define USE_LATE_TASK_STATISTICS
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT       MAX_AUX_CHANNEL_COUNT
#else
#define DEFAULT_AUX_CHANNEL_COUNT       6
#endif

// Set the default cpu_overclock to the first level (108MHz) for F411
// Helps with looptime stability as the CPU is borderline when running native gyro sampling
#if defined(USE_OVERCLOCK) && defined(STM32F411xE)
#define DEFAULT_CPU_OVERCLOCK 1
#else
#define DEFAULT_CPU_OVERCLOCK 0
#endif

#if defined(STM32H7)
// Move ISRs to fast ram to avoid flash latency.
#define FAST_IRQ_HANDLER FAST_CODE
#else
#define FAST_IRQ_HANDLER
#endif


#ifdef USE_ITCM_RAM
#if defined(ITCM_RAM_OPTIMISATION) && !defined(DEBUG)
#define FAST_CODE                   __attribute__((section(".tcm_code"))) __attribute__((optimize(ITCM_RAM_OPTIMISATION)))
#else
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#endif
// Handle case where we'd prefer code to be in ITCM, but it won't fit on the F745
#ifdef STM32F745xx
#define FAST_CODE_PREF
#else
#define FAST_CODE_PREF                  __attribute__((section(".tcm_code")))
#endif
#define FAST_CODE_NOINLINE          NOINLINE
#else
#define FAST_CODE
#define FAST_CODE_PREF
#define FAST_CODE_NOINLINE
#endif // USE_ITCM_RAM

#ifdef USE_CCM_CODE
#define CCM_CODE                    __attribute__((section(".ccm_code")))
#else
#define CCM_CODE
#endif

#ifdef USE_FAST_DATA
#define FAST_DATA_ZERO_INIT         __attribute__ ((section(".fastram_bss"), aligned(4)))
#define FAST_DATA                   __attribute__ ((section(".fastram_data"), aligned(4)))
#else
#define FAST_DATA_ZERO_INIT
#define FAST_DATA
#endif // USE_FAST_DATA

#if defined(STM32F4) || defined(STM32G4)
// F4 can't DMA to/from CCM (core coupled memory) SRAM (where the stack lives)
// On G4 there is no specific DMA target memory
#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO        static
#elif defined (STM32F7)
// F7 has no cache coherency issues DMAing to/from DTCM, otherwise buffers must be cache aligned
#define DMA_DATA_ZERO_INIT          FAST_DATA_ZERO_INIT
#define DMA_DATA                    FAST_DATA
#define STATIC_DMA_DATA_AUTO        static DMA_DATA
#else
// DMA to/from any memory
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA
#endif

#if defined(STM32F4) || defined (STM32H7)
// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
#endif

#ifdef USE_DMA_RAM
#if defined(STM32H7)
#define DMA_RAM __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI __attribute__((section(".DMA_RW_AXI"), aligned(32)))
extern uint8_t _dmaram_start__;
extern uint8_t _dmaram_end__;
#elif defined(STM32G4)
#define DMA_RAM_R __attribute__((section(".DMA_RAM_R")))
#define DMA_RAM_W __attribute__((section(".DMA_RAM_W")))
#define DMA_RAM_RW __attribute__((section(".DMA_RAM_RW")))
#endif
#else
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW
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

#define USE_MAG

#if !defined(USE_BARO) && !defined(USE_FAKE_BARO)
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
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P

#if defined(STM32F405) || defined(STM32F745) || defined(STM32G4) || defined(STM32H7)
#define USE_ACC_MPU6050
#define USE_GYRO_MPU6050
#define USE_ACCGYRO_BMI160
#endif
#endif

#if !defined(USE_EXST) && !defined(USE_FLASH)
#define USE_FLASHFS

#define USE_FLASH_TOOLS
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G    // 1Gb NAND flash support
#define USE_FLASH_W25M       // Stacked die support
#define USE_FLASH_W25M512    // 512Kb (256Kb x 2 stacked) NOR flash support
#define USE_FLASH_W25M02G    // 2Gb (1Gb x 2 stacked) NAND flash support
#define USE_FLASH_W25Q128FV  // 16MB Winbond 25Q128

#endif

#ifndef USE_MAX7456
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

#if defined(STM32F405) || defined(STM32F745) || defined(STM32H7)
#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI

#define USE_TRANSPONDER

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define USE_RANGEFINDER_TF
#endif

#define USE_RX_PPM
#define USE_RX_PWM

#define USE_BRUSHED_ESC_AUTODETECT  // Detect if brushed motors are connected and set defaults appropriately to avoid motors spinning on boot

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
#define USE_OSD_SD
#define USE_OSD_HD
#define USE_BLACKBOX

#if TARGET_FLASH_SIZE > 512

#if defined(USE_SERIALRX)

#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol

#endif // USE_SERIALRX

#if defined(USE_TELEMETRY)

#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_IBUS_EXTENDED
#define USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_MAVLINK
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_LTM

#endif // USE_TELEMETRY

#define USE_BATTERY_CONTINUE
#define USE_DASHBOARD
#define USE_EMFAT_AUTORUN
#define USE_EMFAT_ICON
#define USE_ESCSERIAL_SIMONK
#define USE_GPS
#define USE_GPS_PLUS_CODES
#define USE_LED_STRIP
#define USE_SERIAL_4WAY_SK_BOOTLOADER
#endif

#endif // !defined(CLOUD_BUILD)

#if !defined(LED_MAX_STRIP_LENGTH)
#ifdef USE_LED_STRIP_64
#define LED_MAX_STRIP_LENGTH           64
#else
#define LED_MAX_STRIP_LENGTH           32
#endif
#endif // # !defined(LED_MAX_STRIP_LENGTH)

#if defined(USE_LED_STRIP)
#define USE_LED_STRIP_STATUS_MODE
#endif

#if defined(USE_SDCARD)
#define USE_SDCARD_SPI
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
#define USE_SDCARD_SDIO
#endif
#endif

#if defined(USE_PINIO)
#define USE_PINIOBOX
#define USE_PIN_PULL_UP_DOWN
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
#define CONTROL_RATE_PROFILE_COUNT  4

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
#define USE_D_MIN

#define USE_THROTTLE_BOOST
#define USE_INTEGRATED_YAW_CONTROL

#define USE_ITERM_RELAX
#define USE_RC_SMOOTHING_FILTER
#define USE_THRUST_LINEARIZATION
#define USE_TPA_MODE

#ifdef USE_SERIALRX_SPEKTRUM
#define USE_SPEKTRUM_BIND
#define USE_SPEKTRUM_BIND_PLUG
#define USE_SPEKTRUM_REAL_RSSI
#define USE_SPEKTRUM_FAKE_RSSI
#define USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#define USE_SPEKTRUM_VTX_CONTROL
#define USE_SPEKTRUM_VTX_TELEMETRY
#define USE_SPEKTRUM_CMS_TELEMETRY
#endif // USE_SERIALRX_SPEKTRUM

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
#define USE_MULTI_GYRO
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


#if (defined(USE_OSD_HD) || defined(USE_OSD_SD)) && !defined(USE_OSD)
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
