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

#include "build/version.h"

/*

    The purpose of this file is to enable / disable any firmware "gates" for features and drivers
    that require hardware resources that are either available or not available after the target.h
    has been processed.

    It should also be used to define anything that should be defined (and is required), but is not
    already, to some sort of defaults.

    CLOUD_BUILD and CORE_BUILD should not be referenced here.

    NOTE: for 4.5 we will be removing any conditions related to specific MCU types, instead
    these should be defined in the target.h or in a file that is imported by target.h (in the
    case of common settings for a given MCU group)

*/

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT       MAX_AUX_CHANNEL_COUNT
#else
#define DEFAULT_AUX_CHANNEL_COUNT       6
#endif

#ifdef USE_ITCM_RAM
#if defined(ITCM_RAM_OPTIMISATION) && !defined(DEBUG)
#define FAST_CODE                   __attribute__((section(".tcm_code"))) __attribute__((optimize(ITCM_RAM_OPTIMISATION)))
#else
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#endif
#ifndef FAST_CODE_PREF
#define FAST_CODE_PREF              FAST_CODE
// If a particular target is short of ITCM RAM, defining FAST_CODE_PREF in the target.h file will
// cause functions decorated FAST_CODE_PREF to *not* go into ITCM RAM
// but if FAST_CODE_PREF is not defined for the target, FAST_CODE_PREF is an alias to FAST_CODE, and
// functions decorated with FAST_CODE_PREF *will* go into ITCM RAM.
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

/*
    BEGIN HARDWARE INCLUSIONS

    Simplified options for the moment, i.e. adding USE_MAG or USE_BARO and the entire driver suite is added.
    In the future we can move to specific drivers being added only - to save flash space.
*/

// normalize serial ports definitions
#include "serial_post.h"

#if defined(USE_ACC) \
    && !defined(USE_ACC_MPU6000) \
    && !defined(USE_ACC_MPU6050) \
    && !defined(USE_ACC_MPU6500) \
    && !defined(USE_ACCGYRO_BMI160) \
    && !defined(USE_ACCGYRO_BMI270) \
    && !defined(USE_ACC_SPI_ICM20602) \
    && !defined(USE_ACC_SPI_ICM20649) \
    && !defined(USE_ACC_SPI_ICM20689) \
    && !defined(USE_ACC_SPI_ICM42605) \
    && !defined(USE_ACCGYRO_ICM40609D) \
    && !defined(USE_ACC_SPI_ICM42688P) \
    && !defined(USE_ACCGYRO_ICM45686) \
    && !defined(USE_ACCGYRO_ICM45605) \
    && !defined(USE_ACCGYRO_LSM6DSO) \
    && !defined(USE_ACCGYRO_LSM6DSV16X) \
    && !defined(USE_ACC_SPI_MPU6000) \
    && !defined(USE_ACC_SPI_MPU6500) \
    && !defined(USE_ACC_SPI_MPU9250) \
    && !defined(USE_VIRTUAL_ACC) \
    && !defined(USE_ACCGYRO_IIM42653)
#error At least one USE_ACC device definition required
#endif

#if defined(USE_GYRO) \
    && !defined(USE_GYRO_MPU6050) \
    && !defined(USE_GYRO_MPU6500) \
    && !defined(USE_ACCGYRO_BMI160) \
    && !defined(USE_ACCGYRO_BMI270) \
    && !defined(USE_GYRO_SPI_ICM20602) \
    && !defined(USE_GYRO_SPI_ICM20649) \
    && !defined(USE_GYRO_SPI_ICM20689) \
    && !defined(USE_GYRO_SPI_ICM42605) \
    && !defined(USE_GYRO_SPI_ICM42688P) \
    && !defined(USE_ACCGYRO_ICM45686) \
    && !defined(USE_ACCGYRO_ICM45605) \
    && !defined(USE_ACCGYRO_ICM40609D) \
    && !defined(USE_ACCGYRO_LSM6DSO) \
    && !defined(USE_ACCGYRO_LSM6DSV16X) \
    && !defined(USE_GYRO_SPI_MPU6000) \
    && !defined(USE_GYRO_SPI_MPU6500) \
    && !defined(USE_GYRO_SPI_MPU9250) \
    && !defined(USE_VIRTUAL_GYRO) \
    && !defined(USE_ACCGYRO_IIM42653)
#error At least one USE_GYRO device definition required
#endif

#if defined(USE_MAG) && !defined(USE_VIRTUAL_MAG)

#ifndef USE_MAG_DATA_READY_SIGNAL
#define USE_MAG_DATA_READY_SIGNAL
#endif
#ifndef USE_MAG_HMC5883
#define USE_MAG_HMC5883
#endif
#ifndef USE_MAG_SPI_HMC5883
#define USE_MAG_SPI_HMC5883
#endif
#ifndef USE_MAG_QMC5883
#define USE_MAG_QMC5883
#endif
#ifndef USE_MAG_LIS2MDL
#define USE_MAG_LIS2MDL
#endif
#ifndef USE_MAG_LIS3MDL
#define USE_MAG_LIS3MDL
#endif
#ifndef USE_MAG_AK8963
#define USE_MAG_AK8963
#endif
#ifndef USE_MAG_MPU925X_AK8963
#define USE_MAG_MPU925X_AK8963
#endif
#ifndef USE_MAG_SPI_AK8963
#define USE_MAG_SPI_AK8963
#endif
#ifndef USE_MAG_AK8975
#define USE_MAG_AK8975
#endif
#ifndef USE_MAG_IST8310
#define USE_MAG_IST8310
#endif

#endif // END MAG HW defines

#if defined(USE_RX_CC2500)

#if !defined(USE_RX_SPI)
#define USE_RX_SPI
#endif

#define USE_RX_CC2500_SPI_PA_LNA
#define USE_RX_CC2500_SPI_DIVERSITY

#define USE_RX_SFHSS_SPI
#define USE_RX_REDPINE_SPI

#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_FRSKY_SPI
#define USE_RX_FRSKY_SPI_TELEMETRY

#define USE_RX_FLYSKY
#define USE_RX_FLYSKY_SPI_LED
#define USE_RX_SPEKTRUM
#define USE_RX_SPEKTRUM_TELEMETRY

#endif // defined(USE_RX_CC2500)

#if defined(CAMERA_CONTROL_PIN) && defined(USE_VTX) && !defined(USE_CAMERA_CONTROL)
#define USE_CAMERA_CONTROL
#endif

/* END HARDWARE INCLUSIONS */

#if defined(USE_VTX_RTC6705_SOFTSPI)
#define USE_VTX_RTC6705
#endif

#ifndef USE_DSHOT
#undef USE_ESC_SENSOR
#endif

#ifndef USE_ESC_SENSOR
#undef USE_ESC_SENSOR_TELEMETRY
#endif

// XXX Followup implicit dependencies among DASHBOARD, display_xxx and USE_I2C.
// XXX This should eventually be cleaned up.
#ifndef USE_I2C
#undef USE_I2C_OLED_DISPLAY
#undef USE_DASHBOARD
#else
#ifdef USE_DASHBOARD
#define USE_I2C_OLED_DISPLAY
#endif
#endif

// Remove USE_BARO_BMP280 and USE_BARO_MS5611 if USE_I2C is not defined.
#if !defined(USE_I2C)
#if defined(USE_BARO_BMP280)
#undef USE_BARO_BMP280
#endif
#if defined(USE_BARO_MS5611)
#undef USE_BARO_MS5611
#endif
#endif

// Add VARIO if BARO or GPS is defined. Remove when none defined.
#if defined(USE_BARO) || defined(USE_GPS)
#ifndef USE_VARIO
#define USE_VARIO
#endif
#else
#undef USE_VARIO
#endif

#if !defined(USE_SERIALRX)
#undef USE_SERIALRX_CRSF
#undef USE_SERIALRX_GHST
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_JETIEXBUS
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef USE_SERIALRX_FPORT
#endif

#if !defined(USE_TELEMETRY)
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_IBUS_EXTENDED
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_SRXL
#endif

#ifdef USE_SERIALRX_FPORT
#ifndef USE_TELEMETRY
#define USE_TELEMETRY
#endif
#ifndef USE_TELEMETRY_SMARTPORT
#define USE_TELEMETRY_SMARTPORT
#endif
#endif

#if defined(USE_TELEMETRY_IBUS_EXTENDED) && !defined(USE_TELEMETRY_IBUS)
#ifndef USE_TELEMETRY
#define USE_TELEMETRY
#endif
#define USE_TELEMETRY_IBUS
#endif

#ifdef USE_SERIALRX_JETIEXBUS
#ifndef USE_TELEMETRY
#define USE_TELEMETRY
#endif
#ifndef USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_JETIEXBUS
#endif
#endif // USE_SERIALRX_JETIEXBUS

#if !defined(USE_SERIALRX_CRSF)
#undef USE_TELEMETRY_CRSF
#undef USE_CRSF_LINK_STATISTICS
#undef USE_CRSF_V3
#endif

#if !defined(USE_RX_EXPRESSLRS) && !defined(USE_SERIALRX_CRSF)
#undef USE_RX_RSSI_DBM
#endif

#if !defined(USE_SERIALRX_GHST)
#undef USE_TELEMETRY_GHST
#endif

#if !defined(USE_TELEMETRY_CRSF) || !defined(USE_CMS)
#undef USE_CRSF_CMS_TELEMETRY
#endif

#if !defined(USE_TELEMETRY_CRSF)
#undef USE_CRSF_V3
#endif

#if !defined(USE_SERIALRX_JETIEXBUS)
#undef USE_TELEMETRY_JETIEXBUS
#endif

#if !defined(USE_TELEMETRY_IBUS)
#undef USE_TELEMETRY_IBUS_EXTENDED
#endif

// If USE_SERIALRX_SPEKTRUM or SERIALRX_SRXL2 was dropped by a target, drop all related options
#if !defined(USE_SERIALRX_SPEKTRUM) && !defined(USE_SERIALRX_SRXL2)
#undef USE_SPEKTRUM_BIND
#undef USE_SPEKTRUM_BIND_PLUG
#undef USE_SPEKTRUM_REAL_RSSI
#undef USE_SPEKTRUM_VIRTUAL_RSSI
#undef USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#undef USE_SPEKTRUM_VTX_CONTROL
#undef USE_SPEKTRUM_VTX_TELEMETRY
#undef USE_TELEMETRY_SRXL
#endif // !defined(USE_SERIALRX_SPEKTRUM) && !defined(USE_SERIALRX_SRXL2)

#if !defined(USE_CMS) || !defined(USE_TELEMETRY_SRXL)
#undef USE_SPEKTRUM_CMS_TELEMETRY
#endif

#if defined(USE_SERIALRX_SBUS) || defined(USE_SERIALRX_FPORT)
#if !defined(USE_SBUS_CHANNELS)
#define USE_SBUS_CHANNELS
#endif
#endif

#if !defined(USE_TELEMETRY_SMARTPORT) && !defined(USE_TELEMETRY_CRSF) && !defined(USE_TELEMETRY_GHST)
#undef USE_MSP_OVER_TELEMETRY
#endif

#if !defined(USE_RX_MSP) && defined(USE_RX_MSP_OVERRIDE)
#undef USE_RX_MSP_OVERRIDE
#endif

/* If either VTX_CONTROL or VTX_COMMON is undefined then remove common code and device drivers */
#if !defined(USE_VTX_COMMON) || !defined(USE_VTX_CONTROL)
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#undef USE_VTX_TABLE
#undef USE_VTX_MSP
#endif

// Some target doesn't define USE_ADC which USE_ADC_INTERNAL depends on
#ifndef USE_ADC
#undef USE_ADC_INTERNAL
#endif

#if (defined(USE_SDCARD) || defined(USE_FLASH)) && !defined(USE_BLACKBOX)
#define USE_BLACKBOX
#endif

#ifdef USE_FLASH
#if !defined(USE_FLASH_TOOLS)
#define USE_FLASH_TOOLS
#endif
#if !defined(USE_FLASHFS)
#define USE_FLASHFS
#endif
#endif

#if (defined(USE_FLASH_W25M512) || defined(USE_FLASH_W25Q128FV) || defined(USE_FLASH_PY25Q128HA)) && !defined(USE_FLASH_M25P16)
#define USE_FLASH_M25P16
#endif

#if defined(USE_FLASH_W25M02G) && !defined(USE_FLASH_W25N01G)
#define USE_FLASH_W25N01G
#endif

#if defined(USE_FLASH_W25N02K) || defined(USE_FLASH_W25N01G)
#define USE_FLASH_W25N
#endif

#if (defined(USE_FLASH_M25P16) || defined(USE_FLASH_W25N)) && !defined(USE_FLASH_W25M)
#define USE_FLASH_W25M
#endif

#if defined(USE_FLASH_M25P16) || defined(USE_FLASH_W25M) || defined(USE_FLASH_W25N) || defined(USE_FLASH_W25Q128FV)
#if !defined(USE_FLASH_CHIP)
#define USE_FLASH_CHIP
#endif
#endif

#if defined(USE_SPI) && (defined(USE_FLASH_M25P16) || defined(USE_FLASH_W25M512) || defined(USE_FLASH_W25N) || defined(USE_FLASH_W25M02G))
#if !defined(USE_FLASH_SPI)
#define USE_FLASH_SPI
#endif
#endif

#if defined(USE_QUADSPI) && (defined(USE_FLASH_W25Q128FV) || defined(USE_FLASH_W25N))
#if !defined(USE_FLASH_QUADSPI)
#define USE_FLASH_QUADSPI
#endif
#endif

#if defined(USE_OCTOSPI) && defined(USE_FLASH_W25Q128FV)
#if !defined(USE_FLASH_OCTOSPI)
#define USE_FLASH_OCTOSPI
#endif
#endif

#ifndef USE_FLASH_CHIP
#undef USE_FLASH_TOOLS
#undef USE_FLASHFS
#endif

#if (!defined(USE_SDCARD) && !defined(USE_FLASHFS)) || !defined(USE_BLACKBOX)
#undef USE_USB_MSC
#endif

#if !defined(USE_SDCARD)
#undef USE_SDCARD_SDIO
#undef USE_SDCARD_SPI
#endif

#if !defined(USE_VCP)
#undef USE_USB_CDC_HID
#undef USE_USB_MSC
#endif

#if defined(USE_USB_CDC_HID) || defined(USE_USB_MSC)
#define USE_USB_ADVANCED_PROFILES
#endif

#if !defined(USE_OSD)
#undef USE_RX_LINK_QUALITY_INFO
#undef USE_OSD_PROFILES
#undef USE_OSD_STICK_OVERLAY
#undef USE_RX_LINK_UPLINK_POWER
#endif

// Older ACC/GYRO sensors use MPU6500 driver
#if !defined(USE_ACC_MPU6500) && (defined(USE_ACC_ICM20601) || defined(USE_ACC_ICM20602) || defined(USE_ACC_ICM20608G))
#define USE_ACC_MPU6500
#endif
#if !defined(USE_ACC_SPI_MPU6500) && (defined(USE_ACC_SPI_MPU9250) || defined(USE_ACC_SPI_ICM20601) || defined(USE_ACC_SPI_ICM20602) || defined(USE_ACC_SPI_ICM20608G))
#define USE_ACC_SPI_MPU6500
#endif
#if !defined(USE_GYRO_MPU6500) && (defined(USE_GYRO_ICM20601) || defined(USE_GYRO_ICM20602) || defined(USE_GYRO_ICM20608G))
#define USE_GYRO_MPU6500
#endif
#if !defined(USE_GYRO_SPI_MPU6500) && (defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20602) || defined(USE_GYRO_SPI_ICM20608G))
#define USE_GYRO_SPI_MPU6500
#endif

// Generate USE_SPI_GYRO or USE_I2C_GYRO
#if defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250) \
    || defined(USE_GYRO_L3GD20) || defined(USE_GYRO_SPI_ICM42605) || defined(USE_GYRO_SPI_ICM42688P) || defined(USE_ACCGYRO_ICM45686) \
    || defined(USE_ACCGYRO_ICM45605) || defined(USE_ACCGYRO_IIM42653) || defined(USE_ACCGYRO_BMI160) || defined(USE_ACCGYRO_BMI270) \
    || defined(USE_ACCGYRO_LSM6DSV16X) || defined(USE_ACCGYRO_LSM6DSO) || defined(USE_ACCGYRO_ICM40609D)
#ifndef USE_SPI_GYRO
#define USE_SPI_GYRO
#endif
#endif

#ifndef SIMULATOR_BUILD
#ifndef USE_ACC
#define USE_ACC
#endif

#ifndef USE_GYRO
#define USE_GYRO
#endif
#endif

// CX10 is a special case of SPI RX which requires XN297
#if defined(USE_RX_CX10)
#define USE_RX_XN297
#endif

// Setup crystal frequency on F4 for backward compatibility
// Should be set to zero for generic targets to ensure USB is working
// when unconfigured for targets with non-standard crystal.
// Can be set at runtime with with CLI parameter 'system_hse_mhz'.
#ifndef SYSTEM_HSE_MHZ
#define SYSTEM_HSE_MHZ 0
#endif

#ifndef USE_BLACKBOX
#undef USE_USB_MSC
#endif

#if (!defined(USE_FLASHFS) || !defined(USE_RTC_TIME) || !defined(USE_USB_MSC) || !defined(USE_PERSISTENT_OBJECTS))
#undef USE_PERSISTENT_MSC_RTC
#endif

#if !defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && !defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
#undef  USE_SERIAL_4WAY_BLHELI_INTERFACE
#elif !defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && (defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) || defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
#ifndef USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif
#endif

#if defined(USE_RX_PWM) || defined(USE_DSHOT) || defined(USE_LED_STRIP) || defined(USE_TRANSPONDER) || defined(USE_BEEPER) || defined(USE_SERIAL_4WAY_BLHELI_INTERFACE)
#ifndef USE_PWM_OUTPUT
#define USE_PWM_OUTPUT
#endif
#endif

#if !defined(USE_LED_STRIP)
#undef USE_LED_STRIP_STATUS_MODE
#endif

#if defined(USE_MAX7456) || defined(USE_FRSKYOSD) || defined(USE_MSP_DISPLAYPORT)
#ifndef USE_VIDEO_SYSTEM
#define USE_VIDEO_SYSTEM
#endif
#endif

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_STATUS_MODE)
#define USE_WS2811_SINGLE_COLOUR
#endif

#if defined(SIMULATOR_BUILD) || defined(UNIT_TEST)
// This feature uses 'arm_math.h', which does not exist for x86.
#undef USE_DYN_NOTCH_FILTER
#endif

#ifndef USE_CMS
#undef USE_CMS_FAILSAFE_MENU
#endif

#ifndef USE_DSHOT
#undef USE_DSHOT_TELEMETRY
#undef USE_DSHOT_BITBANG
#endif

#ifndef USE_DSHOT_TELEMETRY
#undef USE_RPM_FILTER
#undef USE_DSHOT_TELEMETRY_STATS
#undef USE_DYN_IDLE
#endif

#if !defined(USE_BOARD_INFO)
#undef USE_SIGNATURE
#endif

#if !defined(USE_ACC)
#undef USE_GPS_RESCUE
#undef USE_ACRO_TRAINER
#endif

#if (!defined(USE_GPS_RESCUE) || !defined(USE_CMS_FAILSAFE_MENU))
#undef USE_CMS_GPS_RESCUE_MENU
#endif

#ifndef USE_BEEPER
#undef BEEPER_PIN
#undef BEEPER_PWM_HZ
#endif

#if defined(USE_DMA_SPEC)
#define USE_TIMER_DMA
#else
#undef USE_TIMER_MGMT
#endif

#if defined(USE_TIMER_MGMT)
#undef USED_TIMERS
#endif

#if defined(USE_OPTICALFLOW_MT)
#ifndef USE_RANGEFINDER_MT
#define USE_RANGEFINDER_MT
#endif
#ifndef USE_OPTICALFLOW
#define USE_OPTICALFLOW
#endif
#endif // USE_OPTICALFLOW_MT

#if defined(USE_RANGEFINDER_HCSR04) || defined(USE_RANGEFINDER_SRF10) || defined(USE_RANGEFINDER_HCSR04_I2C) || defined(USE_RANGEFINDER_VL53L0X) || defined(USE_RANGEFINDER_UIB) || defined(USE_RANGEFINDER_TF) || defined(USE_RANGEFINDER_MT)
#ifndef USE_RANGEFINDER
#define USE_RANGEFINDER
#endif
#endif // USE_RANGEFINDER_XXX

#ifndef USE_GPS_RESCUE
#undef USE_CMS_GPS_RESCUE_MENU
#endif

#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_FILE) || defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_SDCARD) || defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
#ifndef EEPROM_SIZE
#define EEPROM_SIZE     4096
#endif
extern uint8_t eepromData[EEPROM_SIZE];
#define __config_start (*eepromData)
#define __config_end (*ARRAYEND(eepromData))
#else
#ifndef CONFIG_IN_FLASH
#define CONFIG_IN_FLASH
#endif
struct linker_symbol;
extern struct linker_symbol __config_start;   // configured via linker script when building binaries.
extern struct linker_symbol __config_end;
#endif

#if defined(USE_EXST) && !defined(RAMBASED)
#define USE_FLASH_BOOT_LOADER
#endif

#if defined(USE_FLASH_MEMORY_MAPPED)
#if !defined(USE_RAM_CODE)
#define USE_RAM_CODE
#endif

#define MMFLASH_CODE RAM_CODE
#define MMFLASH_CODE_NOINLINE RAM_CODE NOINLINE

#define MMFLASH_DATA FAST_DATA
#define MMFLASH_DATA_ZERO_INIT FAST_DATA_ZERO_INIT
#else
#define MMFLASH_CODE
#define MMFLASH_CODE_NOINLINE
#define MMFLASH_DATA
#define MMFLASH_DATA_ZERO_INIT
#endif

#ifdef USE_RAM_CODE
// RAM_CODE for methods that need to be in RAM, but don't need to be in the fastest type of memory.
// Note: if code is marked as RAM_CODE it *MUST* be in RAM, there is no alternative unlike functions marked with FAST_CODE/CCM_CODE
#define RAM_CODE                   __attribute__((section(".ram_code")))
#endif

#ifndef USE_ITERM_RELAX
#undef USE_ABSOLUTE_CONTROL
#endif

#if defined(USE_RX_EXPRESSLRS)
// ELRS depends on CRSF telemetry
#if !defined(USE_TELEMETRY)
#define USE_TELEMETRY
#endif
#if !defined(USE_TELEMETRY_CRSF)
#define USE_TELEMETRY_CRSF
#endif
#if !defined(USE_CRSF_LINK_STATISTICS)
#define USE_CRSF_LINK_STATISTICS
#endif
#if !defined(USE_SERIALRX_CRSF)
#define USE_SERIALRX_CRSF
#endif
#endif

#if defined(USE_RX_SPI) || defined(USE_SERIALRX_SRXL2) || defined(USE_SERIALRX_CRSF)
#define USE_RX_BIND
#endif

#ifndef USE_GPS
#undef USE_GPS_PLUS_CODES
#undef USE_GPS_LAP_TIMER
#endif

#ifdef USE_GPS_LAP_TIMER
#define USE_CMS_GPS_LAP_TIMER_MENU
#endif

// Enable PINIO by default if any PIN is defined
#if !defined(USE_PINIO) && (defined(PINIO1_BOX) || defined(PINIO2_BOX) || defined(PINIO3_BOX) || defined(PINIO4_BOX))
#define USE_PINIO
#endif

#ifdef USE_PINIO
#ifndef USE_PINIOBOX
#define USE_PINIOBOX
#endif
#ifndef USE_PIN_PULL_UP_DOWN
#define USE_PIN_PULL_UP_DOWN
#endif
#endif // USE_PINIO

