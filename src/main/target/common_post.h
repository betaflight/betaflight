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

// Touch up configuration

#pragma once

#include "build/version.h"

// Targets with built-in vtx do not need external vtx
#if defined(USE_VTX_RTC6705) && !defined(VTX_RTC6705_OPTIONAL)
#undef USE_VTX_SMARTAUDIO
#undef USE_VTX_TRAMP
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

// XXX Remove USE_BARO_BMP280 and USE_BARO_MS5611 if USE_I2C is not defined.
// XXX This should go away buy editing relevant target.h files
#if !defined(USE_I2C)
#if defined(USE_BARO_BMP280)
#undef USE_BARO_BMP280
#endif
#if defined(USE_BARO_MS5611)
#undef USE_BARO_MS5611
#endif
#endif

#if !defined(USE_BARO) && !defined(USE_GPS)
#undef USE_VARIO
#endif

#if !defined(USE_SERIAL_RX)
#undef USE_SERIALRX_CRSF
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
#undef USE_CRSF_CMS_TELEMETRY
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_IBUS_EXTENDED
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_SRXL
#undef USE_SERIALRX_FPORT
#endif

#if !defined(USE_SERIALRX_CRSF)
#undef USE_TELEMETRY_CRSF
#endif

#if !defined(USE_TELEMETRY_CRSF) || !defined(USE_CMS)
#undef USE_CRSF_CMS_TELEMETRY
#endif

#if !defined(USE_SERIALRX_JETIEXBUS)
#undef USE_TELEMETRY_JETIEXBUS
#endif

#if !defined(USE_TELEMETRY_IBUS)
#undef USE_TELEMETRY_IBUS_EXTENDED
#endif

// If USE_SERIALRX_SPEKTRUM was dropped by a target, drop all related options
#ifndef USE_SERIALRX_SPEKTRUM
#undef USE_SPEKTRUM_BIND
#undef USE_SPEKTRUM_BIND_PLUG
#undef USE_SPEKTRUM_REAL_RSSI
#undef USE_SPEKTRUM_FAKE_RSSI
#undef USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#undef USE_SPEKTRUM_VTX_CONTROL
#undef USE_SPEKTRUM_VTX_TELEMETRY
#undef USE_SPEKTRUM_CMS_TELEMETRY
#undef USE_TELEMETRY_SRXL
#endif

#if defined(USE_SERIALRX_SBUS) || defined(USE_SERIALRX_FPORT)
#define USE_SBUS_CHANNELS
#endif

#if !defined(USE_TELEMETRY_SMARTPORT) && !defined(USE_TELEMETRY_CRSF)
#undef USE_MSP_OVER_TELEMETRY
#endif

/* If either VTX_CONTROL or VTX_COMMON is undefined then remove common code and device drivers */
#if !defined(USE_VTX_COMMON) || !defined(USE_VTX_CONTROL)
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#endif

#if defined(USE_RX_FRSKY_SPI_D) || defined(USE_RX_FRSKY_SPI_X)
#define USE_RX_CC2500
#define USE_RX_FRSKY_SPI
#endif

#if defined(USE_RX_SFHSS_SPI)
#define USE_RX_CC2500
#endif

#if !defined(USE_RX_CC2500)
#undef USE_RX_CC2500_SPI_PA_LNA
#endif

#if !defined(USE_RX_CC2500_SPI_PA_LNA)
#undef USE_RX_CC2500_SPI_DIVERSITY
#endif

// Burst dshot to default off if not configured explicitly by target
#ifndef ENABLE_DSHOT_DMAR
#define ENABLE_DSHOT_DMAR false
#endif

// Some target doesn't define USE_ADC which USE_ADC_INTERNAL depends on
#ifndef USE_ADC
#undef USE_ADC_INTERNAL
#endif

#if (!defined(USE_SDCARD) && !defined(USE_FLASHFS)) || !defined(USE_BLACKBOX)
#undef USE_USB_MSC
#endif

#if !defined(USE_VCP)
#undef USE_USB_CDC_HID
#undef USE_USB_MSC
#endif

#if defined(USE_USB_CDC_HID) || defined(USE_USB_MSC)
#define USE_USB_ADVANCED_PROFILES
#endif


#if defined(USE_FLASH_W25M512)
#define USE_FLASH_W25M
#define USE_FLASH_M25P16
#endif

#if defined(USE_FLASH_M25P16)
#define USE_FLASH_CHIP
#endif

#if defined(USE_MAX7456)
#define USE_OSD
#endif

#if !defined(USE_OSD)
#undef USE_RX_LINK_QUALITY_INFO
#undef USE_OSD_PROFILES
#undef USE_OSD_STICK_OVERLAY
#endif

#if defined(USE_GPS_RESCUE)
#define USE_GPS
#endif

// Generate USE_SPI_GYRO or USE_I2C_GYRO
#if defined(USE_GYRO_L3G4200D) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6000) || defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU6500)
#define USE_I2C_GYRO
#endif

#if defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_L3GD20)
#define USE_SPI_GYRO
#endif

// CX10 is a special case of SPI RX which requires XN297
#if defined(USE_RX_CX10)
#define USE_RX_XN297
#endif

#ifdef USE_UNIFIED_TARGET
#define USE_CONFIGURATION_STATE

// Setup crystal frequency for backward compatibility
// Should be set to zero for generic targets and set with CLI variable set system_hse_value.
#define SYSTEM_HSE_VALUE 0
#else
#ifdef TARGET_XTAL_MHZ
#define SYSTEM_HSE_VALUE TARGET_XTAL_MHZ
#else
#define SYSTEM_HSE_VALUE (HSE_VALUE/1000000U)
#endif
#endif // USE_UNIFIED_TARGET

// Number of pins that needs pre-init
#ifdef USE_SPI
#ifndef SPI_PREINIT_COUNT
#define SPI_PREINIT_COUNT 16 // 2 x 8 (GYROx2, BARO, MAG, MAX, FLASHx2, RX)
#endif
#endif

#if (!defined(USE_FLASHFS) || !defined(USE_RTC_TIME) || !defined(USE_USB_MSC))
#undef USE_PERSISTENT_MSC_RTC
#endif

#if !defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) && !defined(USE_SERIAL_4WAY_SK_BOOTLOADER)
#undef  USE_SERIAL_4WAY_BLHELI_INTERFACE
#elif !defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && (defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) || defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

#if !defined(USE_LED_STRIP)
#undef USE_LED_STRIP_STATUS_MODE
#endif

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_STATUS_MODE)
#define USE_WS2811_SINGLE_COLOUR
#endif

#if defined(SIMULATOR_BUILD) || defined(UNIT_TEST)
// This feature uses 'arm_math.h', which does not exist for x86.
#undef USE_GYRO_DATA_ANALYSE
#endif

#ifndef USE_CMS
#undef USE_CMS_FAILSAFE_MENU
#endif

#ifndef USE_DSHOT
#undef USE_DSHOT_TELEMETRY
#endif

#ifndef USE_DSHOT_TELEMETRY
#undef USE_RPM_FILTER
#undef USE_DSHOT_TELEMETRY_STATS
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

#if !defined(USE_DMA_SPEC)
#undef USE_TIMER_MGMT
#endif

#if defined(USE_TIMER_MGMT)
#undef USED_TIMERS
#else
#undef USE_UNIFIED_TARGET
#endif

#if !defined(USE_RANGEFINDER)
#undef USE_RANGEFINDER_HCSR04
#undef USE_RANGEFINDER_SRF10
#undef USE_RANGEFINDER_HCSR04_I2C
#undef USE_RANGEFINDER_VL53L0X
#undef USE_RANGEFINDER_UIB
#undef USE_RANGEFINDER_TF
#endif

// TODO: Remove this once HAL support is fixed for ESCSERIAL
#ifdef STM32F7
#undef USE_ESCSERIAL
#endif
