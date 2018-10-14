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

#ifndef USE_BARO
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

#if !defined(USE_TELEMETRY_CRSF)
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
#define USE_RX_CC2500_BIND
#define USE_RX_FRSKY_SPI
#endif

#if defined(USE_RX_SFHSS_SPI)
#define USE_RX_CC2500
#define USE_RX_CC2500_BIND
#endif

// Burst dshot to default off if not configured explicitly by target
#ifndef ENABLE_DSHOT_DMAR
#define ENABLE_DSHOT_DMAR false
#endif

// Some target doesn't define USE_ADC which USE_ADC_INTERNAL depends on
#ifndef USE_ADC
#undef USE_ADC_INTERNAL
#endif

#if (!defined(USE_SDCARD) && !defined(USE_FLASHFS)) || !(defined(STM32F4) || defined(STM32F7))
#undef USE_USB_MSC
#endif

#if !defined(USE_VCP)
#undef USE_USB_CDC_HID
#endif

#if defined(USE_USB_CDC_HID) || defined(USE_USB_MSC)
#define USE_USB_ADVANCED_PROFILES
#endif

// Determine if the target could have a 32KHz capable gyro
#if defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_SPI_ICM20689)
#define USE_32K_CAPABLE_GYRO
#endif

#if defined(USE_FLASH_W25M512)
#define USE_FLASH_W25M
#define USE_FLASH_M25P16
#endif

#if defined(USE_FLASH_M25P16)
#define USE_FLASH
#endif

#if defined(USE_MAX7456)
#define USE_OSD
#endif

#if defined(USE_GPS_RESCUE)
#define USE_GPS
#endif

// Generate USE_SPI_GYRO or USE_I2C_GYRO
#if defined(USE_GYRO_L3G4200D) || defined(USE_GYRO_L3GD20) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6000) || defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU6500)
#define USE_I2C_GYRO
#endif

#if defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
#define USE_SPI_GYRO
#endif

// CX10 is a special case of SPI RX which requires XN297
#if defined(USE_RX_CX10)
#define USE_RX_XN297
#endif
