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

#define USE_PARAMETER_GROUPS
// type conversion warnings.
// -Wconversion can be turned on to enable the process of eliminating these warnings
//#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
// -Wpadded can be turned on to check padding of structs
//#pragma GCC diagnostic warning "-Wpadded"

//#define SCHEDULER_DEBUG // define this to use scheduler debug[] values. Undefined by default for performance reasons
#define DEBUG_MODE DEBUG_NONE // change this to change initial debug mode

#define I2C1_OVERCLOCK true
#define I2C2_OVERCLOCK true

#ifdef STM32F1
#define MINIMAL_CLI
// Using RX DMA disables the use of receive callbacks
#define USE_UART1_RX_DMA
#define USE_UART1_TX_DMA
#endif

#ifdef STM32F3
#define MINIMAL_CLI
#define USE_DSHOT
#define USE_GYRO_DATA_ANALYSE
#endif

#ifdef STM32F4
#define USE_DSHOT
#define USE_ESC_SENSOR
#define I2C3_OVERCLOCK true
#define USE_GYRO_DATA_ANALYSE
#define USE_ADC
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID

#if defined(STM32F40_41xxx) || defined(STM32F411xE)
#define USE_OVERCLOCK
#endif

#endif // STM32F4

#ifdef STM32F722xx
#define USE_ITCM_RAM
#endif
#ifdef STM32F7
#define USE_DSHOT
#define USE_ESC_SENSOR
#define I2C3_OVERCLOCK true
#define I2C4_OVERCLOCK true
#define USE_GYRO_DATA_ANALYSE
#endif

#if defined(STM32F4) || defined(STM32F7)
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

#ifdef USE_ITCM_RAM
#define FAST_CODE __attribute__((section(".tcm_code")))
#else
#define FAST_CODE
#endif // USE_ITCM_RAM

#ifdef USE_FAST_RAM
#ifdef __APPLE__
#define FAST_RAM                    __attribute__ ((section("__DATA,__.fastram_bss"), aligned(4)))
#else
#define FAST_RAM                    __attribute__ ((section(".fastram_bss"), aligned(4)))
#endif
#else
#define FAST_RAM
#endif // USE_FAST_RAM


#define USE_CLI
#define USE_PPM
#define USE_PWM
#define USE_SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_XBUS       // JR

#if (FLASH_SIZE > 64)
#define MAX_PROFILE_COUNT 3
#else
#define MAX_PROFILE_COUNT 2
#endif

#if (FLASH_SIZE > 64)
#define USE_BLACKBOX
#define USE_RESOURCE_MGMT
#define USE_RUNAWAY_TAKEOFF     // Runaway Takeoff Prevention (anti-taz) - Marked for removal
#define USE_TELEMETRY
#define USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_LTM
#define USE_GYRO_FAST_KALMAN
#define USE_TELEMETRY_SMARTPORT
#define USE_LED_STRIP
#endif

#if (FLASH_SIZE > 128)
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_CAMERA_CONTROL
#define USE_CMS
#define USE_COPY_PROFILE_CMS_MENU
#define USE_DSHOT_DMAR
#define USE_GYRO_OVERFLOW_CHECK
#define USE_HUFFMAN
#define USE_MSP_DISPLAYPORT
#define USE_MSP_OVER_TELEMETRY
#define USE_OSD
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_PINIO
#define USE_PINIOBOX
#define USE_RCDEVICE
#define USE_RTC_TIME
#define USE_RX_MSP
#define USE_SERIALRX_FPORT      // FrSky FPort
#define USE_TELEMETRY_CRSF
#define USE_TELEMETRY_SRXL
#define USE_VIRTUAL_CURRENT_METER
#define USE_VTX_COMMON
#define USE_VTX_CONTROL
#define USE_VTX_SMARTAUDIO
#define USE_VTX_TRAMP
#define USE_GYRO_BIQUAD_RC_FIR2

#ifdef USE_SERIALRX_SPEKTRUM
#define USE_SPEKTRUM_BIND
#define USE_SPEKTRUM_BIND_PLUG
#define USE_SPEKTRUM_REAL_RSSI
#define USE_SPEKTRUM_FAKE_RSSI
#define USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#define USE_SPEKTRUM_VTX_CONTROL
#define USE_SPEKTRUM_VTX_TELEMETRY
#define USE_SPEKTRUM_CMS_TELEMETRY
#endif
#endif

#if (FLASH_SIZE > 256)
#define USE_ALT_HOLD
#define USE_DASHBOARD
#define USE_GPS
#define USE_GPS_NMEA
#define USE_GPS_UBLOX
#define USE_NAV
#define USE_OSD_ADJUSTMENTS
#define USE_SENSOR_NAMES
#define USE_SERIALRX_JETIEXBUS
#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_IBUS_EXTENDED
#define USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_MAVLINK
#define USE_UNCOMMON_MIXERS
#endif
