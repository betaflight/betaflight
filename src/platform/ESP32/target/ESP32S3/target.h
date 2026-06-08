/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "ES3A"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight - ESP32S3"
#endif

#define USE_UART0
#define USE_UART1
#define USE_UART2
#define UART_RX_BUFFER_SIZE 1024
#define UART_TX_BUFFER_SIZE 1024
#define UARTHARDWARE_MAX_PINS 8
#define UART_TRAIT_AF_PORT 1

#define USE_SPI
#define SPIDEV_COUNT 2
#define USE_SPI_DEVICE_0
#define USE_SPI_DEVICE_1
#define USE_SPI_DMA_ENABLE_LATE
#define MAX_SPI_PIN_SEL 4

#define USE_I2C
#define I2CDEV_COUNT 2
#define USE_I2C_DEVICE_0
#define USE_I2C_DEVICE_1

#define USE_VCP

#undef USE_SOFTSERIAL1
#undef USE_SOFTSERIAL2
#undef USE_TRANSPONDER
#undef USE_FLASH
#undef USE_FLASH_CHIP
#undef USE_RCC

// Per-board config.h opts into USE_SDCARD / USE_SDCARD_SPI when the
// board carries an SPI SD socket; target.h stays MCU-shape only.

#ifndef DEFAULT_BLACKBOX_DEVICE
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SERIAL
#endif

// Config in RAM for initial bring-up (no persistence across reboots).
// The memory-mapped flash-config path (CONFIG_IN_FLASH) reads/writes via the
// __config_start mapped address, which on ESP32-S3 needs a byte-accessible
// DROM-window partition mapped by the bootloader; revisit for persistence.
#define CONFIG_IN_RAM
#define EEPROM_SIZE 32768

/* DMA Settings */
#undef USE_DMA_SPEC

// Unsupported features for ESP32 bring-up
#undef USE_RX_SPI
#undef USE_RX_PWM
#undef USE_RX_PPM
#undef USE_RX_CC2500
#undef USE_RX_EXPRESSLRS
#undef USE_RX_SX1280
#undef USE_SERIALRX_JETIEXBUS
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS

// Spektrum requires USE_TIMER for bind support - deferred until timer driver exists
#undef USE_SERIALRX_SPEKTRUM

// Serial RX protocols enabled (UART-based, no special hardware needed):
//   CRSF        - enabled via common_pre.h (most common modern protocol)
//   S.BUS       - enabled via common_pre.h (hardware inversion supported)
//   GHST        - enabled via common_pre.h (ImmersionRC Ghost)
//   IBUS        - enabled via common_pre.h (FlySky)
//   FPORT       - enabled via common_pre.h (FrSky FPort)

#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_IBUS_EXTENDED
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_SRXL

// Telemetry protocols enabled:
//   CRSF        - enabled via common_pre.h (paired with CRSF RX)
//   GHST        - enabled via common_pre.h (paired with GHST RX)

#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_MULTI_GYRO

// IMU: InvenSense MPU-9250 on SPI device 0 (FSPI). Wiring on the lonely binary
// (ESP32-S3 GPIO): SCK=GPIO12, MISO=GPIO13, MOSI=GPIO11, CS=GPIO10, INT=GPIO9.
// (USE_VIRTUAL_GYRO/ACC can be re-enabled instead for sensorless bring-up.)
#define USE_GYRO
#define USE_ACC
#define USE_GYRO_SPI_MPU9250
#define USE_ACC_SPI_MPU9250

#define SPI0_SCK_PIN            PA12
#define SPI0_SDI_PIN            PA13   // MISO
#define SPI0_SDO_PIN            PA11   // MOSI

#define GYRO_1_SPI_INSTANCE     SPI0
#define GYRO_1_CS_PIN           PA10
#define GYRO_1_EXTI_PIN         PA9
#define GYRO_1_ALIGN            CW0_DEG

// WS2812 addressable LED on GPIO48, used as a single status LED. The LED-strip
// feature is enabled by default so it acts as a status indicator out of reset.
#define LED_STRIP_PIN           PA48
#define DEFAULT_FEATURES        (FEATURE_LED_STRIP)

// PWM motor outputs (LEDC-driven), 4 motors for a quad. No DShot ESCs on the
// board yet, so default to standard PWM; DShot stays available and selectable
// via motor_pwm_protocol. Wiring (ESP32-S3 GPIO): M1=38, M2=39, M3=40, M4=41.
#define DEFAULT_MOTOR_PROTOCOL  MOTOR_PROTOCOL_PWM
#define MOTOR1_PIN              PA38
#define MOTOR2_PIN              PA39
#define MOTOR3_PIN              PA40
#define MOTOR4_PIN              PA41

#undef USE_RANGEFINDER_HCSR04
#undef USE_MAG
#undef USE_MAG_HMC5883
#undef USE_MAG_SPI_HMC5883
#undef USE_VTX_RTC6705
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_SRXL
#undef USE_SPEKTRUM_BIND

#undef USE_MSP_DISPLAYPORT

#undef USE_DSHOT_BITBANG
#undef USE_DSHOT_TELEMETRY
#undef USE_ESC_SENSOR

#undef USE_VTX
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#undef USE_SPEKTRUM_VTX_CONTROL
#undef USE_VTX_COMMON

#undef USE_RPM_LIMIT
#undef USE_OSD_HD

#undef USE_USB_MSC
