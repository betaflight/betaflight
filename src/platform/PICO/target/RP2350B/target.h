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

#ifdef PICO_TRACE
//void _bprintf(const char * format, ...);
//#define bprintf(fmt,...) _bprintf(fmt, ## __VA_ARGS__)
//#define bprintf(fmt,...) do {stdio_printf(fmt, ## __VA_ARGS__); stdio_printf("\n"); } while (0)
#include "pico_trace.h"
#define bprintf tprintf
//int __wrap_printf(const char * fmt, ...);
//#define bprintf __wrap_printf
#else
#define bprintf(fmt,...)
#endif

#ifndef RP2350B
#define RP2350B
#endif

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "235B"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight RP2350B"
#endif

#define USE_MULTICORE
#define USE_IO
#define USE_UART0
#define USE_UART1

#define USE_SPI
#define USE_SPI_DEVICE_0
#define USE_SPI_DEVICE_1
#define USE_SPI_DMA_ENABLE_LATE

#define USE_I2C
#define USE_I2C_DEVICE_0
#define USE_I2C_DEVICE_1

// one of these ...
// #define USE_SPI_DMA_ENABLE_EARLY
#define USE_SPI_DMA_ENABLE_LATE

#undef USE_SOFTSERIAL1
#undef USE_SOFTSERIAL2

#define USE_VCP

#undef USE_TRANSPONDER
#undef USE_FLASH
#undef USE_FLASH_CHIP
#undef USE_SDCARD

#undef USE_TIMER
#undef USE_RCC

//////////////////////////
// Radio RX
// TODO tidy up, introduce more possible options
#undef USE_RX_SPI

#undef USE_RX_PWM
#undef USE_RX_PPM
#undef USE_RX_CC2500
#undef USE_RX_EXPRESSLRS // <-- not this one, it's for SPI
#undef USE_RX_SX1280 // no, it's for SPI

//#undef USE_CRSF <-- this one

// #undef USE_SERIALRX_CRSF // <-- this one
#undef USE_SERIALRX_GHST
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_JETIEXBUS
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef USE_SERIALRX_FPORT

// TODO persistent objects -> crsfRxUpdateBaudRate -> crsf, telemetry
//#undef USE_TELEMETRY_CRSF
//#undef USE_TELEMETRY
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

//////////////////////

#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_MULTI_GYRO

#undef USE_RANGEFINDER_HCSR04
#undef USE_MAX7456
#undef USE_MAG
#undef USE_MAG_HMC5883
#undef USE_MAG_SPI_HMC5883
#undef USE_VTX_RTC6705
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_SRXL
#undef USE_OSD
#undef USE_SPEKTRUM
#undef USE_SPEKTRUM_BIND

// #undef USE_CLI
#undef USE_SERIAL_PASSTHROUGH

// #undef USE_MSP
#undef USE_MSP_UART
#undef USE_MSP_DISPLAYPORT

/* DMA Settings */
//#undef USE_DMA
#define DMA_IRQ_CORE_NUM 1 // Use core 1 for DMA IRQs
#undef USE_DMA_SPEC // not yet required - possibly won't be used at all

#undef USE_DSHOT_TELEMETRY
#undef USE_ESC_SENSOR

#undef USE_PWM_OUTPUT

#undef USE_GPS
#undef USE_GPS_UBLOX
#undef USE_GPS_MTK
#undef USE_GPS_NMEA
#undef USE_GPS_SERIAL
#undef USE_GPS_SONAR
#undef USE_GPS_UBLOX7
#undef USE_GPS_UBLOX8
#undef USE_GPS_RESCUE

#undef USE_VTX
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#undef USE_SPEKTRUM_VTX_CONTROL
#undef USE_VTX_COMMON
#undef USE_FLASH
#undef USE_FLASH_CHIP
#undef USE_FLASH_M25P16
#undef USE_FLASH_W25N01G
#undef USE_FLASH_W25N02K
#undef USE_FLASH_W25M
#undef USE_FLASH_W25M512
#undef USE_FLASH_W25M02G
#undef USE_FLASH_W25Q128FV
#undef USE_FLASH_PY25Q128HA
#undef USE_FLASH_W25Q64FV

#undef USE_RPM_LIMIT

#undef USE_SERVOS
#undef USE_LED_STRIP
#undef USE_OSD
#undef USE_OSD_SD
#undef USE_OSD_HD
#undef USE_OSD_QUICK_MENU
#undef USE_POSITION_HOLD

//#define FLASH_PAGE_SIZE                     0x1
#define CONFIG_IN_FLASH

// Pico flash writes are all aligned and in batches of FLASH_PAGE_SIZE (256)
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE   FLASH_PAGE_SIZE
#define FLASH_CONFIG_BUFFER_TYPE            uint8_t

// to be moved to a config file once target if working
//   defaults as per Laurel board for now

// Laurel to enable secondary voltage supplies from main battery
#define PICO_BEC_5V_ENABLE_PIN PA14
// Enable 9V if/when we need it (for DVTX?)
// #define PICO_BEC_9V_ENABLE_PIN PA15

//#undef USE_BARO
#define USE_BARO_DPS310
#undef USE_BARO_MS5611
#undef USE_BARO_SPI_MS5611
#undef USE_BARO_BMP280
#undef USE_BARO_SPI_BMP280
#undef USE_BARO_BMP388
#undef USE_BARO_SPI_BMP388
#undef USE_BARO_LPS
#undef USE_BARO_SPI_LPS
#undef USE_BARO_QMP6988
#undef USE_BARO_SPI_QMP6988
#undef USE_BARO_SPI_DPS310
#undef USE_BARO_BMP085
#undef USE_BARO_2SMBP_02B
#undef USE_BARO_SPI_2SMBP_02B
#undef USE_BARO_LPS22DF
#undef USE_BARO_SPI_LPS22DF

#define BARO_I2C_INSTANCE    I2CDEV_0
#define I2C0_SDA_PIN         PA44
#define I2C0_SCL_PIN         PA45

// PA20, PA21 on laurel are set aside for jumper to connect to Radio RX on UART2 (software PIO uart)
// but let's assign them for h/w UART1 for now [UART1 TX at 20 and RX at 21 are available on RP2350]
#define UART1_TX_PIN         PA20
#define UART1_RX_PIN         PA21
// [target.mk] Switch PICO_DEFAULT_UART to 0 and change PICO_DEFAULT_UART_TX,RX_PINs to 34, 35
// which are available to UART0, and appear on spare UART connector J10

#define LED0_PIN             PA6
#define LED1_PIN             PA7

#define SPI0_SCK_PIN         PA2
#define SPI0_SDI_PIN         PA4
#define SPI0_SDO_PIN         PA3

#define SPI1_SCK_PIN         PA26
#define SPI1_SDI_PIN         PA24
#define SPI1_SDO_PIN         PA27

#define SDCARD_CS_PIN        PA25
#define FLASH_CS_PIN         PA25
#define MAX7456_SPI_CS_PIN   PA17

#define GYRO_1_CS_PIN        PA1
#define GYRO_2_CS_PIN        NONE

#if 0 // def TEST_DSHOT_ETC
// testing motor1 and motor2 pins -> GPIO_SPARE1, GPIO_SPARE2
#define MOTOR1_PIN           PA32
#define MOTOR2_PIN           PA33
#define MOTOR3_PIN           PA30
#define MOTOR4_PIN           PA31
#else
#define MOTOR1_PIN           PA28
#define MOTOR2_PIN           PA29
#define MOTOR3_PIN           PA30
#define MOTOR4_PIN           PA31
#endif

#define MAX7456_SPI_INSTANCE SPI1
#define SDCARD_SPI_INSTANCE  SPI1
#define GYRO_1_SPI_INSTANCE  SPI0

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_ICM42688P

#if 1 // TODO review
#undef USE_ACCGYRO_LSM6DSV16X
#undef USE_GYRO_SPI_ICM20689
#undef USE_GYRO_SPI_MPU6000
#undef USE_GYRO_SPI_MPU6500
#undef USE_GYRO_SPI_MPU9250
#undef USE_ACCGYRO_LSM6DSO
#undef USE_ACCGYRO_BMI160
#undef USE_ACCGYRO_BMI270
#endif
   
//#define USE_FLASH
//#define USE_FLASH_W25Q128FV
//#define USE_MAX7456

/*

SPI0_CS         PA1
SPI0_SCLK       PA2
SPI0_COPI       PA3
SPI0_CIPO       PA4
BUZZER          PA5
LED0            PA6
LED1            PA7
UART1_TX        PA8
UART1_RX        PA9
I2C1_SDA        PA10
I2C1_SCL        PA11
UART0_TX        PA12
UART0_RX        PA13

OSD_CS          PA17

UART2_TX        PA20
UART2_RX        PA21

GYRO_INT        PA22

GYRO_CLK        PA23

SPI1_CIPO       PA24
SPI1_CS         PA25
SPI1_SCLK       PA26
SPI1_COPI       PA27

MOTOR1          PA28
MOTOR2          PA29
MOTOR3          PA30
MOTOR4          PA31

SPARE1          PA32
SPARE2          PA33

UART3_TX        PA34
UART3_RX        PA35

DVTX_SBUS_RX    PA36
TELEM_RX        PA37
LED_STRIP       PA38
RGB_LED         PA39

VBAT_SENSE      PA40
CURR_SENSE      PA41
ADC_SPARE       PA42

I2C0_SDA        PA44
I2C0_SCL        PA45

SPARE3          PA47

*/

#define SPIDEV_COUNT 2
#define MAX_SPI_PIN_SEL 6

#define I2CDEV_COUNT 2

#define UART_RX_BUFFER_SIZE 1024
#define UART_TX_BUFFER_SIZE 1024

#define UARTHARDWARE_MAX_PINS 12
#define UART_TRAIT_AF_PORT 1

#define I2CDEV_COUNT 2
