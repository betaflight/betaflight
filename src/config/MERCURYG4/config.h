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

#define FC_TARGET_MCU                   STM32G47X

#define BOARD_NAME                      MERCURYG4
#define MANUFACTURER_ID                 ANYL

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_M25P16

#define BEEPER_PIN                      PA10

#define MOTOR1_PIN                      PC6
#define MOTOR2_PIN                      PA4
#define MOTOR3_PIN                      PB0
#define MOTOR4_PIN                      PB1

#define UART1_TX_PIN                    PB6
#define UART1_RX_PIN                    PB7

// UART 2: ELRS receiver
#define UART2_TX_PIN                    PB3
#define UART2_RX_PIN                    PB4

#define UART3_TX_PIN                    PB10
#define UART3_RX_PIN                    PB11

// UART 4: OSD
#define UART4_TX_PIN                    PC10
#define UART4_RX_PIN                    PC11

// LPUART 1:
#define UART9_TX_PIN                    PA2
#define UART9_RX_PIN                    PA3

// I2C
#define I2C1_SCL_PIN                    PA15
#define I2C1_SDA_PIN                    PB9

// I2C2: Barometer
#define I2C2_SCL_PIN                    PA9
#define I2C2_SDA_PIN                    PA8
#define BARO_I2C_INSTANCE               (I2CDEV_2)

// TODO #define LED0_PIN
// TODO #define LED1_PIN

#define ADC_VBAT_PIN                    PA1 // ADC2 in  2
#define ADC_CURR_PIN                    PB2 // ADC2 in 12

#define FLASH_CS_PIN                    PA0
#define FLASH_SPI_INSTANCE              SPI2

#define GYRO_1_EXTI_PIN                 PC13
#define GYRO_1_CS_PIN                   PB12
#define GYRO_1_SPI_INSTANCE             SPI1
#define GYRO_1_ALIGN                    CW90_DEG
#define GYRO_1_ALIGN_YAW                1800

// SPI 1: IMU
#define SPI_SCK1_PIN                    PA5
#define SPI_MISO1_PIN                   PA6
#define SPI_MOSI1_PIN                   PA7

// SPI 2: Blackbox flash
#define SPI_SCK2_PIN                    PB13
#define SPI_MISO2_PIN                   PB14
#define SPI_MOSI2_PIN                   PB15

/*

# We don't use ADC 1 directly, but it is mandatory to assign it, since
# the MCU's temperature sensor, and VDDA measurement are on ADC1.
dma ADC 1 6
dma ADC 2 7
dma SPI_MOSI 1 8
dma SPI_MISO 1 9
dma SPI_MOSI 2 10
dma SPI_MISO 2 11


# Motors and timer pins for burst DMA DSHOT
timer C06 AF2    # Tim3 ch1
timer A04 AF2    # Tim3 ch2
timer B00 AF2    # Tim3 ch3
timer B01 AF2    # Tim3 ch4

resource MOTOR 1 C06
resource MOTOR 2 A04
resource MOTOR 3 B00
resource MOTOR 4 B01

timer A10 AF10  # Beeper: Tim2 ch4
resource BEEPER 1 A10
*/
#define TIMER_PIN_MAPPING               TIMER_PIN_MAP( 0, BEEPER_PIN , 2,  -1) \
                                        TIMER_PIN_MAP( 1, MOTOR1_PIN , 1,  -1) \
                                        TIMER_PIN_MAP( 2, MOTOR2_PIN , 1,  -1) \
                                        TIMER_PIN_MAP( 3, MOTOR3_PIN , 1,  -1) \
                                        TIMER_PIN_MAP( 4, MOTOR4_PIN , 1,  -1)


#define ADC_INSTANCE                    ADC1

#define ADC1_DMA_OPT                    0
#define ADC2_DMA_OPT                    0

#define SPI1_TX_DMA_OPT                 0
#define SPI1_RX_DMA_OPT                 0
#define SPI2_TX_DMA_OPT                 0
#define SPI2_RX_DMA_OPT                 0

#define UART1_TX_DMA_OPT                0
#define UART1_RX_DMA_OPT                0
#define UART2_TX_DMA_OPT                0
#define UART2_RX_DMA_OPT                0
#define UART3_TX_DMA_OPT                0
#define UART3_RX_DMA_OPT                0
#define UART4_TX_DMA_OPT                0
#define UART4_RX_DMA_OPT                0
#define UART5_TX_DMA_OPT                0
#define UART5_RX_DMA_OPT                0

#define BEEPER_INVERTED

#define SYSTEM_HSE_MHZ                  16
#define PID_PROCESS_DENOM               2

#define DEFAULT_RX_FEATURE              FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER               SERIALRX_CRSF

#define DEFAULT_BLACKBOX_DEVICE         BLACKBOX_DEVICE_FLASH

#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE     118

/*

# Barometer
set baro_hardware = DPS310
set baro_i2c_address = 119  # 0x77
set i2c2_clockspeed_khz = 400
set i2c2_pullup = ON  # No hardware PUs are present.
set mco_on_pa8 = OFF

# ADC
set adc_device = 2
set vbat_divider = 5
set vbat_multiplier = 2

# Control Rx
set serialrx_provider = CRSF
# For RSSI on the OSD, use AUX11/ch15 for Link Quality Indictaor, or AUX12/ch16 for RSSI dBm
set rssi_channel = 15

# Motors and timer pins for burst DMA DSHOT
timer PC6 AF2    # Tim3 ch1
timer PA4 AF2    # Tim3 ch2
timer PB0 AF2    # Tim3 ch3
timer PB1 AF2    # Tim3 ch4
timer PA10 AF10  # Beeper: Tim2 ch4

# DMA
# Assign DMA channels explicitly. Leaving the default assignments triggers
# system failures indicated by the attitude indication responding strangely
# on the `Setup` page.

# We don't use ADC 1 directly, but it is mandatory to assign it, since
# the MCU's temperature sensor, and VDDA measurement are on ADC1.

dma ADC 1 6
dma ADC 2 7
dma SPI_MOSI 1 8
dma SPI_MISO 1 9
dma SPI_MOSI 2 10
dma SPI_MISO 2 11

# Features
feature TELEMETRY
feature OSD
feature RX_SERIAL  # For the built-en ELRS receiver, via CRSF.

# Configure UART4 for digital (MSP Displayport) OSD.
serial 3 1 115200 57600 0 115200
# Configure UART2 for the built-in ELRS receiver, via CRSF.
serial 1 64 115200 57600 0 115200

# Master
set system_hse_mhz = 16
# 4kHz PID loop frequency.
set pid_process_denom = 2

*/
