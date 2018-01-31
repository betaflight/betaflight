/*
 * This software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "N446" // STM32 Nucleo F446RE
#define USBD_PRODUCT_STRING     "NucleoF446RE"

#define LED0_PIN                PA5  // Onboard LED

//#define BEEPER                  PD12

#define USE_EXTI

#define USE_SPI
//#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN PB12
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15

#define USE_GYRO
#define USE_FAKE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250

#define USE_ACC
#define USE_FAKE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU9250

#define MPU9250_CS_PIN PB12
#define MPU9250_SPI_INSTANCE SPI2

#define MPU6500_CS_PIN PB12
#define MPU6500_SPI_INSTANCE SPI2

#define USE_EXTI

//#define MPU_INT_EXTI PC13
//#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_BARO
#define USE_FAKE_BARO
//#define USE_BARO_BMP085
//#define USE_BARO_BMP280
//#define USE_BARO_MS5611

//#define USE_MAX7456
//#define MAX7456_SPI_INSTANCE    SPI2
//#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

#define USE_CMS

//#define USE_SDCARD
//
//#define SDCARD_SPI_INSTANCE     SPI2
//#define SDCARD_SPI_CS_PIN       PB12
//// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
//#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
//// Divide to under 25MHz for normal operation:
//#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 2
//
//// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
//#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USE_MAG
#define USE_FAKE_MAG
//#define USE_MAG_AK8963
//#define USE_MAG_AK8975
//#define USE_MAG_HMC5883

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1
// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define RX_CE_PIN               PC7 // D9
#define RX_NSS_PIN              PB6 // D10
#define RX_SCK_PIN              PA5 // D13
#define RX_MISO_PIN             PA6 // D12
#define RX_MOSI_PIN             PA7 // D11
// NUCLEO has NSS on PB6, rather than the standard PA4
#define SPI1_NSS_PIN            RX_NSS_PIN
#define SPI1_SCK_PIN            RX_SCK_PIN
#define SPI1_MISO_PIN           RX_MISO_PIN
#define SPI1_MOSI_PIN           RX_MOSI_PIN

#define USE_RX_NRF24
#define USE_RX_CX10
#define USE_RX_H8_3D
#define USE_RX_INAV
#define USE_RX_SYMA
#define USE_RX_V202
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_H8_3D

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_UART3
#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

//#define USE_UART4
//#define USE_UART5

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8  // (HARDARE=0,PPM)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_I2C

#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PB10, shared with UART3TX
#define I2C2_SDA                NONE // PB11, shared with UART3RX

#define USE_I2C_DEVICE_3
#define I2C3_SCL                NONE // PA8
#define I2C3_SDA                NONE // PC9
#define I2C_DEVICE              (I2CDEV_2)

#define USE_ADC
#define ADC_INSTANCE            ADC1
//#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#define USE_ESC_SENSOR

#define USE_SONAR
#define SONAR_TRIGGER_PIN       PB0
#define SONAR_ECHO_PIN          PB1

#define MAX_SUPPORTED_MOTORS    12

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)|BIT(13)))
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )
