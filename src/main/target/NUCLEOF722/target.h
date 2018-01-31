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

#define TARGET_BOARD_IDENTIFIER "NU72"

#define USBD_PRODUCT_STRING "NucleoF722"

//#define USE_ESC_TELEMETRY

#define LED0_PIN   PB7  // blue
#define LED1_PIN   PB14 // red

#define BEEPER   PA0
#define BEEPER_INVERTED

#define USE_ACC
#define USE_FAKE_ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN CW270_DEG

#define USE_GYRO
#define USE_FAKE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN CW270_DEG

// MPU6050 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define MPU_INT_EXTI PB15
#define USE_EXTI

#define USE_MAG
#define USE_FAKE_MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN CW270_DEG_FLIP

#define USE_BARO
#define USE_FAKE_BARO
#define USE_BARO_MS5611

#define USABLE_TIMER_CHANNEL_COUNT 11

#define USE_VCP
#define VBUS_SENSING_PIN PA9

//#define USE_UART1
//#define UART1_RX_PIN PA10
//#define UART1_TX_PIN PA9

#define USE_UART2
#define UART2_RX_PIN PD6
#define UART2_TX_PIN PD5

#define USE_UART3
#define UART3_RX_PIN PD9
#define UART3_TX_PIN PD8

#define USE_UART4
#define UART4_RX_PIN PC11
#define UART4_TX_PIN PC10

//#define USE_UART5
#define UART5_RX_PIN PD2
#define UART5_TX_PIN PC12

//#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

//#define USE_UART7
#define UART7_RX_PIN PE7
#define UART7_TX_PIN PE8

//#define USE_UART8
#define UART8_RX_PIN PE0
#define UART8_TX_PIN PE1

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 6 //VCP, USART2, USART3, UART4,SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB15 // (Hardware=0, PPM)

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_4

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI4_NSS_PIN            PE11
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14

//#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PF14

#define SDCARD_SPI_INSTANCE                 SPI4
#define SDCARD_SPI_CS_PIN                   SPI4_NSS_PIN

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 422kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 8 // 27MHz

#define SDCARD_DMA_STREAM_TX_FULL           DMA2_Stream1
#define SDCARD_DMA_CHANNEL                  4

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE                  (I2CDEV_1)
#define I2C1_SCL                    PB8
#define I2C1_SDA                    PB9

#define USE_ADC
#define VBAT_ADC_PIN                PA3
#define CURRENT_METER_ADC_PIN       PC0
#define RSSI_ADC_GPIO_PIN           PC3

//#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11))
