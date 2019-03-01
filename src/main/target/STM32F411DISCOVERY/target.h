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

#define TARGET_BOARD_IDENTIFIER "S411" // STM Discovery F411
#define USBD_PRODUCT_STRING     "STM32F411DISCOVERY"

#define USE_SENSOR_NAMES

#define LED0_PIN                PD15 // Blue
#define LED1_PIN                PD13 // Orange

#define USE_BEEPER
#define BEEPER_PIN              PD12 // Green LED
#define BEEPER_PWM_HZ           2000 // Beeper PWM frequency in Hz

// Gyro
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_GYRO

#define USE_GYRO_L3GD20
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PE3
#define GYRO_1_ALIGN            CW180_DEG

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PE1
#define USE_MPU_DATA_READY_SIGNAL

// Acc
#define USE_ACC

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB9

#define MPU_I2C_INSTANCE        (I2CDEV_1)
#define USE_ACC_LSM303DLHC
#define ACC_1_ALIGN             CW0_DEG

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

// Serial
#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PA9

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA15

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       4 // VCP, USART1, USART2, USART6

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

// ADC
#define USE_ADC
#define VBAT_ADC_PIN            PC1
#define CURRENT_METER_ADC_PIN   PC2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8

#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

// Buses & Timers
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTH         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )

