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

#define TARGET_BOARD_IDENTIFIER "MSKY" // Micro sciSKY

#define LED0_PIN                PB3
#define LED1_PIN                PB4

#define USE_BEEPER
#define BEEPER_PIN              PA12

#define BARO_XCLR_PIN           PC13
#define BARO_EOC_PIN            PC14

#define INVERTER_PIN_UART2      PB2 // PB2 (BOOT1) abused as inverter select GPIO

#define USE_EXTI
#define MAG_INT_EXTI            PC14
#define USE_MPU_DATA_READY_SIGNAL
#define USE_MAG_DATA_READY_SIGNAL

// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

#define USE_SPI
#define USE_SPI_DEVICE_2

#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_1_ALIGN            CW0_DEG

#define USE_ACC
#define USE_ACC_MPU6050

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG

#define USE_UART1
#define USE_UART2
#define SERIAL_PORT_COUNT       2

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)

#define BRUSHED_MOTORS
#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SPEKTRUM1024
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER

#undef USE_SERVOS
#define USE_QUAD_MIXER_ONLY


// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
