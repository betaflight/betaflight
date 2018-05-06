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
#define TARGET_BOARD_IDENTIFIER "S7X2"

#define USBD_PRODUCT_STRING     "S7X2"

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

// MPU6500 interrupt
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

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
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN CW270_DEG_FLIP

#define USE_BARO
#define USE_FAKE_BARO
#define USE_BARO_MS5611

#define USE_SDCARD

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define USE_I2C
#define USE_I2C_DEVICE_1

#define USE_VCP

#define USE_UART1
#define USE_UART3
#define USE_UART6
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL

#define USE_SPI

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define USE_ADC

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 25

#define USE_TIMER_MGMT
