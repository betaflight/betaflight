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

// Treat the target as generic, and expect manufacturer id / board name
// to be supplied when the board is configured for the first time
#define GENERIC_TARGET

#define TARGET_BOARD_IDENTIFIER "S405"

#define USBD_PRODUCT_STRING     "S405"

#define USE_BEEPER

// MPU interrupt
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_GYRO_EXTI

#define USE_ACC
#define USE_GYRO

#define USE_ACC_MPU6050
#define USE_GYRO_MPU6050
#define USE_ACC_MPU6500
#define USE_GYRO_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_ICM20689
#define USE_GYRO_SPI_ICM20689
// Other USE_ACCs and USE_GYROs should follow

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_SPI_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define USE_MAG_AK8963
#define USE_MAG_SPI_AK8963

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_SPI_MS5611
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define USE_BARO_LPS
#define USE_BARO_SPI_LPS

#define USE_SDCARD
#define USE_SDCARD_SPI

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_MAX7456

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define I2C_FULL_RECONFIGURABILITY

#define USE_VCP

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define USE_INVERTER
#define SERIAL_PORT_COUNT       9

#define USE_ESCSERIAL

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define SPI_FULL_RECONFIGURABILITY

#define USE_ADC

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 70

#define USE_TIMER_MGMT
