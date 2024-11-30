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
#define TARGET_BOARD_IDENTIFIER "R235"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight RP2350"
#endif

#undef USE_DMA
#undef USE_FLASH
#undef USE_FLASH_CHIP

#undef USE_TIMER
#undef USE_SPI
#undef USE_I2C
#undef USE_UART
#undef USE_DSHOT
#undef USE_RCC
#undef USE_CLI
#undef USE_PWM_OUTPUT
#undef USE_RX_PWM
#undef USE_RX_PPM
#undef USE_RX_SPI
#undef USE_RX_CC2500
#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_MULTI_GYRO
#undef USE_GYRO
#undef USE_ACC
#undef USE_SPI_GYRO
#undef USE_BARO

#undef USE_ACC_MPU6500
#undef USE_GYRO_MPU6500
#undef USE_ACC_SPI_MPU6000
#undef USE_ACC_SPI_MPU6500
#undef USE_GYRO_SPI_MPU6500
#undef USE_ACC_SPI_ICM20689
#undef USE_GYRO_SPI_ICM20689
#undef USE_ACCGYRO_LSM6DSO
#undef USE_GYRO_SPI_ICM42605
#undef USE_GYRO_SPI_ICM42688P
#undef USE_ACC_SPI_ICM42605
#undef USE_ACC_SPI_ICM42688P
#undef USE_ACCGYRO_LSM6DSV16X
#undef USE_ACC_MPU6050
#undef USE_GYRO_MPU6050
#undef USE_ACCGYRO_BMI160
#undef USE_GYRO_SPI_ICM20689
#undef USE_GYRO_SPI_MPU6500
#undef USE_GYRO_SPI_MPU9250
#undef USE_GYRO_L3GD20
#undef USE_ACCGYRO_BMI160
#undef USE_ACCGYRO_BMI270
#undef USE_ACCGYRO_LSM6DSO

#undef USE_RANGEFINDER_HCSR04
#undef USE_CRSF
#undef USE_TELEMETRY_CRSF
#undef USE_RX_EXPRESSLRS
#undef USE_MAX7456
#undef USE_MAG
#undef USE_MAG_HMC5883
#undef USE_MAG_SPI_HMC5883
#undef USE_VTX_RTC6705
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_RX_SX1280

#define GPIOA_BASE ((intptr_t)0x0001)

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

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

#define FLASH_PAGE_SIZE 0x1000
#define CONFIG_IN_RAM

#define U_ID_0 0
#define U_ID_1 1
#define U_ID_2 2
