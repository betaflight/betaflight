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

#define FC_TARGET_MCU     AT32F435M

// REVO with STM32F405 swapped for an AT32F435

#define BOARD_NAME        REVO_AT
#define MANUFACTURER_ID   OPEN

#define LED0_PIN             PB5
#define LED1_PIN             PB4

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_ALIGN            CW270_DEG

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4

#define SPI1_SCK_PIN         PA5
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI3_SDO_PIN         PC12

#define USE_FLASH
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3

#define I2C1_SCL_PIN            PB8
#define I2C1_SDA_PIN            PB9

#define I2C2_SCL_PIN            PB10
#define I2C2_SDA_PIN            PB11

#define MAG_I2C_INSTANCE        I2CDEV_1
#define USE_MAG
#define USE_MAG_HMC5883

#define BARO_I2C_INSTANCE       I2CDEV_1
#define USE_BARO
#define USE_BARO_MS5611

#define ADC1_DMA_OPT                    1
