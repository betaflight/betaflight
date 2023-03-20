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

#define FC_TARGET_MCU     AT32F435G

#define BOARD_NAME        NEUTRONRCF435
#define MANUFACTURER_ID   NERC

#define LED0_PIN             PC4

#define USE_GYRO
#define USE_ACC
#define USE_ACCGYRO_BMI270
#define GYRO_1_CS_PIN        PA15
#define GYRO_1_SPI_INSTANCE  SPI1
#define GYRO_1_ALIGN         CW270_DEG

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN      PB8

#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5

#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define FLASH_CS_PIN         PB9
#define FLASH_SPI_INSTANCE   SPI3

#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11

#define BARO_I2C_INSTANCE    I2CDEV_2
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE SPI2
#define MAX7456_SPI_CS_PIN   PB12

#define ADC1_DMA_OPT                    1

