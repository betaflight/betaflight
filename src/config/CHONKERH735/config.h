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

/*

Details: https://github.com/crteensy/yolo-chonker/tree/as-built-20230303

*/

#pragma once

#define FC_TARGET_MCU           STM32H723

#define BOARD_NAME              CHONKERH735
#define MANUFACTURER_ID         YOLO


#define USE_ACC
#define USE_ACC_SPI_ICM20602

#define USE_GYRO
#define USE_SPI_GYRO
#define USE_GYRO_SPI_ICM20602
#define USE_ACC_GYRO_BMI270

#define USE_BARO
#define USE_BARO_DPS310

#define USE_FLASH
#define USE_FLASH_W25Q128FV



// Resources

// #define ADC_VBAT_PIN
// #define ADC_RSSI_PIN
// #define ADC_EXTERNAL1_PIN

#define ADC_CURR_PIN                    PA5

// #define BEEPER_PIN

#define FLASH_CS_PIN                    PE4

#define GYRO_1_EXTI_PIN                 PD10
#define GYRO_2_EXTI_PIN                 PD11

#define GYRO_1_CS_PIN                   PA15
#define GYRO_2_CS_PIN                   PB12

#define I2C2_SCL_PIN                    PB10
#define I2C2_SDA_PIN                    PB11

#define LED0_PIN                        PA10
#define LED1_PIN                        PA8

#define MOTOR1_PIN                      PC6
#define MOTOR2_PIN                      PC9
#define MOTOR3_PIN                      PC8
#define MOTOR4_PIN                      PC7

#define SPI2_SCK_PIN                    PB13
#define SPI2_SDI_PIN                    PB14
#define SPI2_SDO_PIN                    PB15

#define SPI3_SCK_PIN                    PC10
#define SPI3_SDI_PIN                    PC11
#define SPI3_SDO_PIN                    PC12

#define SPI4_SCK_PIN                    PE2
#define SPI4_SDI_PIN                    PE5
#define SPI4_SDO_PIN                    PE6

// #define LED_STRIP_PIN

#define UART2_TX_PIN                    PD5
#define UART3_TX_PIN                    PD8
#define UART4_TX_PIN                    PA0
#define UART7_TX_PIN                    PE8
#define UART8_TX_PIN                    PE1
#define UART9_TX_PIN                    PD14

#define UART2_RX_PIN                    PD6
#define UART3_RX_PIN                    PD9
#define UART4_RX_PIN                    PA1
#define UART7_RX_PIN                    PE7
#define UART8_RX_PIN                    PE0
#define UART9_RX_PIN                    PD15

#define USB_DETECT_PIN                  PA9

//TODO Timer mapping

//TODO DMA options

#define BARO_I2C_INSTANCE               (I2CDEV_2)
#define FLASH_SPI_INSTANCE              SPI4
#define GYRO_1_SPI_INSTANCE             SPI2
#define GYRO_2_SPI_INSTANCE             SPI3

#define DEFAULT_BLACKBOX_DEVICE         BLACKBOX_DEVICE_FLASH
// #define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_1
// #define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC

#define DEFAULT_RX_FEATURE              FEATURE_RX_SERIAL
#define RX_SERIAL_PROTOCOL              CRSF

#define GYRO_1_ALIGN CW0_DEG_FLIP
#define GYRO_1_ALIGN_PITCH 1800
#define GYRO_2_ALIGN CW0_DEG_FLIP
#define GYRO_2_ALIGN_PITCH 1800

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
