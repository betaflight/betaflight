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

#define FC_TARGET_MCU                       AT32F435M

#define BOARD_NAME                          AIRBOTF435
#define MANUFACTURER_ID                     AIRB

#define USE_ACC
#define USE_ACC_SPI_ICM42688P

#define USE_BARO
#define USE_BARO_DPS310

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_SPI_GYRO

#define USE_FLASH
#define USE_FLASH_M25P16

#define USE_MAX7456
#define MAX7456_SPI_CS_PIN                  PH2
#define MAX7456_SPI_INSTANCE                SPI3

#define BEEPER_PIN                          PB10
#define MOTOR1_PIN                          PC9
#define MOTOR2_PIN                          PC8
#define MOTOR3_PIN                          PB7
#define MOTOR4_PIN                          PB6
#define MOTOR5_PIN                          PB0
#define MOTOR6_PIN                          PB1
#define MOTOR7_PIN                          PC7
#define MOTOR8_PIN                          PC6

#define LED_STRIP_PIN                       PA15

#define UART1_TX_PIN                        PA9
#define UART2_TX_PIN                        PA2
#define UART3_TX_PIN                        PC10
#define UART4_TX_PIN                        PA0
#define UART5_TX_PIN                        PC12

#define UART7_TX_PIN                        PC0
#define UART8_TX_PIN                        PC2

#define UART1_RX_PIN                        PA10

#define UART3_RX_PIN                        PC11
#define UART4_RX_PIN                        PA1
#define UART5_RX_PIN                        PD2

#define UART7_RX_PIN                        PC1
#define UART8_RX_PIN                        PC3

#define I2C1_SCL_PIN                        PB8
#define I2C1_SDA_PIN                        PB9

#define LED0_PIN                            PC15

#define SPI1_SCK_PIN                        PA5
#define SPI2_SCK_PIN                        PB13
#define SPI3_SCK_PIN                        PB3

#define SPI1_SDI_PIN                        PA6
#define SPI2_SDI_PIN                        PB14
#define SPI3_SDI_PIN                        PB4

#define SPI1_SDO_PIN                        PA7
#define SPI2_SDO_PIN                        PB15
#define SPI3_SDO_PIN                        PB5

#define ADC_VBAT_PIN                        PC4
#define ADC_CURR_PIN                        PA3

#define PINIO1_PIN                          PC11
//TODO #define PINIO2_PIN                          PH3

#define FLASH_CS_PIN                        PC13
#define FLASH_SPI_INSTANCE                  SPI3

#define GYRO_1_EXTI_PIN                     PB12
#define GYRO_1_CS_PIN                       PC14
#define GYRO_1_SPI_INSTANCE                 SPI2

#define TIMER_PIN_MAPPING                   TIMER_PIN_MAP( 0, LED_STRIP_PIN, 1, -1 ) \
                                            TIMER_PIN_MAP( 1, MOTOR4_PIN, 1,  2 ) \
                                            TIMER_PIN_MAP( 2, MOTOR3_PIN, 1,  4 ) \
                                            TIMER_PIN_MAP( 3, MOTOR2_PIN, 2,  1 ) \
                                            TIMER_PIN_MAP( 4, MOTOR1_PIN, 2,  3 ) \
                                            TIMER_PIN_MAP( 5, MOTOR5_PIN, 3,  7 ) \
                                            TIMER_PIN_MAP( 6, MOTOR6_PIN, 3,  8 ) \
                                            TIMER_PIN_MAP( 7, MOTOR8_PIN, 2,  9 ) \
                                            TIMER_PIN_MAP( 8, MOTOR7_PIN, 2, 10 )



#define ADC1_DMA_OPT                        0
#define SPI3_TX_DMA_OPT                     5

#define ADC_INSTANCE                        ADC1
#define BARO_I2C_INSTANCE                   (I2CDEV_1)

#define DEFAULT_BLACKBOX_DEVICE             BLACKBOX_DEVICE_FLASH

#define DEFAULT_CURRENT_METER_SOURCE        CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE        VOLTAGE_METER_ADC
