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

#define FC_TARGET_MCU                   STM32F411

#define BOARD_NAME                      SUB250F411
#define MANUFACTURER_ID                 SU25

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_ACCGYRO_BMI270
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_DPS310
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BEEPER_PIN                      B02
#define MOTOR1_PIN                      B04
#define MOTOR2_PIN                      B05
#define MOTOR3_PIN                      B06
#define MOTOR4_PIN                      B07
#define RX_PPM_PIN                      A03
#define LED_STRIP_PIN                   A08
#define UART1_TX_PIN                    A09
#define UART2_TX_PIN                    A02
#define UART1_RX_PIN                    A10
#define UART2_RX_PIN                    A03
#define INVERTER_PIN_UART2              C15
#define I2C1_SCL_PIN                    B08
#define I2C1_SDA_PIN                    B09
#define LED0_PIN                        C13
#define LED1_PIN                        C14
#define SPI1_SCK_PIN                    A05
#define SPI2_SCK_PIN                    B13
#define SPI1_SDI_PIN                    A06
#define SPI2_SDI_PIN                    B14
#define SPI1_SDO_PIN                    A07
#define SPI2_SDO_PIN                    B15
#define ADC_VBAT_PIN                    B00
#define ADC_CURR_PIN                    B01
#define PINIO1_PIN                      B10
#define FLASH_CS_PIN                    A00
#define MAX7456_SPI_CS_PIN              B12
#define GYRO_1_EXTI_PIN                 A01
#define GYRO_1_CS_PIN                   A04

#define TIMER_PIN_MAPPING               TIMER_PIN_MAP( 0, PA3, 3, -1) \
                                        TIMER_PIN_MAP( 1, PB4, 1,  0) \
                                        TIMER_PIN_MAP( 2, PB5, 1,  0) \
                                        TIMER_PIN_MAP( 3, PB6, 1,  0) \
                                        TIMER_PIN_MAP( 4, PB7, 1,  0) \
                                        TIMER_PIN_MAP( 4, PA8, 1,  0)


#define ADC1_DMA_OPT                    0

#define SERIALRX_UART                   SERIAL_PORT_USART2

#define BARO_I2C_INSTANCE               (I2CDEV_1)
#define SERIALRX_PROVIDER               CRSF
#define DEFAULT_BLACKBOX_DEVICE         BLACKBOX_DEVICE_FLASH
//TODO #define DSHOT_IDLE_VALUE 800
#define DEFAULT_DSHOT_BURST             DSHOT_DMAR_AUTO
#define DEFAULT_DSHOT_BITBANG           DSHOT_BITBANG_OFF
//TODO #define MOTOR_PWM_PROTOCOL DSHOT300
#define ALIGN_BOARD_YAW                 45
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE     500

#define BEEPER_INVERTED
#define DEFAULT_PID_PROCESS_DENOM 1
#define SYSTEM_HSE_MHZ                  8
#define MAX7456_SPI_INSTANCE            SPI2

#define PINIO1_BOX                      40
#define FLASH_SPI_INSTANCE              SPI2
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE             SPI1
#define GYRO_1_ALIGN                    CW270_DEG
#define GYRO_1_ALIGN_YAW                2700
