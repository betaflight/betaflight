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

#define FC_TARGET_MCU     STM32F405

#define BOARD_NAME        MATEKF405TEMINI
#define MANUFACTURER_ID   MTKS

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_FLASH_M25P16

#define BEEPER_PIN           PB9
#define MOTOR1_PIN           PC9
#define MOTOR2_PIN           PC8
#define MOTOR3_PIN           PB15
#define MOTOR4_PIN           PA8
#define MOTOR5_PIN           PB11
#define MOTOR6_PIN           PB10
#define MOTOR7_PIN           PB3
#define MOTOR8_PIN           PA15
#define SERVO1_PIN           PB14
#define SERVO2_PIN           PA6
#define SERVO3_PIN           PB6
#define RX_PPM_PIN           PA3
#define LED_STRIP_PIN        PB1
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PC10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PC11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PA14
#define LED1_PIN             PA13
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         NONE
#define SPI1_SDI_PIN         PB4
#define SPI2_SDI_PIN         PC2
#define SPI3_SDI_PIN         NONE
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PC3
#define SPI3_SDO_PIN         NONE
#define ADC_VBAT_PIN         PC4
#define ADC_RSSI_PIN         PB0
#define ADC_CURR_PIN         PC5
#define ADC_EXTERNAL1_PIN    PC0
#define SDCARD_CS_PIN        PC1
#define PINIO1_PIN           PA4
#define PINIO2_PIN           PB5
#define FLASH_CS_PIN         PC13
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PC15
#define GYRO_1_CS_PIN        PC14

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, MOTOR1_PIN, 2,  0) \
    TIMER_PIN_MAP( 1, MOTOR2_PIN, 2,  0) \
    TIMER_PIN_MAP( 2, MOTOR3_PIN, 1,  1) \
    TIMER_PIN_MAP( 3, MOTOR4_PIN, 1,  1) \
    TIMER_PIN_MAP( 4, MOTOR5_PIN, 1,  0) \
    TIMER_PIN_MAP( 5, MOTOR6_PIN, 1,  0) \
    TIMER_PIN_MAP( 6, MOTOR7_PIN, 1,  0) \
    TIMER_PIN_MAP( 7, MOTOR8_PIN, 1,  0) \
    TIMER_PIN_MAP( 8, SERVO1_PIN, 3, -1) \
    TIMER_PIN_MAP( 9, SERVO2_PIN, 2, -1) \
    TIMER_PIN_MAP(10, SERVO3_PIN, 1,  0) \
    TIMER_PIN_MAP(11, LED_STRIP_PIN, 2,  0) \
    TIMER_PIN_MAP(12, BEEPER_PIN, 2, -1) \
    TIMER_PIN_MAP(13, RX_PPM_PIN, 3, -1) \
    TIMER_PIN_MAP(14, UART2_TX_PIN, 2,  0)


#define ADC1_DMA_OPT                    1

//TODO #define MAG_BUSTYPE I2C
#define MAG_I2C_INSTANCE                (I2CDEV_1)
//TODO #define MAG_HARDWARE AUTO
#define USE_BARO
#define BARO_I2C_INSTANCE               (I2CDEV_1)
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE     210
#define DEFAULT_CURRENT_METER_SCALE     150
#define BEEPER_INVERTED
#define PINIO1_BOX 40
#define PINIO2_BOX 41
#define SYSTEM_HSE_MHZ                  8
#define DEFAULT_BLACKBOX_DEVICE         BLACKBOX_DEVICE_FLASH
#define MAX7456_SPI_INSTANCE            SPI1
#define FLASH_SPI_INSTANCE              SPI2

#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE             SPI1
#define GYRO_1_ALIGN                    CW270_DEG_FLIP
#define GYRO_1_ALIGN_PITCH              1800
#define GYRO_1_ALIGN_YAW                2700

#define DEFAULT_RX_FEATURE              FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER               SERIALRX_CRSF
