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

#define FC_TARGET_MCU     STM32F7X2

#define BOARD_NAME        TMOTORF7
#define MANUFACTURER_ID   TMTR

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_BARO
#define USE_BARO_SPI_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16

#define USE_MAX7456

#define BEEPER_PIN           PC13
#define MOTOR1_PIN           PC6
#define MOTOR2_PIN           PC7
#define MOTOR3_PIN           PB0
#define MOTOR4_PIN           PB1
#define MOTOR5_PIN           PB6
#define MOTOR6_PIN           PB8
#define MOTOR7_PIN           PB7
#define MOTOR8_PIN           PC9
#define RX_PPM_PIN           PA15
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PC10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PC11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PB2
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define CAMERA_CONTROL_PIN   PB9
#define ADC_VBAT_PIN         PC1
#define ADC_RSSI_PIN         PC3
#define ADC_CURR_PIN         PC2
#define BARO_CS_PIN          PC15
#define FLASH_CS_PIN         PC8
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4
#define PINIO1_PIN           PC14

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC6 , 1,  0) \
    TIMER_PIN_MAP( 1, PC7 , 1,  0) \
    TIMER_PIN_MAP( 2, PB0 , 3,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  0) \
    TIMER_PIN_MAP( 4, PB6 , 1,  0) \
    TIMER_PIN_MAP( 5, PB8 , 1,  0) \
    TIMER_PIN_MAP( 6, PA15, 1,  0) \
    TIMER_PIN_MAP( 7, PB9 , 2, -1) \
    TIMER_PIN_MAP( 8, PA8 , 1,  0) \
    TIMER_PIN_MAP( 9, PB7 , 1,  0) \
    TIMER_PIN_MAP(10, PC9 , 2,  0)


#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE (I2CDEV_2)
#define BARO_SPI_INSTANCE SPI3
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
//TODO #define OSD_GPS_SATS_SHOW_HDOP ON
#define MAX7456_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI3
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define PINIO1_BOX 40
