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

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        KAKUTEH7V2
#define MANUFACTURER_ID   HBRO

#define USE_GYRO
#define USE_ACC
#define USE_ACCGYRO_BMI270
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_MAX7456

#define BEEPER_PIN           PC13
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB3
#define MOTOR4_PIN           PB10
#define MOTOR5_PIN           PA0
#define MOTOR6_PIN           PA2
#define MOTOR7_PIN           PC8
#define MOTOR8_PIN           PC9
#define LED_STRIP_PIN        PD12
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PD1
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PD0
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PC2
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI4_SCK_PIN         PE2
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI4_SDI_PIN         PE5
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI4_SDO_PIN         PE6
#define CAMERA_CONTROL_PIN   PE9
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC1
#define FLASH_CS_PIN         PA4
#define PINIO1_PIN           PE13
#define PINIO2_PIN           PB11
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PE1
#define GYRO_1_CS_PIN        PE4
#define USB_DETECT_PIN       PA8

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 2,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  1) \
    TIMER_PIN_MAP( 2, PB3 , 1,  2) \
    TIMER_PIN_MAP( 3, PB10, 1,  3) \
    TIMER_PIN_MAP( 4, PA0 , 2,  4) \
    TIMER_PIN_MAP( 5, PA2 , 2,  5) \
    TIMER_PIN_MAP( 6, PC8 , 2,  6) \
    TIMER_PIN_MAP( 7, PC9 , 2,  7) \
    TIMER_PIN_MAP( 8, PD12, 1, 14) \
    TIMER_PIN_MAP( 9, PE9 , 1, 12)


#define SPI1_TX_DMA_OPT    13
#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9
#define TIMUP1_DMA_OPT      0
#define TIMUP2_DMA_OPT      0
#define TIMUP3_DMA_OPT      2
#define TIMUP4_DMA_OPT      0
#define TIMUP5_DMA_OPT      0
#define TIMUP8_DMA_OPT      1

#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
//TODO #define SDCARD_MODE OFF
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 109
#define DEFAULT_CURRENT_METER_SCALE 168
#define BEEPER_INVERTED
#define FLASH_SPI_INSTANCE SPI1
#define MAX7456_SPI_INSTANCE SPI2
#define DASHBOARD_I2C_INSTANCE (I2CDEV_1)
#define PINIO1_CONFIG 129
#define PINIO2_CONFIG 129
#define PINIO1_BOX 0
#define PINIO2_BOX 40
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI4
