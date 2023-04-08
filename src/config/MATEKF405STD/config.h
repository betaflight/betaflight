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

#define BOARD_NAME        MATEKF405STD
#define MANUFACTURER_ID   MTKS

#define USE_ACC
#define USE_ACC_SPI_ICM20602
#define USE_GYRO
#define USE_GYRO_SPI_ICM20602
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_SDCARD

#define BEEPER_PIN           PC13
#define MOTOR1_PIN           PC6
#define MOTOR2_PIN           PC7
#define MOTOR3_PIN           PC8
#define MOTOR4_PIN           PC9
#define MOTOR5_PIN           PA15
#define MOTOR6_PIN           PA8
#define MOTOR7_PIN           PB8
#define MOTOR8_PIN           PB0
#define RX_PPM_PIN           PA3
#define RX_PWM1_PIN          PA0
#define RX_PWM2_PIN          PA1
#define RX_PWM3_PIN          PA2
#define LED_STRIP_PIN        PB6
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
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PB9
#define LED1_PIN             PA14
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define ESCSERIAL_PIN        PA3
#define ADC_VBAT_PIN         PC5
#define ADC_RSSI_PIN         PB1
#define ADC_CURR_PIN         PC4
#define SDCARD_SPI_CS_PIN    PC1
#define MAX7456_SPI_CS_PIN   PB10
#define GYRO_1_EXTI_PIN      PC3
#define GYRO_1_CS_PIN        PC2
#define USB_DETECT_PIN       PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC6 , 1,  0) \
    TIMER_PIN_MAP( 1, PC7 , 2,  1) \
    TIMER_PIN_MAP( 2, PC8 , 2,  1) \
    TIMER_PIN_MAP( 3, PC9 , 2,  0) \
    TIMER_PIN_MAP( 4, PA15, 1,  0) \
    TIMER_PIN_MAP( 5, PA8 , 1,  0) \
    TIMER_PIN_MAP( 6, PB8 , 1,  0) \
    TIMER_PIN_MAP( 7, PB0 , 2,  0) \
    TIMER_PIN_MAP( 8, PB6 , 1,  0) \
    TIMER_PIN_MAP( 9, PA0 , 2,  0) \
    TIMER_PIN_MAP(10, PA1 , 2,  0) \
    TIMER_PIN_MAP(11, PA2 , 3, -1) \
    TIMER_PIN_MAP(12, PA3 , 2,  0)


#define SPI3_TX_DMA_OPT     1
#define ADC1_DMA_OPT        0


#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 179
#define BEEPER_INVERTED
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI3
#define SYSTEM_HSE_MHZ 8
#define MAX7456_SPI_INSTANCE SPI2
#define DASHBOARD_I2C_INSTANCE (I2CDEV_1)
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
