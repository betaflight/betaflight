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

#define BOARD_NAME        SPRACINGF4NEO
#define MANUFACTURER_ID   SPRO

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_MAX7456
#define USE_SDCARD

#define BEEPER_PIN           PC15
#define MOTOR1_PIN           PC8
#define MOTOR2_PIN           PC7
#define MOTOR3_PIN           PC9
#define MOTOR4_PIN           PC6
#define MOTOR5_PIN           PB6
#define MOTOR6_PIN           PB7
#define MOTOR7_PIN           PB10
#define MOTOR8_PIN           PB11
#define SERVO1_PIN           PA9
#define SERVO2_PIN           PA10
#define RX_PPM_PIN           PA3
#define RX_PWM1_PIN          PA3
#define RX_PWM2_PIN          PA2
#define RX_PWM3_PIN          PA9
#define RX_PWM4_PIN          PA10
#define LED_STRIP_PIN        PA1
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PC10
#define UART5_TX_PIN         PC12
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PC11
#define UART5_RX_PIN         PD2
#define INVERTER_PIN_UART2   PB2
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define LED0_PIN             PA0
#define LED1_PIN             PB1
#define TRANSPONDER_PIN      PA8
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
#define ADC_VBAT_PIN         PC1
#define ADC_RSSI_PIN         PC0
#define ADC_CURR_PIN         PC2
#define SDCARD_SPI_CS_PIN    PB12
#define SDCARD_DETECT_PIN    PC14
#define MAX7456_SPI_CS_PIN   PA15
#define GYRO_1_EXTI_PIN      PC13
#define GYRO_1_CS_PIN        PA4
#define VTX_POWER_PIN        PC3
#define VTX_CS_PIN           PC4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA3 , 3, -1) \
    TIMER_PIN_MAP( 1, PA2 , 3, -1) \
    TIMER_PIN_MAP( 2, PC8 , 2,  1) \
    TIMER_PIN_MAP( 3, PC7 , 2,  1) \
    TIMER_PIN_MAP( 4, PC9 , 2,  0) \
    TIMER_PIN_MAP( 5, PC6 , 2,  1) \
    TIMER_PIN_MAP( 6, PB6 , 1,  0) \
    TIMER_PIN_MAP( 7, PB7 , 1,  0) \
    TIMER_PIN_MAP( 8, PA1 , 1,  0) \
    TIMER_PIN_MAP( 9, PB10, 1,  0) \
    TIMER_PIN_MAP(10, PB11, 1,  1) \
    TIMER_PIN_MAP(11, PA8 , 1,  0) \
    TIMER_PIN_MAP(12, PA9 , 1,  0) \
    TIMER_PIN_MAP(13, PA10, 1,  0)


#define SPI2_TX_DMA_OPT     0
#define ADC1_DMA_OPT        0

#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 300
#define BEEPER_INVERTED
#define SDCARD_DETECT_INVERTED
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI2
#define SYSTEM_HSE_MHZ 8
//TODO #define VTX_SPI_BUS 3
#define MAX7456_SPI_INSTANCE SPI3
#define DASHBOARD_I2C_INSTANCE (I2CDEV_1)
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
