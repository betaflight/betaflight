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

#define BOARD_NAME        ALIENFLIGHTF4
#define MANUFACTURER_ID   AFNG

#define USE_GYRO
#define USE_GYRO_SPI_MPU9250
#define USE_ACC
#define USE_ACC_SPI_MPU9250
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_MAG_SPI_AK8963
#define USE_MAX7456
#define USE_SDCARD

#define BEEPER_PIN           PC13
#define MOTOR1_PIN           PB8
#define MOTOR2_PIN           PB9
#define MOTOR3_PIN           PA0
#define MOTOR4_PIN           PA1
#define MOTOR5_PIN           PC6
#define MOTOR6_PIN           PC7
#define MOTOR7_PIN           PC8
#define MOTOR8_PIN           PC9
#define RX_PPM_PIN           PA8
#define RX_PWM1_PIN          PA8
#define RX_PWM2_PIN          PB0
#define RX_PWM3_PIN          PB1
#define RX_PWM4_PIN          PB14
#define RX_PWM5_PIN          PB15
#define LED_STRIP_PIN        PB15
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART4_TX_PIN         PC10
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART4_RX_PIN         PC11
#define INVERTER_PIN_UART2   PC15
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PC12
#define LED1_PIN             PD2
#define SPEKTRUM_RX_BIND_PIN PB2
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PC2
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PC3
#define SPI3_SDO_PIN         PB5
#define ESCSERIAL_PIN        PA8
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PC4
#define ADC_CURR_PIN         PC1
#define ADC_EXTERNAL1_PIN    PC5
#define SDCARD_CS_PIN        PB10
#define SDCARD_DETECT_PIN    PB11
#define FLASH_CS_PIN         PB12
#define GYRO_1_EXTI_PIN      PC14
#define GYRO_1_CS_PIN        PA4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA8 , 1,  1) \
    TIMER_PIN_MAP( 1, PB0 , 2,  0) \
    TIMER_PIN_MAP( 2, PB1 , 2,  0) \
    TIMER_PIN_MAP( 3, PB14, 1,  1) \
    TIMER_PIN_MAP( 4, PB15, 1,  0) \
    TIMER_PIN_MAP( 5, PB8 , 1,  0) \
    TIMER_PIN_MAP( 6, PB9 , 1, -1) \
    TIMER_PIN_MAP( 7, PA0 , 2,  0) \
    TIMER_PIN_MAP( 8, PA1 , 2,  0) \
    TIMER_PIN_MAP( 9, PC6 , 2,  0) \
    TIMER_PIN_MAP(10, PC7 , 2,  0) \
    TIMER_PIN_MAP(11, PC8 , 2,  1) \
    TIMER_PIN_MAP(12, PC9 , 2,  0)



#define SPI2_TX_DMA_OPT     0
#define ADC1_DMA_OPT        0

#define MAG_ALIGN CW180_DEG_FLIP
#define MAG_ALIGN_PITCH 1800
#define MAG_ALIGN_YAW 1800
//TODO #define MAG_BUSTYPE I2C
#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
//TODO #define SERIALRX_PROVIDER SPEK2048
//TODO #define SPEKTRUM_SAT_BIND 9
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
//TODO #define SDCARD_DETECT_INVERTED ON
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI2
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
//TODO #define I2C1_PULLUP ON
