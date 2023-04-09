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

#define BOARD_NAME        COLIBRI
#define MANUFACTURER_ID   TEBS

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_BARO_MS5611
#define USE_FLASH
#define USE_FLASH_M25P16

#define BEEPER_PIN           PC5
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB4
#define MOTOR3_PIN           PB1
#define MOTOR4_PIN           PB15
#define MOTOR5_PIN           PB5
#define MOTOR6_PIN           PB14
#define MOTOR7_PIN           PB8
#define MOTOR8_PIN           PB9
#define RX_PPM_PIN           PA10
#define RX_PWM1_PIN          PA10
#define RX_PWM2_PIN          PC6
#define RX_PWM3_PIN          PC7
#define RX_PWM4_PIN          PC8
#define RX_PWM5_PIN          PA15
#define RX_PWM6_PIN          PB3
#define RX_PWM7_PIN          PA0
#define RX_PWM8_PIN          PA1
#define LED_STRIP_PIN        PB7
#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define INVERTER_PIN_UART2   PB2
#define I2C3_SCL_PIN         PA8
#define I2C3_SDA_PIN         PC9
#define LED0_PIN             PC14
#define LED1_PIN             PC13
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PC2
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PC3
#define ESCSERIAL_PIN        PA10
#define COMPASS_EXTI_PIN     PC1
#define FLASH_CS_PIN         PB12
#define GYRO_1_EXTI_PIN      PC0
#define GYRO_1_CS_PIN        PC4
#define USB_DETECT_PIN       PA9

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA10, 1,  0) \
    TIMER_PIN_MAP( 1, PC6 , 2,  0) \
    TIMER_PIN_MAP( 2, PC7 , 2,  0) \
    TIMER_PIN_MAP( 3, PC8 , 2,  0) \
    TIMER_PIN_MAP( 4, PA15, 1,  0) \
    TIMER_PIN_MAP( 5, PB3 , 1,  0) \
    TIMER_PIN_MAP( 6, PA0 , 2,  0) \
    TIMER_PIN_MAP( 7, PA1 , 2,  0) \
    TIMER_PIN_MAP( 8, PB0 , 2,  0) \
    TIMER_PIN_MAP( 9, PB4 , 1,  0) \
    TIMER_PIN_MAP(10, PB1 , 2,  0) \
    TIMER_PIN_MAP(11, PB15, 3, -1) \
    TIMER_PIN_MAP(12, PB5 , 1,  0) \
    TIMER_PIN_MAP(13, PB14, 3, -1) \
    TIMER_PIN_MAP(14, PB8 , 2, -1) \
    TIMER_PIN_MAP(15, PB9 , 2, -1) \
    TIMER_PIN_MAP(16, PB7 , 1,  0)



#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE (I2CDEV_3)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_3)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define SYSTEM_HSE_MHZ 16
#define DASHBOARD_I2C_INSTANCE (I2CDEV_3)
#define FLASH_SPI_INSTANCE SPI2
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG_FLIP
#define GYRO_1_ALIGN_PITCH 1800
#define GYRO_1_ALIGN_YAW 2700
