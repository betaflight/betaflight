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

#define BOARD_NAME        NUCLEOF722
#define MANUFACTURER_ID   STMI

#define USE_GYRO
#define USE_GYRO_MPU6050
#define USE_ACC
#define USE_ACC_MPU6050

#define MOTOR1_PIN           PB8
#define MOTOR2_PIN           PA3
#define MOTOR3_PIN           PB5
#define MOTOR4_PIN           PB9
#define MOTOR5_PIN           PE6
#define MOTOR6_PIN           PB4
#define RX_PPM_PIN           PB15
#define RX_PWM1_PIN          PB15
#define RX_PWM2_PIN          PC6
#define RX_PWM3_PIN          PC7
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PA0
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PA1
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define LED0_PIN             PB7
#define LED1_PIN             PB14
#define SPI1_SCK_PIN         PA5
#define SPI4_SCK_PIN         PE12
#define SPI1_SDI_PIN         PA6
#define SPI4_SDI_PIN         PE13
#define SPI1_SDO_PIN         PA7
#define SPI4_SDO_PIN         PE14
#define ADC_VBAT_PIN         PA3
#define ADC_RSSI_PIN         PC3
#define ADC_CURR_PIN         PC0
#define GYRO_1_EXTI_PIN      PB15
#define USB_DETECT_PIN       PA9

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB15, 3, -1) \
    TIMER_PIN_MAP( 1, PC6 , 2,  0) \
    TIMER_PIN_MAP( 2, PC7 , 2,  1) \
    TIMER_PIN_MAP( 3, PB8 , 1,  0) \
    TIMER_PIN_MAP( 4, PA3 , 1,  1) \
    TIMER_PIN_MAP( 5, PB5 , 1,  0) \
    TIMER_PIN_MAP( 6, PB9 , 1, -1) \
    TIMER_PIN_MAP( 7, PE6 , 1, -1) \
    TIMER_PIN_MAP( 8, PB4 , 1,  0)


#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
