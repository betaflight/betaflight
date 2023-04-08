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

#define FC_TARGET_MCU     STM32F411

#define BOARD_NAME        REVONANO
#define MANUFACTURER_ID   OPEN

#define USE_GYRO
#define USE_GYRO_SPI_MPU9250
#define USE_ACC
#define USE_ACC_SPI_MPU9250

#define BEEPER_PIN           PC13
#define MOTOR1_PIN           PA10
#define MOTOR2_PIN           PB3
#define MOTOR3_PIN           PB8
#define MOTOR4_PIN           PB9
#define MOTOR5_PIN           PA0
#define MOTOR6_PIN           PA1
#define RX_PPM_PIN           PB10
#define RX_PWM1_PIN          PB10
#define RX_PWM2_PIN          PB1
#define RX_PWM3_PIN          PB0
#define RX_PWM4_PIN          PA7
#define RX_PWM5_PIN          PA6
#define RX_PWM6_PIN          PA5
#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PA2
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PA3
#define INVERTER_PIN_UART2   PC15
#define I2C3_SCL_PIN         PA8
#define I2C3_SDA_PIN         PB4
#define LED0_PIN             PC14
#define LED1_PIN             PC13
#define SPI2_SCK_PIN         PB13
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PB15
#define ESCSERIAL_PIN        PB10
#define ADC_VBAT_PIN         PA6
#define ADC_RSSI_PIN         PA5
#define ADC_CURR_PIN         PA7
#define PINIO1_PIN           PB10
#define GYRO_1_EXTI_PIN      PA15
#define GYRO_1_CS_PIN        PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB10, 1,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  0) \
    TIMER_PIN_MAP( 2, PB0 , 2,  0) \
    TIMER_PIN_MAP( 3, PA7 , 2,  0) \
    TIMER_PIN_MAP( 4, PA6 , 1,  0) \
    TIMER_PIN_MAP( 5, PA5 , 1,  0) \
    TIMER_PIN_MAP( 6, PA10, 1,  0) \
    TIMER_PIN_MAP( 7, PB3 , 1,  0) \
    TIMER_PIN_MAP( 8, PB8 , 1,  0) \
    TIMER_PIN_MAP( 9, PB9 , 1, -1) \
    TIMER_PIN_MAP(10, PA0 , 2,  0) \
    TIMER_PIN_MAP(11, PA1 , 2,  0)


#define ADC1_DMA_OPT        1

#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_3)
#define SYSTEM_HSE_MHZ 8
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI2
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
