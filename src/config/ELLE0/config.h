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

#define BOARD_NAME        ELLE0
#define MANUFACTURER_ID   LEGA

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500

#define MOTOR1_PIN           PC6
#define MOTOR2_PIN           PC7
#define MOTOR3_PIN           PC8
#define MOTOR4_PIN           PC9
#define MOTOR5_PIN           PA0
#define MOTOR6_PIN           PA1
#define MOTOR7_PIN           PB8
#define MOTOR8_PIN           PB9
#define RX_PPM_PIN           PA2
#define RX_PWM1_PIN          PA2
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define LED0_PIN             PA8
#define LED1_PIN             PB4
#define LED2_PIN             PC2
#define SPI2_SCK_PIN         PB13
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PB15
#define ADC_VBAT_PIN         PC4
#define ADC_CURR_PIN         PC5
#define GYRO_1_EXTI_PIN      PB5
#define GYRO_1_CS_PIN        PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA2 , 1,  0) \
    TIMER_PIN_MAP( 1, PC6 , 2,  0) \
    TIMER_PIN_MAP( 2, PC7 , 2,  0) \
    TIMER_PIN_MAP( 3, PC8 , 2,  1) \
    TIMER_PIN_MAP( 4, PC9 , 2,  0) \
    TIMER_PIN_MAP( 5, PA0 , 2,  0) \
    TIMER_PIN_MAP( 6, PA1 , 2,  0) \
    TIMER_PIN_MAP( 7, PB8 , 1,  0) \
    TIMER_PIN_MAP( 8, PB9 , 1, -1)



#define ADC1_DMA_OPT        1

#define SYSTEM_HSE_MHZ 25
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI2
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
