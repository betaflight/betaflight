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

#define BOARD_NAME        VRRACE
#define MANUFACTURER_ID   LEGA

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_MAX7456

#define BEEPER_PIN           PA0
#define MOTOR1_PIN           PA1
#define MOTOR2_PIN           PA2
#define MOTOR3_PIN           PA3
#define MOTOR4_PIN           PB5
#define MOTOR5_PIN           PB0
#define MOTOR6_PIN           PB1
#define RX_PPM_PIN           PE9
#define RX_PWM1_PIN          PE9
#define RX_PWM2_PIN          PE11
#define RX_PWM3_PIN          PE13
#define RX_PWM4_PIN          PE14
#define RX_PWM5_PIN          PE6
#define RX_PWM6_PIN          PE7
#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART6_TX_PIN         PC6
#define SOFTSERIAL1_TX_PIN   PE11
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART6_RX_PIN         PC7
#define SOFTSERIAL1_RX_PIN   PE13
#define INVERTER_PIN_UART6   PD7
#define LED0_PIN             PD14
#define LED1_PIN             PD15
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define ESCSERIAL_PIN        PE9
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PB1
#define ADC_CURR_PIN         PA5
#define GYRO_1_EXTI_PIN      PD10
#define GYRO_1_CS_PIN        PE10

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PE9 , 1,  0) \
    TIMER_PIN_MAP( 1, PE11, 1,  0) \
    TIMER_PIN_MAP( 2, PE13, 1,  0) \
    TIMER_PIN_MAP( 3, PE14, 1,  0) \
    TIMER_PIN_MAP( 4, PA1 , 1,  0) \
    TIMER_PIN_MAP( 5, PA2 , 1,  0) \
    TIMER_PIN_MAP( 6, PA3 , 1,  0) \
    TIMER_PIN_MAP( 7, PB5 , 1,  0) \
    TIMER_PIN_MAP( 8, PB0 , 2,  0) \
    TIMER_PIN_MAP( 9, PB1 , 2,  0)


#define ADC1_DMA_OPT        1

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI2
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
