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

#define BOARD_NAME        TRANSTECF411HD
#define MANUFACTURER_ID   TTRH

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000

#define BEEPER_PIN           PB6
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB10
#define MOTOR4_PIN           PB9
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define INVERTER_PIN_UART1   PC13
#define LED0_PIN             PA14
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7
#define ADC_VBAT_PIN         PA0
#define ADC_CURR_PIN         PB4
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PA1
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PC15

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 2,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  0) \
    TIMER_PIN_MAP( 2, PB10, 1,  0) \
    TIMER_PIN_MAP( 3, PB9 , 1, -1) \
    TIMER_PIN_MAP( 4, PA8 , 1,  0)


#define ADC1_DMA_OPT        1

//TODO #define MAG_HARDWARE NONE
#define DEFAULT_BARO_DEVICE BARO_NONE

//TODO #define DSHOT_IDLE_VALUE 600
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
//TODO #define MOTOR_PWM_PROTOCOL DSHOT300
//TODO #define MOTOR_POLES 12
#define DEFAULT_ALIGN_BOARD_ROLL 180
//TODO #define VBAT_MAX_CELL_VOLTAGE 435
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_NONE
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
//TODO #define YAW_MOTORS_REVERSED ON
//TODO #define SMALL_ANGLE 90
//TODO #define PID_PROCESS_DENOM 1
#define SYSTEM_HSE_MHZ 8
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW90_DEG
#define GYRO_1_ALIGN_YAW 900
#define MAX7456_SPI_INSTANCE SPI0
//TODO #define SBUS_BAUD_FAST OFF
