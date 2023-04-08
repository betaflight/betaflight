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

#define BOARD_NAME        TRANSTECF405HD
#define MANUFACTURER_ID   TTRH

#define BEEPER_PIN           PB4
#define MOTOR1_PIN           PC6
#define MOTOR2_PIN           PC7
#define MOTOR3_PIN           PC8
#define MOTOR4_PIN           PC9
#define LED_STRIP_PIN        PB6
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define INVERTER_PIN_UART1   PB8
#define LED0_PIN             PB9
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7
#define ADC_VBAT_PIN         PC5
#define ADC_RSSI_PIN         PB1
#define ADC_CURR_PIN         PC4
#define GYRO_1_EXTI_PIN      PC3
#define GYRO_1_CS_PIN        PC2
#define USB_DETECT_PIN       PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC6 , 1,  0) \
    TIMER_PIN_MAP( 1, PC7 , 1,  0) \
    TIMER_PIN_MAP( 2, PC8 , 2,  0) \
    TIMER_PIN_MAP( 3, PC9 , 2,  0) \
    TIMER_PIN_MAP( 4, PB6 , 1,  0)


#define ADC1_DMA_OPT        1

//TODO #define MAG_HARDWARE NONE
#define DEFAULT_BARO_DEVICE BARO_NONE

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
//TODO #define MOTOR_PWM_PROTOCOL DSHOT600
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 179
#define BEEPER_INVERTED
//TODO #define YAW_MOTORS_REVERSED ON
//TODO #define OSD_CURRENT_POS 2486
#define SYSTEM_HSE_MHZ 8
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define MAX7456_SPI_INSTANCE SPI0
