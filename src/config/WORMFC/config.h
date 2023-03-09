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

#define BOARD_NAME        WORMFC
#define MANUFACTURER_ID   FOSS

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_BARO_SPI_LPS
#define USE_MAX7456

#define BEEPER_PIN           PB14
#define MOTOR1_PIN           PB1
#define MOTOR2_PIN           PB0
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PA10
#define MOTOR6_PIN           PA8
#define RX_PPM_PIN           PB9
#define LED_STRIP_PIN        PA10
#define UART1_TX_PIN         PB6
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PB7
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART3   PB12
#define INVERTER_PIN_UART6   PB13
#define LED0_PIN             PA15
#define LED1_PIN             PC14
#define SPI1_SCK_PIN         PA5
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI3_SDO_PIN         PB5
#define ESCSERIAL_PIN        PB9
#define ADC_VBAT_PIN         PC1
#define ADC_CURR_PIN         PC2
#define BARO_CS_PIN          PB8
#define SDCARD_DETECT_PIN    PB15
#define MAX7456_SPI_CS_PIN   PC0
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PA9

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB9 , 1, -1) \
    TIMER_PIN_MAP( 1, PB1 , 2,  0) \
    TIMER_PIN_MAP( 2, PB0 , 2,  0) \
    TIMER_PIN_MAP( 3, PA2 , 1,  0) \
    TIMER_PIN_MAP( 4, PA3 , 1,  1) \
    TIMER_PIN_MAP( 5, PA10, 1,  1) \
    TIMER_PIN_MAP( 6, PA8 , 1,  1)


#define ADC1_DMA_OPT        1

#define USE_BARO
#define BARO_SPI_INSTANCE SPI3
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define MAX7456_SPI_INSTANCE SPI3
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG_FLIP
#define GYRO_1_ALIGN_PITCH 1800
#define GYRO_1_ALIGN_YAW 1800
