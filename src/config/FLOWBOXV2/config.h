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

#define BOARD_NAME        FLOWBOXV2
#define MANUFACTURER_ID   NERC

#define BEEPER_PIN           PB8
#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PB0
#define MOTOR3_PIN           PB1
#define MOTOR4_PIN           PB10
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define LED0_PIN             PC13
#define SPI1_SCK_PIN         PA5
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI3_SDO_PIN         PB5
#define SDCARD_SPI_CS_PIN    PA15
#define GYRO_1_EXTI_PIN      PA1
#define GYRO_1_CS_PIN        PA4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB8 , 1,  0) \
    TIMER_PIN_MAP( 1, PA0 , 1,  0) \
    TIMER_PIN_MAP( 2, PB0 , 2,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  0) \
    TIMER_PIN_MAP( 4, PB10, 1,  0)



#define SPI3_TX_DMA_OPT     0
#define ADC1_DMA_OPT        1

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define BEEPER_PWM_HZ 2185
#define BEEPER_INVERTED
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI3
#define SYSTEM_HSE_MHZ 8
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
#define GYRO_1_ALIGN_YAW 1800
