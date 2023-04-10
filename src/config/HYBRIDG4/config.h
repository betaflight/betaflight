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

#define FC_TARGET_MCU     STM32G47X

#define BOARD_NAME        HYBRIDG4
#define MANUFACTURER_ID   NERC

#define BEEPER_PIN           PC14
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB2
#define MOTOR4_PIN           PB3
#define RX_PPM_PIN           PA2
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define I2C1_SCL_PIN         PA15
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PC15
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define ESCSERIAL_PIN        PC6
#define CAMERA_CONTROL_PIN   PB6
#define ADC_VBAT_PIN         PA0
#define ADC_CURR_PIN         PA1
#define FLASH_CS_PIN         PB9
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 1,  0) \
    TIMER_PIN_MAP( 1, PB1 , 1,  1) \
    TIMER_PIN_MAP( 2, PB2 , 1,  2) \
    TIMER_PIN_MAP( 3, PB3 , 1,  3) \
    TIMER_PIN_MAP( 4, PA8 , 1,  8) \
    TIMER_PIN_MAP( 5, PB6 , 2,  0) \
    TIMER_PIN_MAP( 6, PA2 , 3,  0)


#define ADC1_DMA_OPT       12
#define ADC4_DMA_OPT        0
#define ADC5_DMA_OPT        0
#define TIMUP3_DMA_OPT      0
#define TIMUP4_DMA_OPT      0
#define TIMUP5_DMA_OPT      0

#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define MAX7456_SPI_INSTANCE SPI2
//TODO #define LED_INVERSION 1
#define FLASH_SPI_INSTANCE SPI3
#define DEFAULT_PID_PROCESS_DENOM 2
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN_YAW 2700
