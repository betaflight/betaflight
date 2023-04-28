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

#define BOARD_NAME        AIRBOTG4AIO
#define MANUFACTURER_ID   AIR

#define USE_ACC
#define USE_GYRO
#define USE_FLASH

#define USE_FLASH_W25Q128FV

#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_SPI_GYRO

#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO_DPS310
#define USE_BARO_BPM280
#define USE_BARO_BPM085
#define USE_BARO_MS5611
#define USE_BARO_QMP6988
#define USE_BARO_LPS

/*
resource GYRO_EXTI 1 C04
resource GYRO_CS 1 B00
*/
#define GYRO_1_SPI_INSTANCE  SPI1
#define GYRO_1_ALIGN         CW0_DEG_FLIP
#define GYRO_1_ALIGN_YAW     2700
#define ALIGN_BOARD_YAW      -45
#define GYRO_1_CS_PIN        PB0
#define GYRO_1_EXTI_PIN      PC4

#define BEEPER_PIN           PB7
#define CAMERA_CONTROL_PIN   PA15


#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3

#define LED_STRIP_PIN        PB6

#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PB3
#define UART4_TX_PIN         PC10
#define LPUART1_TX_PIN       PB11


#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PB4
#define UART3_RX_PIN         PB9
#define UART4_RX_PIN         PC11
#define LPUART1_RX_PIN       PB10

#define I2C1_SCL_PIN         PA13
#define I2C1_SDA_PIN         PA14

#define LED0_PIN             PB5
#define LED1_PIN             PC15

#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13

#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14

#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15

#define ADC_VBAT_PIN         PB2
#define ADC_CURR_PIN         PB1

#define FLASH_CS_PIN         PC6
#define MAX7456_SPI_CS_PIN   PA8

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, LED_STRIP_PIN , 3,  12) \
    TIMER_PIN_MAP( 1, MOTOR1_PIN , 1,  1) \
    TIMER_PIN_MAP( 2, MOTOR2_PIN , 1,  2) \
    TIMER_PIN_MAP( 3, MOTOR3_PIN , 2,  3) \
    TIMER_PIN_MAP( 4, MOTOR4_PIN , 2,  4)

#define ADC1_DMA_OPT       10
#define ADC2_DMA_OPT       11

#define SPI1_RX_DMA_OPT     6
#define SPI1_TX_DMA_OPT     7

#define SPI2_RX_DMA_OPT     8
#define SPI2_TX_DMA_OPT     9
//#define TIMUP2_DMA_OPT      0
//#define TIMUP8_DMA_OPT      0

#define ADC_INSTANCE                 ADC2
#define DEFAULT_BLACKBOX_DEVICE      BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_BURST          DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE  179
#define BEEPER_INVERTED
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI2
