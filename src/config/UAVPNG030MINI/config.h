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

#define BOARD_NAME        UAVPNG030MINI
#define MANUFACTURER_ID   NGUA

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000

#define BEEPER_PIN           PB0
#define MOTOR1_PIN           PA8
#define MOTOR2_PIN           PE11
#define MOTOR3_PIN           PE13
#define MOTOR4_PIN           PE14
#define MOTOR5_PIN           PB5
#define MOTOR6_PIN           PD13
#define MOTOR7_PIN           PD14
#define MOTOR8_PIN           PD15
#define RX_PPM_PIN           PA2
#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART6_TX_PIN         PC7
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART6_RX_PIN         PC6
#define I2C1_SCL_PIN         PB8
#define I2C2_SCL_PIN         PB10
#define I2C1_SDA_PIN         PB9
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PE5
#define LED1_PIN             PE7
#define LED2_PIN             PE6
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7
#define ESCSERIAL_PIN        PA2
#define ADC_VBAT_PIN         PC1
#define BARO_CS_PIN          PE1
#define COMPASS_EXTI_PIN     PE12
#define GYRO_1_EXTI_PIN      PE0
#define GYRO_1_CS_PIN        PA4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PE11, 1,  0) \
    TIMER_PIN_MAP( 1, PE13, 1,  0) \
    TIMER_PIN_MAP( 2, PE14, 1,  0) \
    TIMER_PIN_MAP( 3, PD13, 1,  0) \
    TIMER_PIN_MAP( 4, PD14, 1,  0) \
    TIMER_PIN_MAP( 5, PD15, 1, -1)


#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE (I2CDEV_2)
#define USE_BARO
#define BARO_SPI_INSTANCE SPI1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define DASHBOARD_I2C_INSTANCE (I2CDEV_2)
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
#define GYRO_1_ALIGN_YAW 1800
