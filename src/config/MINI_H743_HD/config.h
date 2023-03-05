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

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        MINI_H743_HD
#define MANUFACTURER_ID   RAST

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BEEPER_PIN           PB3
#define MOTOR1_PIN           PD12
#define MOTOR2_PIN           PD13
#define MOTOR3_PIN           PB0
#define MOTOR4_PIN           PB1
#define MOTOR5_PIN           PC8
#define MOTOR6_PIN           PC9
#define MOTOR7_PIN           PB4
#define MOTOR8_PIN           PB5
#define RX_PPM_PIN           PA3
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C1_SCL_PIN         PB8
#define I2C2_SCL_PIN         PB10
#define I2C1_SDA_PIN         PB9
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PE2
#define LED1_PIN             PE3
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI4_SCK_PIN         PE12
#define SPI1_MISO_PIN        PA6
#define SPI2_MISO_PIN        PB14
#define SPI3_MISO_PIN        PC11
#define SPI4_MISO_PIN        PE13
#define SPI1_MOSI_PIN        PA7
#define SPI2_MOSI_PIN        PB15
#define SPI3_MOSI_PIN        PC12
#define SPI4_MOSI_PIN        PE14
#define CAMERA_CONTROL_PIN   PD9
#define ADC_VBAT_PIN         PC3
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC2
#define ADC_EXTERNAL1_PIN    PC1
#define PINIO1_PIN           PD9
#define PINIO2_PIN           PD11
#define FLASH_CS_PIN         PB12
#define MAX7456_SPI_CS_PIN   PA15
#define GYRO_1_EXTI_PIN      PB2
#define GYRO_2_EXTI_PIN      PE15
#define GYRO_1_CS_PIN        PA4
#define GYRO_2_CS_PIN        PE11

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PD12, 1,  6) \
    TIMER_PIN_MAP( 1, PA3 , 2,  1) \
    TIMER_PIN_MAP( 2, PC8 , 2,  0) \
    TIMER_PIN_MAP( 3, PB0 , 2,  0) \
    TIMER_PIN_MAP( 4, PB1 , 2,  1) \
    TIMER_PIN_MAP( 5, PE9 , 1,  0) \
    TIMER_PIN_MAP( 6, PE11, 1,  0) \
    TIMER_PIN_MAP( 7, PB4 , 1,  4) \
    TIMER_PIN_MAP( 8, PB5 , 1,  5) \
    TIMER_PIN_MAP( 9, PE13, 1,  6) \
    TIMER_PIN_MAP(10, PE14, 1,  7) \
    TIMER_PIN_MAP(11, PA0 , 2,  2) \
    TIMER_PIN_MAP(12, PA1 , 2,  4) \
    TIMER_PIN_MAP(13, PA2 , 2,  0) \
    TIMER_PIN_MAP(14, PD13, 1,  7) \
    TIMER_PIN_MAP(15, PD14, 1, 12) \
    TIMER_PIN_MAP(16, PD15, 1, -1) \
    TIMER_PIN_MAP(17, PE5 , 1,  0) \
    TIMER_PIN_MAP(18, PE6 , 1, -1) \
    TIMER_PIN_MAP(19, PA8 , 1, 14) \
    TIMER_PIN_MAP(20, PA15, 1,  0)


#define ADC1_DMA_OPT        8
#define ADC2_DMA_OPT        9
#define ADC3_DMA_OPT       10
#define TIMUP1_DMA_OPT      0
#define TIMUP2_DMA_OPT      0
#define TIMUP3_DMA_OPT      0
#define TIMUP5_DMA_OPT      0
#define TIMUP8_DMA_OPT      4

#define MAG_I2C_INSTANCE (I2CDEV_2)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
//TODO #define MOTOR_PWM_PROTOCOL DSHOT600
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 250
#define MAX7456_SPI_INSTANCE SPI3
//TODO #define PINIO_BOX 40,41,255,255
#define FLASH_SPI_INSTANCE SPI2
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
#define GYRO_1_ALIGN_YAW 1800
#define GYRO_2_SPI_INSTANCE SPI4
#define GYRO_2_ALIGN CW0_DEG_FLIP
#define GYRO_2_ALIGN_PITCH 1800
