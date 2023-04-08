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

#define FC_TARGET_MCU     STM32F745

#define BOARD_NAME        ANYFCF7
#define MANUFACTURER_ID   FOSS

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_BARO_MS5611
#define USE_MAX7456
#define USE_SDCARD

#define BEEPER_PIN           PB2
#define MOTOR1_PIN           PB8
#define MOTOR2_PIN           PA2
#define MOTOR3_PIN           PA1
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PB5
#define MOTOR6_PIN           PA0
#define MOTOR7_PIN           PB9
#define MOTOR8_PIN           PE6
#define RX_PPM_PIN           PB14
#define RX_PWM1_PIN          PB14
#define RX_PWM2_PIN          PB15
#define RX_PWM3_PIN          PC6
#define RX_PWM4_PIN          PC7
#define RX_PWM5_PIN          PC8
#define RX_PWM6_PIN          PC9
#define LED_STRIP_PIN        PB3
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PC10
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PC11
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C2_SCL_PIN         PB10
#define I2C4_SCL_PIN         PD12
#define I2C2_SDA_PIN         PB11
#define I2C4_SDA_PIN         PD13
#define LED0_PIN             PB7
#define LED1_PIN             PB6
#define SPI1_SCK_PIN         PA5
#define SPI3_SCK_PIN         PC10
#define SPI4_SCK_PIN         PE12
#define SPI1_SDI_PIN         PA6
#define SPI3_SDI_PIN         PC11
#define SPI4_SDI_PIN         PE13
#define SPI1_SDO_PIN         PA7
#define SPI3_SDO_PIN         PC12
#define SPI4_SDO_PIN         PE14
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PC2
#define ADC_CURR_PIN         PC1
#define SDCARD_SPI_CS_PIN    PE11
#define SDCARD_DETECT_PIN    PD3
#define MAX7456_SPI_CS_PIN   PD2
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PA8

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB14, 3, -1) \
    TIMER_PIN_MAP( 1, PB15, 3, -1) \
    TIMER_PIN_MAP( 2, PC6 , 2,  0) \
    TIMER_PIN_MAP( 3, PC7 , 2,  1) \
    TIMER_PIN_MAP( 4, PC8 , 2,  1) \
    TIMER_PIN_MAP( 5, PC9 , 2,  0) \
    TIMER_PIN_MAP( 6, PB8 , 1,  0) \
    TIMER_PIN_MAP( 7, PA2 , 2,  0) \
    TIMER_PIN_MAP( 8, PA1 , 2,  0) \
    TIMER_PIN_MAP( 9, PA3 , 2,  0) \
    TIMER_PIN_MAP(10, PB5 , 1,  0) \
    TIMER_PIN_MAP(11, PA0 , 2,  0) \
    TIMER_PIN_MAP(12, PB9 , 1, -1) \
    TIMER_PIN_MAP(13, PE6 , 1, -1) \
    TIMER_PIN_MAP(14, PB3 , 1,  0) \
    TIMER_PIN_MAP(15, PB4 , 1,  0)



#define SPI4_TX_DMA_OPT     0
#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE (I2CDEV_2)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_2)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define BEEPER_INVERTED
#define SDCARD_DETECT_INVERTED
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI4
#define MAX7456_SPI_INSTANCE SPI3
#define DASHBOARD_I2C_INSTANCE (I2CDEV_2)
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
