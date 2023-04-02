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

#define BOARD_NAME        F4BY
#define MANUFACTURER_ID   FOSS

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_BARO_MS5611
#define USE_SDCARD

#define BEEPER_PIN           PE5
#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PE9
#define MOTOR6_PIN           PE11
#define MOTOR7_PIN           PE13
#define MOTOR8_PIN           PE14
#define RX_PWM1_PIN          PC9
#define RX_PWM2_PIN          PC8
#define RX_PWM3_PIN          PC6
#define RX_PWM4_PIN          PC7
#define RX_PWM5_PIN          PD15
#define RX_PWM6_PIN          PD14
#define RX_PWM7_PIN          PD13
#define RX_PWM8_PIN          PD12
#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PC10
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PC11
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART6   PD3
#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PE3
#define LED1_PIN             PE2
#define LED2_PIN             PE1
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB3
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB4
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define ESCSERIAL_PIN        PA0
#define ADC_VBAT_PIN         PC3
#define ADC_RSSI_PIN         PC1
#define ADC_CURR_PIN         PC2
#define SDCARD_SPI_CS_PIN    PE15
#define GYRO_1_EXTI_PIN      PB0
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PA9

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC9 , 1,  0) \
    TIMER_PIN_MAP( 1, PC8 , 1,  0) \
    TIMER_PIN_MAP( 2, PC6 , 1,  0) \
    TIMER_PIN_MAP( 3, PC7 , 1,  0) \
    TIMER_PIN_MAP( 4, PD15, 1, -1) \
    TIMER_PIN_MAP( 5, PD14, 1,  0) \
    TIMER_PIN_MAP( 6, PD13, 1,  0) \
    TIMER_PIN_MAP( 7, PD12, 1,  0) \
    TIMER_PIN_MAP( 8, PA0 , 1,  0) \
    TIMER_PIN_MAP( 9, PA1 , 1,  0) \
    TIMER_PIN_MAP(10, PA2 , 2,  0) \
    TIMER_PIN_MAP(11, PA3 , 2,  0) \
    TIMER_PIN_MAP(12, PE9 , 1,  0) \
    TIMER_PIN_MAP(13, PE11, 1,  0) \
    TIMER_PIN_MAP(14, PE13, 1,  0) \
    TIMER_PIN_MAP(15, PE14, 1,  0) \
    TIMER_PIN_MAP(16, PE6 , 1, -1)



#define SPI2_TX_DMA_OPT     0
#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE (I2CDEV_2)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_2)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI2
#define SYSTEM_HSE_MHZ 8
#define DASHBOARD_I2C_INSTANCE (I2CDEV_2)
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW90_DEG
#define GYRO_1_ALIGN_YAW 900
