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

#define BOARD_NAME        BLUEJAYF4
#define MANUFACTURER_ID   BKMN

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_FLASH
#define USE_FLASH_W25P16

#define BEEPER_PIN           PC1
#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PB0
#define MOTOR6_PIN           PB1
#define RX_PPM_PIN           PC7
#define LED_STRIP_PIN        PB0
#define UART1_TX_PIN         PA9
#define UART3_TX_PIN         PB10
#define UART6_TX_PIN         PC6
#define SOFTSERIAL1_TX_PIN   PB3
#define UART1_RX_PIN         PA10
#define UART3_RX_PIN         PB11
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART6   PB15
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define LED0_PIN             PB6
#define LED1_PIN             PB5
#define LED2_PIN             PB4
#define SPI1_SCK_PIN         PA5
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI3_SDO_PIN         PC12
#define ESCSERIAL_PIN        PC7
#define ADC_VBAT_PIN         PC3
#define ADC_CURR_PIN         PC2
#define SDCARD_SPI_CS_PIN    PA15
#define SDCARD_DETECT_PIN    PD2
#define FLASH_CS_PIN         PB7
#define GYRO_1_EXTI_PIN      PC5
#define GYRO_1_CS_PIN        PC4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC7 , 1,  0) \
    TIMER_PIN_MAP( 1, PA0 , 2,  0) \
    TIMER_PIN_MAP( 2, PA1 , 2,  0) \
    TIMER_PIN_MAP( 3, PA2 , 2,  0) \
    TIMER_PIN_MAP( 4, PA3 , 2,  1) \
    TIMER_PIN_MAP( 5, PB0 , 1,  0) \
    TIMER_PIN_MAP( 6, PB1 , 3,  0) \
    TIMER_PIN_MAP( 7, PB3 , 1,  0)



#define SPI3_TX_DMA_OPT     0
#define ADC1_DMA_OPT        1

#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
#define SDCARD_DETECT_INVERTED
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE SPI3
#define SYSTEM_HSE_MHZ 8
#define DASHBOARD_I2C_INSTANCE (I2CDEV_1)
#define FLASH_SPI_INSTANCE SPI3
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1
