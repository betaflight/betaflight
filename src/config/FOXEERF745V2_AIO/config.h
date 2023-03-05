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

#define BOARD_NAME        FOXEERF745V2_AIO
#define MANUFACTURER_ID   FOXE

#define USE_ACCGYRO_BMI270
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BEEPER_PIN           PD2
#define MOTOR1_PIN           PB4
#define MOTOR2_PIN           PB5
#define MOTOR3_PIN           PB1
#define MOTOR4_PIN           PB0
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART7_TX_PIN         PE8
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART7_RX_PIN         PE7
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define LED0_PIN             PC13
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI4_SCK_PIN         PE2
#define SPI1_MISO_PIN        PA6
#define SPI2_MISO_PIN        PB14
#define SPI3_MISO_PIN        PC11
#define SPI4_MISO_PIN        PE5
#define SPI1_MOSI_PIN        PA7
#define SPI2_MOSI_PIN        PB15
#define SPI3_MOSI_PIN        PC12
#define SPI4_MOSI_PIN        PE6
#define CAMERA_CONTROL_PIN   PB3
#define ADC_VBAT_PIN         PC3
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC2
#define ADC_EXTERNAL1_PIN    PC1
#define FLASH_CS_PIN         PE4
#define MAX7456_SPI_CS_PIN   PA4
#define GYRO_1_EXTI_PIN      PD0
#define GYRO_1_CS_PIN        PA15

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA8 , 1,  0) \
    TIMER_PIN_MAP( 1, PB3 , 1,  0) \
    TIMER_PIN_MAP( 2, PB0 , 2,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  0) \
    TIMER_PIN_MAP( 4, PB4 , 1,  0) \
    TIMER_PIN_MAP( 5, PB5 , 1,  0)



#define ADC1_DMA_OPT        0

#define MAG_ALIGN CW180_DEG
#define MAG_ALIGN_YAW 1800
//TODO #define MAG_BUSTYPE I2C
#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
//TODO #define MOTOR_PWM_PROTOCOL DSHOT600
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 100
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI1
#define FLASH_SPI_INSTANCE SPI4
#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI3
#define GYRO_1_ALIGN CW0_DEG
#define GYRO_1_ALIGN_YAW 1800
