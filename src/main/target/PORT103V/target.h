/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "103V"

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_13
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOB

#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_14
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOB

#define LED2_GPIO   GPIOB
#define LED2_PIN    Pin_15
#define LED2_PERIPHERAL RCC_APB2Periph_GPIOB

#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12 // PA12 (Beeper)
#define BEEP_PERIPHERAL RCC_APB2Periph_GPIOA

#define INVERTER_PIN Pin_2 // PB2 (BOOT1) abused as inverter select GPIO
#define INVERTER_GPIO GPIOB
#define INVERTER_PERIPHERAL RCC_APB2Periph_GPIOB
#define INVERTER_USART USART2

#define MPU6000_CS_GPIO       GPIOB
#define MPU6000_CS_PIN        GPIO_Pin_12
#define MPU6000_SPI_INSTANCE  SPI2

#define MPU6500_CS_GPIO       GPIOB
#define MPU6500_CS_PIN        GPIO_Pin_12
#define MPU6500_SPI_INSTANCE  SPI2
#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOB

#define GYRO
#define USE_FAKE_GYRO
//#define USE_GYRO_L3G4200D
//#define USE_GYRO_L3GD20
//#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
//#define USE_GYRO_SPI_MPU6000
//#define USE_GYRO_SPI_MPU6500

#define ACC
#define USE_FAKE_ACC
//#define USE_ACC_ADXL345
//#define USE_ACC_BMA280
//#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
//#define USE_ACC_SPI_MPU6000
//#define USE_ACC_SPI_MPU6500

#define BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_AK8975

#define SONAR
#define BEEPER
#define BEEPER_INVERTED
#define LED0
#define LED1
#define LED2
#define INVERTER
#define DISPLAY

//#define USE_VCP
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_USART4
#define USE_USART5
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 5

// #define SOFTSERIAL_1_TIMER TIM3
// #define SOFTSERIAL_1_TIMER_RX_HARDWARE 4 // PWM 5
// #define SOFTSERIAL_1_TIMER_TX_HARDWARE 5 // PWM 6
// #define SOFTSERIAL_2_TIMER TIM3
// #define SOFTSERIAL_2_TIMER_RX_HARDWARE 6 // PWM 7
// #define SOFTSERIAL_2_TIMER_TX_HARDWARE 7 // PWM 8

#define USART3_RX_PIN Pin_11
#define USART3_TX_PIN Pin_10
#define USART3_GPIO GPIOB
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_APB2_PERIPHERALS RCC_APB2Periph_GPIOB

// pins for UART4 are fixed by hardware
#define USART4_RX_PIN Pin_11
#define USART4_TX_PIN Pin_10
#define USART4_GPIO GPIOC
#define USART4_APB1_PERIPHERALS RCC_APB1Periph_UART4
#define USART4_APB2_PERIPHERALS RCC_APB2Periph_GPIOC

// pins for UART5 are fixed by hardware and on GPIOC and D
#define USART5_RX_PIN Pin_2 // GPIOD
#define USART5_TX_PIN Pin_12 // GPIOC
#define USART5_GPIO_TX GPIOC
#define USART5_GPIO_RX GPIOD
#define USART5_APB1_PERIPHERALS RCC_APB1Periph_UART5
#define USART5_APB2_PERIPHERALS_TX RCC_APB2Periph_GPIOC
#define USART5_APB2_PERIPHERALS_RX RCC_APB2Periph_GPIOD

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

#define USE_ADC

#define CURRENT_METER_ADC_GPIO      GPIOB
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_9

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_4

#define RSSI_ADC_GPIO               GPIOA
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_1
#define RSSI_ADC_CHANNEL            ADC_Channel_1

#define EXTERNAL1_ADC_GPIO          GPIOA
#define EXTERNAL1_ADC_GPIO_PIN      GPIO_Pin_5
#define EXTERNAL1_ADC_CHANNEL       ADC_Channel_5

#define LED_STRIP
#define LED_STRIP_TIMER TIM3

#define BLACKBOX
#define GPS
#define GTUNE
#define SERIAL_RX
#define TELEMETRY
#define USE_SERVOS
#define USE_CLI

#define USE_SERIAL_1WIRE

#define S1W_TX_GPIO         GPIOA
#define S1W_TX_PIN          GPIO_Pin_9
#define S1W_RX_GPIO         GPIOA
#define S1W_RX_PIN          GPIO_Pin_10
