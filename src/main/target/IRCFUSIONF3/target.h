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

#define TARGET_BOARD_IDENTIFIER "IFF3"

#define LED0
#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define USABLE_TIMER_CHANNEL_COUNT 17

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN CW270_DEG

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN CW270_DEG

#define BARO
#define USE_BARO_BMP085

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT 3

#ifndef UART1_GPIO
#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10
#endif

#define UART2_TX_PIN        GPIO_Pin_14 // PA14 / SWCLK
#define UART2_RX_PIN        GPIO_Pin_15 // PA15
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource14
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#ifndef UART3_GPIO
#define UART3_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11
#endif

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define M25P16_CS_GPIO          GPIOB
#define M25P16_CS_PIN           GPIO_Pin_12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC2
#define ADC_DMA_CHANNEL             DMA2_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2

#define ADC0_GPIO                   GPIOA
#define ADC0_GPIO_PIN               GPIO_Pin_4
#define ADC0_CHANNEL                ADC_Channel_1

#define ADC1_GPIO                   GPIOA
#define ADC1_GPIO_PIN               GPIO_Pin_5
#define ADC1_CHANNEL                ADC_Channel_2

#define ADC2_GPIO                   GPIOB
#define ADC2_GPIO_PIN               GPIO_Pin_2
#define ADC2_CHANNEL                ADC_Channel_12

#define ADC_CHANNEL_COUNT 3

#define ADC_BATTERY     ADC_CHANNEL0
#define ADC_CURRENT     ADC_CHANNEL1
#define ADC_RSSI        ADC_CHANNEL2

#define BLACKBOX
#define GPS
#define GTUNE
#define SERIAL_RX
#define USE_SERVOS
#define TELEMETRY
#define USE_CLI
#define USE_EXTI

#define SPEKTRUM_BIND
// USART3,
#define BIND_PORT  GPIOB
#define BIND_PIN   Pin_11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package ?
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF (BIT(0)|BIT(1))
