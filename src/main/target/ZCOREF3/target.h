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

#define TARGET_BOARD_IDENTIFIER "ZCF3"

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define LED0    	PB8

#define BEEPER      PC15
#define BEEPER_INVERTED

#define EXTI15_10_CALLBACK_HANDLER_COUNT 1

#define USE_EXTI
#define MPU_INT_EXTI PC13
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH


#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6500_ALIGN 	CW180_DEG
#define GYRO_MPU6500_ALIGN 	CW180_DEG

#define BARO
#define USE_BARO_BMP280

#define USE_USART1
#define USE_USART2
#define USE_USART3
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
#define USE_SPI_DEVICE_1 // PB9,3,4,5 on AF5 SPI1 (MPU)
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5 SPI2 (SDCard)

#define SPI1_NSS_PIN            PB9
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5
 
#define MPU6500_CS_PIN          PB9
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER
#define ADC_INSTANCE                ADC2
#define VBAT_ADC_PIN                PA4
#define CURRENT_METER_ADC_PIN       PA5
#define RSSI_ADC_PIN                PB2

#define DEFAULT_RX_FEATURE FEATURE_RX_PPM
#define DEFAULT_FEATURES (FEATURE_TRANSPONDER | FEATURE_BLACKBOX | FEATURE_RSSI_ADC | FEATURE_CURRENT_METER | FEATURE_TELEMETRY)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 17 // PPM, 8 PWM, UART3 RX/TX, LED Strip

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) | TIM_N(17) )

