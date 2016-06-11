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

#define TARGET_BOARD_IDENTIFIER "SING"

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEP_GPIO   GPIOC
#define BEEP_PIN    Pin_15
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOC

#define USABLE_TIMER_CHANNEL_COUNT 10

#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN CW0_DEG_FLIP

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN CW0_DEG_FLIP

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define BEEPER
#define LED0

#define USE_VCP
#define USE_USART1          // JST-SH Serial - TX (PA9) RX (PA10)
#define USE_USART2          // Input - TX (NC) RX (PA15)
#define USE_USART3          // Solder Pads - TX (PB10) RX (PB11)
#define USE_SOFTSERIAL1     // Telemetry
#define SERIAL_PORT_COUNT 5

#define UART1_TX_PIN        GPIO_Pin_9
#define UART1_RX_PIN        GPIO_Pin_10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_14 //Not connected
#define UART2_RX_PIN        GPIO_Pin_15
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource14
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#define UART3_TX_PIN        GPIO_Pin_10
#define UART3_RX_PIN        GPIO_Pin_11
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

#define SOFTSERIAL_1_TIMER TIM15
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 7 //Not connected
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 8

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

#define USE_SPI
#define USE_SPI_DEVICE_1 // PA4, 5, 6, 7
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define VTX
#define RTC6705_CS_GPIO         GPIOA
#define RTC6705_CS_PIN          GPIO_Pin_4
#define RTC6705_SPI_INSTANCE    SPI1

#define M25P16_CS_GPIO          GPIOB
#define M25P16_CS_PIN           GPIO_Pin_12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC2
#define ADC_DMA_CHANNEL             DMA2_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2

#define VBAT_ADC_GPIO               GPIOB
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_2
#define VBAT_ADC_CHANNEL            ADC_Channel_12

#define LED_STRIP
#define LED_STRIP_TIMER TIM1

#define USE_LED_STRIP_ON_DMA1_CHANNEL2
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_6
#define WS2811_PIN                      GPIO_Pin_8
#define WS2811_PIN_SOURCE               GPIO_PinSource8
#define WS2811_TIMER                    TIM1
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

#define AUTOTUNE
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define GPS
#define USE_SERVOS
#define USE_CLI
#define DEFAULT_RX_FEATURE FEATURE_RX_PPM
#define DEFAULT_FEATURES (FEATURE_BLACKBOX | FEATURE_RX_SERIAL)

#define SPEKTRUM_BIND
// USART2, PA15
#define BIND_PORT GPIOA
#define BIND_PIN Pin_15

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

