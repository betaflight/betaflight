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

#define TARGET_BOARD_IDENTIFIER "LUXR" // LUX Race

#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_15
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOC

#define LED1_GPIO   GPIOC
#define LED1_PIN    Pin_14
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOC

#define LED2_GPIO   GPIOC
#define LED2_PIN    Pin_13
#define LED2_PERIPHERAL RCC_AHBPeriph_GPIOC

#define BEEP_GPIO   GPIOB
#define BEEP_PIN    Pin_13
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOB
#define BEEPER_INVERTED

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOA
#define MPU6500_CS_GPIO                  GPIOA
#define MPU6500_CS_PIN                   GPIO_Pin_4
#define MPU6500_SPI_INSTANCE             SPI1

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_GPIO               GPIOB
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI1_SCK_PIN            GPIO_Pin_3
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource3
#define SPI1_MISO_PIN           GPIO_Pin_4
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource4
#define SPI1_MOSI_PIN           GPIO_Pin_5
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource5

#define USABLE_TIMER_CHANNEL_COUNT 11

#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN CW270_DEG

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN CW270_DEG

#define BEEPER
#define LED0
#define LED1
#define LED2

#define USB_IO

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT 4

#define UART1_TX_PIN        GPIO_Pin_4
#define UART1_RX_PIN        GPIO_Pin_5
#define UART1_GPIO          GPIOC
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource4
#define UART1_RX_PINSOURCE  GPIO_PinSource5

#define UART2_TX_PIN        GPIO_Pin_14
#define UART2_RX_PIN        GPIO_Pin_15
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource14
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#define UART3_TX_PIN        GPIO_Pin_10
#define UART3_RX_PIN        GPIO_Pin_11
#define UART3_GPIO          GPIOB
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

#define ADC0_GPIO                   GPIOC
#define ADC0_GPIO_PIN               GPIO_Pin_0
#define ADC0_CHANNEL                ADC_Channel_6

#define ADC1_GPIO                   GPIOC
#define ADC1_GPIO_PIN               GPIO_Pin_1
#define ADC1_CHANNEL                ADC_Channel_7

#define ADC2_GPIO                   GPIOC
#define ADC2_GPIO_PIN               GPIO_Pin_2
#define ADC2_CHANNEL                ADC_Channel_8

#define ADC3_GPIO                   GPIOC
#define ADC3_GPIO_PIN               GPIO_Pin_3
#define ADC3_CHANNEL                ADC_Channel_9

#define ADC_CHANNEL_COUNT 4

#define ADC_BATTERY     ADC_CHANNEL0
#define ADC_CURRENT     ADC_CHANNEL1
#define ADC_RSSI        ADC_CHANNEL2
#define ADC_EXTERNAL    ADC_CHANNEL3

#define DEFAULT_RX_FEATURE FEATURE_RX_PPM

#define BLACKBOX
#define GPS
#define GTUNE
#define LED_STRIP

#define LED_STRIP_TIMER TIM16

#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_1
#define WS2811_PIN                      GPIO_Pin_6 // TIM16_CH1
#define WS2811_PIN_SOURCE               GPIO_PinSource6
#define WS2811_TIMER                    TIM16
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1Channel3Descriptor


// MPU6500 interrupt
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI
#define USE_EXTI

#define SPEKTRUM_BIND
// USART1, PC5
#define BIND_PORT  GPIOC
#define BIND_PIN   Pin_5

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(4))
