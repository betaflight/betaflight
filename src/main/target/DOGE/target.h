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

#define TARGET_BOARD_IDENTIFIER "DOGE"

// tqfp48 pin 34
#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_13
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOA

// tqfp48 pin 37
#define LED1_GPIO   GPIOA
#define LED1_PIN    Pin_14
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOA

// tqfp48 pin 38
#define LED2_GPIO   GPIOA
#define LED2_PIN    Pin_15
#define LED2_PERIPHERAL RCC_AHBPeriph_GPIOA

#define BEEP_GPIO   GPIOB
#define BEEP_PIN    Pin_2
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOB
#define BEEPER_INVERTED

// tqfp48 pin 3
#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOC
#define MPU6500_CS_GPIO                  GPIOC
#define MPU6500_CS_PIN                   GPIO_Pin_14
#define MPU6500_SPI_INSTANCE             SPI1

// tqfp48 pin 25
#define BMP280_CS_GPIO_CLK_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define BMP280_CS_GPIO                   GPIOB
#define BMP280_CS_PIN                    GPIO_Pin_12
#define BMP280_SPI_INSTANCE              SPI2

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI1_GPIO               GPIOB
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
// tqfp48 pin 39
#define SPI1_SCK_PIN            GPIO_Pin_3
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource3
// tqfp48 pin 40
#define SPI1_MISO_PIN           GPIO_Pin_4
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource4
// tqfp48 pin 41
#define SPI1_MOSI_PIN           GPIO_Pin_5
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource5

#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
// tqfp48 pin 26
#define SPI2_SCK_PIN            GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
// tqfp48 pin 27
#define SPI2_MISO_PIN           GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
// tqfp48 pin 28
#define SPI2_MOSI_PIN           GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15

// timer definitions in drivers/timer.c
// channel mapping in drivers/pwm_mapping.c
// only 6 outputs available on hardware
#define USABLE_TIMER_CHANNEL_COUNT 9

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define GYRO
// #define USE_FAKE_GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN CW270_DEG // ??

#define ACC
// #define USE_FAKE_ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN CW270_DEG // ??

#define BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280

#define BEEPER
#define LED0
#define LED1
#define LED2

#define USB_IO
#define USE_VCP
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define SERIAL_PORT_COUNT 4

// tqfp48 pin 42
#define UART1_TX_PIN        GPIO_Pin_6
// tqfp48 pin 43
#define UART1_RX_PIN        GPIO_Pin_7
#define UART1_GPIO          GPIOB
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource6
#define UART1_RX_PINSOURCE  GPIO_PinSource7

// tqfp48 pin 12
#define UART2_TX_PIN        GPIO_Pin_2
// tqfp48 pin 13
#define UART2_RX_PIN        GPIO_Pin_3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource2
#define UART2_RX_PINSOURCE  GPIO_PinSource3

// tqfp48 pin 21
#define UART3_TX_PIN        GPIO_Pin_10
// tqfp48 pin 22
#define UART3_RX_PIN        GPIO_Pin_11
#define UART3_GPIO          GPIOB
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC2
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2
#define ADC_DMA_CHANNEL             DMA2_Channel1

// tqfp48 pin 14
#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_1

// tqfp48 pin 15
#define CURRENT_METER_ADC_GPIO      GPIOA
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_5
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_2

// mpu_int definition in sensors/initialisation.c
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define BLACKBOX
#define GPS
//#define GTUNE
#define LED_STRIP

// tqfp48 pin 16
#define LED_STRIP_TIMER TIM16
#define USE_LED_STRIP_ON_DMA1_CHANNEL3
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
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI
#define DEFAULT_RX_FEATURE FEATURE_RX_PPM

#define SPEKTRUM_BIND
// Use UART3 for speksat
#define BIND_PORT  GPIOB
#define BIND_PIN   Pin_11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE
