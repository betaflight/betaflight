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

#define TARGET_BOARD_IDENTIFIER "SRFM"

// early prototype had slightly different pin mappings.
//#define SPRACINGF3MINI_MKII_REVA

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEP_GPIO   GPIOC
#define BEEP_PIN    Pin_15
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOC
#define BEEPER_INVERTED

#define USABLE_TIMER_CHANNEL_COUNT 14 // 8 Outputs, PPM, LED Strip, SonarPads & 2 pins shared by UART3 RX/TX pins

#define EXTI15_10_CALLBACK_HANDLER_COUNT 2 // MPU_INT, SDCardDetect

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH


#define GYRO
//#define USE_FAKE_GYRO
#define USE_GYRO_MPU6500

#define ACC
//#define USE_FAKE_ACC
#define USE_ACC_MPU6500

#define ACC_MPU6500_ALIGN CW180_DEG
#define GYRO_MPU6500_ALIGN CW180_DEG

#define BARO
#define USE_BARO_BMP280

#define MAG
//#define USE_MPU9250_MAG // Enables bypass configuration
#define USE_MAG_AK8975
#define USE_MAG_HMC5883 // External

#define MAG_AK8975_ALIGN CW270_DEG_FLIP

#define SONAR
#define BEEPER
#define LED0

#define USE_VCP
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT 5

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

#define SOFTSERIAL_1_TIMER TIM2
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 9 // PA0 / PAD3
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 10 // PA1 / PAD4

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_NSS_PIN            Pin_12
#define SPI2_NSS_PIN_SOURCE     GPIO_PinSource12
#define SPI2_SCK_PIN            Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN           Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN           Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15

#define USE_SD_CARD

#define SD_DETECT_INVERTED

#define SD_DETECT_PIN                   GPIO_Pin_14
#define SD_DETECT_EXTI_LINE             EXTI_Line14
#define SD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource14
#define SD_DETECT_GPIO_PORT             GPIOC
#define SD_DETECT_GPIO_CLK              RCC_AHBPeriph_GPIOC
#define SD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOC
#define SD_DETECT_EXTI_IRQn             EXTI15_10_IRQn

#define SD_CS_GPIO                      SPI2_GPIO
#define SD_CS_PIN                       SPI2_NSS_PIN
#define SD_SPI_INSTANCE                 SPI2
#define SD_CS_GPIO_CLK                  RCC_AHBPeriph_GPIOB

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER


#define ADC_INSTANCE                ADC2
#define ADC_DMA_CHANNEL             DMA2_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define CURRENT_METER_ADC_GPIO      GPIOA
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_5
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_2

#define RSSI_ADC_GPIO               GPIOB
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_2
#define RSSI_ADC_CHANNEL            ADC_Channel_12

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

#define GPS
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define AUTOTUNE
#define DISPLAY
#define USE_SERVOS
#define USE_CLI

#define SPEKTRUM_BIND
// USART3,
#define BIND_PORT  GPIOB
#define BIND_PIN   Pin_11
