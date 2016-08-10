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

#define TARGET_BOARD_IDENTIFIER "SPEV"

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_8
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEP_GPIO   GPIOC
#define BEEP_PIN    Pin_15
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOC
#define BEEPER_INVERTED

#define USABLE_TIMER_CHANNEL_COUNT 12 // PPM, 8 PWM, UART3 RX/TX, LED Strip

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH


#define GYRO
//#define USE_FAKE_GYRO
#define USE_GYRO_SPI_MPU6500

#define ACC
//#define USE_FAKE_ACC
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6500_ALIGN CW180_DEG
#define GYRO_MPU6500_ALIGN CW180_DEG

#define BARO
#define USE_BARO_BMP280

#define MAG
#define USE_MPU9250_MAG // Enables bypass configuration
#define USE_MAG_AK8963
//#define USE_MAG_HMC5883 // External

#define MAG_AK8963_ALIGN CW90_DEG_FLIP

//#define SONAR
#define BEEPER
#define LED0

#define USB_IO

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT 4

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

#define SPI1_GPIO               GPIOB
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI1_NSS_PIN            Pin_9
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource9
#define SPI1_SCK_PIN            Pin_3
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource3
#define SPI1_MISO_PIN           Pin_4
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource4
#define SPI1_MOSI_PIN           Pin_5
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource5

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

#define USE_SDCARD
#define USE_SDCARD_SPI2

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   GPIO_Pin_14
#define SDCARD_DETECT_GPIO_PORT             GPIOC
#define SDCARD_DETECT_GPIO_CLK              RCC_AHBPeriph_GPIOC
#define SDCARD_DETECT_IO                    PC14

#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_GPIO                  SPI2_GPIO
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

// Note, this is the same DMA channel as USART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA1_FLAG_TC5

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   SPI1_GPIO_PERIPHERAL
#define MPU6500_CS_GPIO                  SPI1_GPIO
#define MPU6500_CS_PIN                   GPIO_Pin_9
#define MPU6500_SPI_INSTANCE             SPI1

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
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1Channel2Descriptor

#define TRANSPONDER
#define TRANSPONDER_GPIO                     GPIOA
#define TRANSPONDER_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define TRANSPONDER_GPIO_AF                  GPIO_AF_6
#define TRANSPONDER_PIN                      GPIO_Pin_8
#define TRANSPONDER_PIN_SOURCE               GPIO_PinSource8
#define TRANSPONDER_TIMER                    TIM1
#define TRANSPONDER_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define TRANSPONDER_DMA_CHANNEL              DMA1_Channel2
#define TRANSPONDER_IRQ                      DMA1_Channel2_IRQn
#define TRANSPONDER_DMA_TC_FLAG              DMA1_FLAG_TC2
#define TRANSPONDER_DMA_HANDLER_IDENTIFER    DMA1Channel2Descriptor

#define DEFAULT_RX_FEATURE FEATURE_RX_PPM
#define DEFAULT_FEATURES (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_CURRENT_METER | FEATURE_TELEMETRY)

#define GPS
#define BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define TELEMETRY
#define SERIAL_RX
#define GTUNE
#define DISPLAY
#define USE_SERVOS
#define USE_CLI
#define USE_EXTI

#define SPEKTRUM_BIND
// USART3,
#define BIND_PORT  GPIOB
#define BIND_PIN   Pin_11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF (BIT(0)|BIT(1))
