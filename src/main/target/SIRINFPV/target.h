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

#define TARGET_BOARD_IDENTIFIER "SIRF"

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_2
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_1
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOA

#define USABLE_TIMER_CHANNEL_COUNT 6

#define EXTI15_10_CALLBACK_HANDLER_COUNT 1 // MPU_INT

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500

#define ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500

// MPU6000
#define ACC_MPU6000_ALIGN CW180_DEG
#define GYRO_MPU6000_ALIGN CW180_DEG

#define MPU6000_CS_GPIO GPIOA
#define MPU6000_CS_PIN GPIO_Pin_4
#define MPU6000_SPI_INSTANCE SPI1

// MPU6500
#define ACC_MPU6500_ALIGN CW90_DEG
#define GYRO_MPU6500_ALIGN CW90_DEG

#define MPU6500_CS_GPIO GPIOA
#define MPU6500_CS_PIN GPIO_Pin_4
#define MPU6500_SPI_INSTANCE SPI1
#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOA


#define BEEPER

#define USB_IO

//#define USB_CABLE_DETECTION
//#define USB_DETECT_PIN                   GPIO_Pin_2
//#define USB_DETECT_GPIO_PORT             GPIOB
//#define USB_DETECT_GPIO_CLK              RCC_AHBPeriph_GPIOB

#define USE_VCP
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define SERIAL_PORT_COUNT 4

#ifndef UART1_GPIO
#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10
#endif

#define UART2_TX_PIN        GPIO_Pin_2 // PA14 / SWCLK
#define UART2_RX_PIN        GPIO_Pin_3 // PA15
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource2
#define UART2_RX_PINSOURCE  GPIO_PinSource3

#ifndef UART3_GPIO
#define UART3_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11
#endif


#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5
#define USE_SPI_DEVICE_3

#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
#define SPI1_NSS_PIN            Pin_4
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource4
#define SPI1_SCK_PIN            Pin_5
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource5
#define SPI1_MISO_PIN           Pin_6
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource6
#define SPI1_MOSI_PIN           Pin_7
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource7

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

#define SPI3_GPIO               GPIOB
#define SPI3_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI3_NSS_GPIO           GPIOA
#define SPI3_NSS_PERIPHERAL     RCC_AHBPeriph_GPIOA
#define SPI3_NSS_PIN            GPIO_Pin_15
#define SPI3_NSS_PIN_SOURCE     GPIO_PinSource15
#define SPI3_SCK_PIN            GPIO_Pin_3
#define SPI3_SCK_PIN_SOURCE     GPIO_PinSource3
#define SPI3_MISO_PIN           GPIO_Pin_4
#define SPI3_MISO_PIN_SOURCE    GPIO_PinSource4
#define SPI3_MOSI_PIN           GPIO_Pin_5
#define SPI3_MOSI_PIN_SOURCE    GPIO_PinSource5

#define USE_MAX7456
#define MAX7456_CS_GPIO          GPIOA
#define MAX7456_CS_PIN           GPIO_Pin_15
#define MAX7456_SPI_INSTANCE     SPI3

#define USE_RTC6705
#define RTC6705_SPIDATA_GPIO        GPIOC
#define RTC6705_SPIDATA_PIN         Pin_15
#define RTC6705_SPIDATA_PERIPHERAL  RCC_AHBPeriph_GPIOC
#define RTC6705_SPILE_GPIO          GPIOC
#define RTC6705_SPILE_PIN           Pin_14
#define RTC6705_SPILE_PERIPHERAL    RCC_AHBPeriph_GPIOC
#define RTC6705_SPICLK_GPIO         GPIOC
#define RTC6705_SPICLK_PIN          Pin_13
#define RTC6705_SPICLK_PERIPHERAL   RCC_AHBPeriph_GPIOC

#define USE_SDCARD
#define USE_SDCARD_SPI2

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

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC1
#define ADC_DMA_CHANNEL             DMA1_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
#define VBAT_ADC_CHANNEL            ADC_Channel_1

//#define USE_QUAD_MIXER_ONLY
#define BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define TELEMETRY
#define SERIAL_RX
#define USE_CLI
#define OSD
#define LED0

#define USE_SERVOS
#define DEFAULT_RX_FEATURE FEATURE_RX_PPM

#define USE_SERIAL_4WAY_BLHELI_INTERFACE
