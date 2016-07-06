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

#define TARGET_BOARD_IDENTIFIER "SOF3"

#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_15 // PBA15 (LED)
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOA

// FIXME The board has a bus switch, not LED1
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_3 // PB3
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOA

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI1_CS_GPIO      GPIOA
#define SPI1_CS_PIN       GPIO_Pin_4

#define SPI2_CS_GPIO      GPIOB
#define SPI2_CS_PIN       GPIO_Pin_12

#define M25P16_CS_GPIO        SPI1_CS_GPIO
#define M25P16_CS_PIN         SPI1_CS_PIN
#define M25P16_SPI_INSTANCE   SPI1

#define MAX7456_CS_GPIO        SPI2_CS_GPIO
#define MAX7456_CS_PIN         SPI2_CS_PIN
#define MAX7456_SPI_INSTANCE   SPI2

#define MAX7456_NRST_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define MAX7456_NRST_GPIO               GPIOB
#define MAX7456_NRST_PIN                Pin_2

#define EXTI_CALLBACK_HANDLER_COUNT 3 // LOS, HSYNC, VSYNC

#define USE_FLASHFS
#define USE_FLASHTOOLS
#define USE_FLASH_M25P16

#define LED0
#define LED1

#define USB_IO

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define SERIAL_PORT_COUNT 3

#ifndef UART1_GPIO
#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10
#endif

#define USE_MSP_CLIENT

#define USE_ADC
//#define DEBUG_ADC_CHANNELS

// 12v
#define ADC1_GPIO               GPIOA
#define ADC1_GPIO_PIN           GPIO_Pin_0
#define ADC1_CHANNEL            ADC_Channel_1

// 5v
#define ADC2_GPIO               GPIOA
#define ADC2_GPIO_PIN           GPIO_Pin_1
#define ADC2_CHANNEL            ADC_Channel_2

// vbat
#define ADC3_GPIO               GPIOA
#define ADC3_GPIO_PIN           GPIO_Pin_2
#define ADC3_CHANNEL            ADC_Channel_3

// current
#define ADC4_GPIO               GPIOA
#define ADC4_GPIO_PIN           GPIO_Pin_3
#define ADC4_CHANNEL            ADC_Channel_4


