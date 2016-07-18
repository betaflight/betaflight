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

#define TARGET_BOARD_IDENTIFIER "SOF1"

#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_15 // PBA15 (LED)
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOA

// FIXME The board has a bus switch, not LED1
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_3 // PB3
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOA

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

#define MAX7456_NRST_GPIO_PERIPHERAL    RCC_APB2Periph_GPIOB
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

#define USE_MSP_CLIENT

#define USE_EXTI
#define USE_ADC
//#define DEBUG_ADC_CHANNELS

#define ADC_INSTANCE                ADC1
#define ADC_ABP2_PERIPHERAL         RCC_APB2Periph_ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

// 12v
#define ADC0_GPIO               GPIOA
#define ADC0_GPIO_PIN           GPIO_Pin_0
#define ADC0_CHANNEL            ADC_Channel_0

// 5v
#define ADC1_GPIO               GPIOA
#define ADC1_GPIO_PIN           GPIO_Pin_1
#define ADC1_CHANNEL            ADC_Channel_1

//vbat
#define ADC2_GPIO               GPIOA
#define ADC2_GPIO_PIN           GPIO_Pin_2
#define ADC2_CHANNEL            ADC_Channel_2

// current
#define ADC3_GPIO               GPIOA
#define ADC3_GPIO_PIN           GPIO_Pin_3
#define ADC3_CHANNEL            ADC_Channel_3

// adc channel mapping
#define ADC_CHANNEL_COUNT 4

#define ADC_POWER_12V ADC_CHANNEL0
#define ADC_POWER_5V ADC_CHANNEL1
#define ADC_BATTERY ADC_CHANNEL2
#define ADC_CURRENT ADC_CHANNEL3

// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))

