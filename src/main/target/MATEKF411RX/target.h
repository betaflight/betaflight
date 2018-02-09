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

#define TARGET_BOARD_IDENTIFIER "M41R"
#define USBD_PRODUCT_STRING     "MATEKF411RX"

#define LED0_PIN                PC13

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_SPI

// *************** SPI1 Gyro & ACC **********************
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define USE_EXTI
#define MPU_INT_EXTI            PA1
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

// *************** SPI2 OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12

// *************** SPI3 CC2500 ***************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#define SPI3_NSS_PIN            PA15

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI3
#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA

#define RX_SCK_PIN              SPI3_SCK_PIN
#define RX_MISO_PIN             SPI3_MISO_PIN
#define RX_MOSI_PIN             SPI3_MOSI_PIN
#define RX_NSS_PIN              SPI3_NSS_PIN

#define RX_FRSKY_SPI_DISABLE_CHIP_DETECTION
#define RX_FRSKY_SPI_GDO_0_PIN     PC14
#define RX_FRSKY_SPI_LED_PIN       PB9
#define RX_FRSKY_SPI_LED_PIN_INVERTED

#define USE_RX_FRSKY_SPI_PA_LNA
#define RX_FRSKY_SPI_TX_EN_PIN      PA8
#define RX_FRSKY_SPI_LNA_EN_PIN     PA13

#define USE_RX_FRSKY_SPI_DIVERSITY
#define RX_FRSKY_SPI_ANT_SEL_PIN    PA14

#define BINDPLUG_PIN             PB2

#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKY_X
#define USE_RX_FRSKY_SPI_TELEMETRY

// *************** UART *****************************
#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT       4

// *************** ADC *****************************
#define USE_ADC
#define ADC1_DMA_STREAM         DMA2_Stream0
#define VBAT_ADC_PIN            PB0
#define CURRENT_METER_ADC_PIN   PB1

#define USE_ESCSERIAL
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY )
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 179

#define USE_LED_STRIP

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

//#define  USE_DSHOT_DMA

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             ( TIM_N(1)|TIM_N(2)|TIM_N(4)|TIM_N(5)|TIM_N(9))
