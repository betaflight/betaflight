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

#define USB_VCP_ENABLED 1

#define TARGET_BOARD_IDENTIFIER "TFSH" // http://fishpepper.de/projects/tinyFISH


#define LED0_PIN                PC14
#define LED1_PIN                PA13

#define BEEPER                  PC15
#define BEEPER_INVERTED


#define USE_EXTI
#define MPU_INT_EXTI PC13
#define USE_MPU_DATA_READY_SIGNAL

#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6000_CS_PIN          PA4

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG_FLIP

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG_FLIP


#if USB_VCP_ENABLED
  #define USE_VCP
  #define USBD_PRODUCT_STRING "tinyFISH"
  #define SERIAL_PORT_COUNT 6
#else
  #define SERIAL_PORT_COUNT 5
#endif

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define SBUS_TELEMETRY_UART     SERIAL_PORT_USART2

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI
#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define M25P16_CS_PIN           SPI2_NSS_PIN
#define M25P16_SPI_INSTANCE     SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define VBAT_ADC_PIN                PB1
#define CURRENT_METER_ADC_PIN       PB0
#define ADC_INSTANCE                ADC3
#define VBAT_SCALE_DEFAULT          100

#define CURRENT_TARGET_CPU_VOLTAGE 3.0

// board uses an ina139, RL=0.005, Rs=30000
// V/A = (0.005 * 0.001 * 30000) * I
// rescale to 1/10th mV / A -> * 1000 * 10
// use 3.0V as cpu and adc voltage -> rescale by 3.0/3.3
#define CURRENT_METER_SCALE_DEFAULT    (0.005 * 0.001 * 30000) * 1000 * 10 * (CURRENT_TARGET_CPU_VOLTAGE / 3.3)
#define CURRENT_METER_OFFSET_DEFAULT   0

#define WS2811_PIN                      PA8
#define WS2811_TIMER                    TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY )
#define USE_TARGET_CONFIG

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 5
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(4) | TIM_N(8))
