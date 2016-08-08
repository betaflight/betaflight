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

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define LED0                    PB3

#define BEEPER                  PC15

#define USABLE_TIMER_CHANNEL_COUNT 10

#define USE_EXTI
#define MPU_INT_EXTI            PC13
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW0_DEG_FLIP

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW0_DEG_FLIP

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
#define USE_UART1          // JST-SH Serial - TX (PA9) RX (PA10)
#define USE_UART2          // Input - TX (NC) RX (PA15)
#define USE_UART3          // Solder Pads - TX (PB10) RX (PB11)
#define USE_SOFTSERIAL1     // Telemetry
#define SERIAL_PORT_COUNT 5

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14 //Not connected
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11


#define SOFTSERIAL_1_TIMER      TIM15
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 7 //Not connected
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 8

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1) // PB6/SCL, PB7/SDA

#define USE_SPI
#define USE_SPI_DEVICE_1 // PA4, 5, 6, 7
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define VTX
#define RTC6705_CS_GPIO         GPIOA
#define RTC6705_CS_PIN          GPIO_Pin_4
#define RTC6705_SPI_INSTANCE    SPI1

#define M25P16_CS_GPIO          GPIOB
#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_ADC

#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PB2
#define VBAT_SCALE_DEFAULT      77

#define LED_STRIP

#define USE_LED_STRIP_ON_DMA1_CHANNEL2
#define WS2811_PIN                      PA8
#define WS2811_TIMER                    TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_FEATURES        (FEATURE_VBAT | FEATURE_BLACKBOX)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART3

#define SPEKTRUM_BIND
// USART2, PA15
#define BIND_PIN                PA15

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
//#define TARGET_IO_PORTF (BIT(0)|BIT(1))
// !!TODO - check the following line is correct
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(16) |TIM_N(17))

