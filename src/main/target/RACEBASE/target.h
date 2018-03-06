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

#define TARGET_BOARD_IDENTIFIER "RBFC"
#define USE_HARDWARE_REVISION_DETECTION
#define USE_TARGET_CONFIG

#define LED0_PIN                PB3
#define LED0_INVERTED

#define LED1_PIN                PB4
#define LED1_INVERTED

#define BEEPER                  PA12
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI            PC5
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW


#define MPU6000_CS_PIN          PB5
#define MPU6000_SPI_INSTANCE    SPI2

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000

#define USE_ACC
#define USE_ACC_SPI_MPU6000

#define ACC_MPU6000_ALIGN CW90_DEG
#define GYRO_MPU6000_ALIGN CW90_DEG

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA0  // (HARDARE=0,PPM)

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define SERIALRX_UART           SERIAL_PORT_USART2


#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PA7
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)


#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2
#define M25P16_SPI_SHARED
#define USE_FLASHFS
#define USE_FLASH_M25P16

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5
#define RSSI_ADC_PIN            PA6

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_RSSI_ADC | FEATURE_OSD)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(5))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#if defined(USE_UART3_RX_DMA) && defined(USE_DSHOT)
#undef USE_UART3_RX_DMA
#endif

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS  (TIM_N(2) | TIM_N(3)| TIM_N(4) | TIM_N(8) | TIM_N(17))
