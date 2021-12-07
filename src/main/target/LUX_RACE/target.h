/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef LUXV2_RACE
#define TARGET_BOARD_IDENTIFIER "LUXR"
#else
#define TARGET_BOARD_IDENTIFIER "LUX"
#endif

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC


#define LED0_PIN                PC15
#define LED1_PIN                PC14
#ifndef LUXV2_RACE
#define LED2_PIN                PC13
#endif

#ifdef LUXV2_RACE
#define USE_BEEPER
#define BEEPER_PIN              PB9
#else
#define USE_BEEPER
#define BEEPER_PIN              PB13
#endif
#define BEEPER_INVERTED

// MPU6500 interrupt
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA5
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_SPI
#define USE_SPI_DEVICE_1
#ifdef LUXV2_RACE
#define USE_SPI_DEVICE_2
#endif

#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5
//#ifndef LUXV2_RACE
#define SPI1_NSS_PIN            PA4
//#endif

#ifdef LUXV2_RACE
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC13
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_SPI_DMA_OPT                  0    // DMA 1 Channel 5
#endif

#define GYRO_1_CS_PIN           SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_GYRO
#define GYRO_1_ALIGN       CW270_DEG
#ifdef LUXV2_RACE
#define USE_GYRO_SPI_MPU6000
#else
#define USE_GYRO_SPI_MPU6500
#endif

#define USE_ACC
#ifdef LUXV2_RACE
#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
#else
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#endif

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#ifdef LUXV2_RACE
#  define USE_UART4
#  define USE_UART5
#  define SERIAL_PORT_COUNT       8
#else
#  define SERIAL_PORT_COUNT       6
#endif

#define UART1_TX_PIN            PC4
#define UART1_RX_PIN            PC5

#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#undef USE_I2C

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2
#define SBUS_TELEMETRY_UART     SERIAL_PORT_USART1

#ifdef LUXV2_RACE
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY)
#else
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY)
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8  // (HARDARE=0,PPM)

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#ifdef LUXV2_RACE
#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             (TIM_N(1) | TIM_N(3) | TIM_N(8) | TIM_N(16))
#else
#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(15) | TIM_N(16))
#endif
