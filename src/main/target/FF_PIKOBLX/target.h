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

#if defined(FF_RADIANCE)
#define TARGET_BOARD_IDENTIFIER "RADI" // Furious FPV RADIANCE
#elif defined(FF_KOMBINI)
#define TARGET_BOARD_IDENTIFIER "KOMB" // Furious FPV KOMBINI
#elif defined(FF_ACROWHOOPSP)
#define TARGET_BOARD_IDENTIFIER "AWHS" // Furious FPV ACROWHOOP SPEKTRUM
#else
#define TARGET_BOARD_IDENTIFIER "PIKO" // Furious FPV PIKOBLX
#endif

#define ENABLE_DSHOT_DMAR       true
#define REMAP_TIM16_DMA


#define USE_TARGET_CONFIG

#define LED0_PIN                PB9
#define LED1_PIN                PB5

#define USE_BEEPER
#define BEEPER_PIN              PA0
#define BEEPER_INVERTED

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA15
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW180_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_1_ALIGN             CW180_DEG

#define GYRO_1_CS_PIN           PB12
#define GYRO_1_SPI_INSTANCE     SPI2

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL
#if defined(FF_RADIANCE) || defined(FF_KOMBINI)
#define ESCSERIAL_TIMER_TX_PIN  PA7  // (Hardware=0)
#else
#define ESCSERIAL_TIMER_TX_PIN  PA4  // (Hardware=0)
#endif

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PB3
#define UART2_RX_PIN            PB4

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_SPI
#define USE_SPI_DEVICE_2

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define ADC_INSTANCE            ADC2
#define CURRENT_METER_ADC_PIN   PB2
#define VBAT_ADC_PIN            PA5


#if defined(FF_KOMBINI)
#define CURRENT_METER_SCALE_DEFAULT 125
#endif

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define USE_TRANSPONDER

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(16) | TIM_N(17))
