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

#if defined(RADIANCED)
#define TARGET_BOARD_IDENTIFIER "RADD" // Furious FPV RADIANCE DSHOT 
#elif defined(RADIANCE)
#define TARGET_BOARD_IDENTIFIER "RADI" // Furious FPV RADIANCE V1 
#elif defined(KOMBINID)
#define TARGET_BOARD_IDENTIFIER "KOMD" // Furious FPV KOMBINI DSHOT 
#elif defined(KOMBINI)
#define TARGET_BOARD_IDENTIFIER "KOMB" // Furious FPV KOMBINI V1 
#elif defined(RACEWHOOP)
#define TARGET_BOARD_IDENTIFIER "RWHO" // Furious FPV RACEWHOOP
#elif defined(ACROWHOOPFR)
#define TARGET_BOARD_IDENTIFIER "AWHF" // Furious FPV ACROWHOOP FRSKY
#elif defined(ACROWHOOPSP)
#define TARGET_BOARD_IDENTIFIER "AWHS" // Furious FPV ACROWHOOP SPEKTRUM
#elif defined(NUKE)
#define TARGET_BOARD_IDENTIFIER "NUKE" // Furious FPV NUKE 
#else 
#define TARGET_BOARD_IDENTIFIER "PIKO" // Furious FPV PIKOBLX
#endif

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define TARGET_CONFIG

#if defined(ACROWHOOPFR) || defined(ACROWHOOPSP) || defined(NUKE)
#define BRUSHED_MOTORS
#define RX_CHANNELS_TAER
#endif

#if defined(RACEWHOOP)
#define RX_CHANNELS_TAER
#endif

#define LED0                    PB9
#define LED1                    PB5

#define BEEPER                  PA0
#define BEEPER_INVERTED

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PA15
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define MPU6000_CS_GPIO         GPIOB
#define MPU6000_CS_PIN          PB12
#define MPU6000_SPI_INSTANCE    SPI2

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

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


#if defined(KOMBINI) || defined(KOMBINID)
#define CURRENT_METER_SCALE_DEFAULT 125
#elif defined(RACEWHOOP)
#define CURRENT_METER_SCALE_DEFAULT 1000
#endif

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#if defined(RADIANCE) || defined(RADIANCED)
#define SPEKTRUM_BIND_PIN       UART2_RX_PIN
#else
#define SPEKTRUM_BIND_PIN       UART3_RX_PIN
#endif

#if !(defined(NUKE) || defined(ACROWHOPFR) || defined(ACROWHOPSP) || defined(RACEWHOOP))
#define TRANSPONDER
#endif

#if !defined(BRUSHED_MOTORS)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(16) | TIM_N(17))
