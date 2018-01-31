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

#define TARGET_BOARD_IDENTIFIER "BBV2" // BeeBrain V2.
#define USE_TARGET_CONFIG

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define USE_BRUSHED_ESC_AUTODETECT

#define LED0_PIN                PB1
#define LED1_PIN                PB2

#define USE_EXTI
#define MPU_INT_EXTI            PB6
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

#define SERIAL_PORT_COUNT       4

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3

#define USE_MSP_UART

#define AVOID_UART2_FOR_PWM_PPM

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_3

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define MPU6500_CS_PIN          PA15
#define MPU6500_SPI_INSTANCE    SPI3

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PA4

#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI
#define USE_VTX_CONTROL
#define RTC6705_SPI_MOSI_PIN    PC15
#define RTC6705_SPICLK_PIN      PC13
#define RTC6705_CS_PIN          PB12

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_NONE
#define ADC_INSTANCE            ADC3
#define VBAT_ADC_PIN            PB13

#define USE_TRANSPONDER
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER

#if defined(BEEBRAIN_V2D)
    // Receiver - DSM
    #define DEFAULT_FEATURES        (FEATURE_TRANSPONDER | FEATURE_MOTOR_STOP | FEATURE_OSD)
    #define SERIALRX_PROVIDER       SERIALRX_SPEKTRUM2048
#else
    // Receiver - Frsky
    #define DEFAULT_FEATURES        (FEATURE_TRANSPONDER | FEATURE_MOTOR_STOP | FEATURE_OSD | FEATURE_TELEMETRY)
    #define SERIALRX_PROVIDER       SERIALRX_SBUS
#endif

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 5
#define USED_TIMERS             ( TIM_N(1) | TIM_N(8) | TIM_N(15) | TIM_N(16) )
