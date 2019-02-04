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

//Target code By BorisB and Hector "Hectech FPV" Hind

#pragma once

#define TARGET_BOARD_IDENTIFIER "BFF3"

#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       true

#define USABLE_TIMER_CHANNEL_COUNT 10

#define GYRO_1_CS_PIN           PA15
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW180_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_1_ALIGN             CW180_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN             PC13
#define USE_EXTI

#define REMAP_TIM16_DMA
#define REMAP_TIM17_DMA

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB7  // (HARDARE=0,PPM)

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define SOFTSERIAL1_RX_PIN      PB0 // PWM 5
#define SOFTSERIAL1_TX_PIN      PB1 // PWM 6

#undef USE_I2C

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI1_NSS_PIN            PA15
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

// include the max7456 driver
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PA1
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC14
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC24_DMA_REMAP
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5
#define RSSI_ADC_PIN            PB2

#define CURRENT_METER_SCALE_DEFAULT 220

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2
#define SBUS_TELEMETRY_UART     SERIAL_PORT_USART1
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD)

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16) | TIM_N(17) )
