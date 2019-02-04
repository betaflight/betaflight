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

#define TARGET_BOARD_IDENTIFIER "SIRF"

#define LED0_PIN                PB2
#define USE_BEEPER
#define BEEPER_PIN              PA1


#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA8
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

// MPU6000
#define ACC_1_ALIGN             CW180_DEG
#define GYRO_1_ALIGN            CW180_DEG

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

// MPU6500
//#define ACC_1_ALIGN             CW90_DEG // XXX Must be post-flash configured
//#define GYRO_1_ALIGN            CW90_DEG // XXX Must be post-flash configured

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB11  // (HARDARE=0,PPM)

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2 // PA14 / SWCLK
#define UART2_RX_PIN            PA3 // PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#undef  USE_UART1_RX_DMA
#undef  USE_UART1_TX_DMA

#undef  USE_I2C

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5
#define USE_SPI_DEVICE_3

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#define SPI3_TX_DMA_OPT         0 // DMA2_Channel2
#define SPI3_RX_DMA_OPT         0 // DMA2_Channel1

#define REMAP_TIM17_DMA

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_DMA_IRQ_HANDLER_ID          DMA2_CH1_HANDLER

#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI

#define RTC6705_SPI_MOSI_PIN    PC15
#define RTC6705_SPICLK_PIN      PC13
#define RTC6705_CS_PIN          PC14

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_GPIO                  SPI2_GPIO
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_SPI_DMA_OPT                  0    // DMA 1 Channel 5

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA3
#define RSSI_ADC_PIN            PA2

//#define USE_QUAD_MIXER_ONLY
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT


#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        (FEATURE_RX_SERIAL | FEATURE_OSD)

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(17))
