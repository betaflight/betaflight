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
#define TARGET_BOARD_IDENTIFIER "YPF4"
#define USE_TARGET_CONFIG

#define USBD_PRODUCT_STRING     "YupiF4"

#define USE_HARDWARE_REVISION_DETECTION

#define LED0_PIN                PB6
#define LED1_PIN                PB4
#define LED2_PIN                PB5

#define USE_BEEPER
#define BEEPER_PIN              PC9
#define BEEPER_PWM_HZ           3150 // Beeper PWM frequency in Hz

// Gyro interrupt
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define USE_ACC_SPI_MPU6500
#define ACC_1_ALIGN             CW90_DEG

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_GYRO_SPI_MPU6500
#define GYRO_1_ALIGN            CW90_DEG

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_VCP

// UART Ports
#define USE_UART1
#define INVERTER_PIN_UART1      PB12
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define INVERTER_PIN_UART6      PB15
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_RX_PIN      PB1 // PWM7
#define SOFTSERIAL1_TX_PIN      PB0 // PWM5

#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6 // VCP, UART1, UART3, UART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PC8  // (HARDARE=0,PPM)

// SD Card
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN               PD2
#define SDCARD_SPI_INSTANCE             SPI3
#define SDCARD_SPI_CS_PIN               PA15
#define SPI3_TX_DMA_OPT                     0     // DMA 1 Stream 5 Channel 0

// SPI Ports
#define USE_SPI

#define USE_SPI_DEVICE_1 //Gyro
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3 //dataslash - SD Card
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

// OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PA14
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

// ADC inputs
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define USE_ADC
#define RSSI_ADC_PIN            PC0
#define VBAT_ADC_PIN            PC1
#define CURRENT_METER_ADC_PIN   PC2
#define CURRENT_METER_SCALE_DEFAULT 150

// Default configuration
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define DEFAULT_FEATURES        (FEATURE_OSD)
#define USE_GYRO_FAST_KALMAN

// Target IO and timers
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS             (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12))
