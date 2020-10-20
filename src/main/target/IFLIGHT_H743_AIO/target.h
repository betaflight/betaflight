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

#define TARGET_BOARD_IDENTIFIER "IF7A"
#define USBD_PRODUCT_STRING     "IFLIGHT_H743_AIO"

#define LED0_PIN                PC13

#define USE_BEEPER
#define BEEPER_PIN              PD15
#define BEEPER_INVERTED 

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART7
#define UART7_RX_PIN            PE7
#define UART7_TX_PIN            PE8

#define USE_UART8
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1

#define USE_VCP

#define SERIAL_PORT_COUNT       8

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define SPI1_NSS_PIN            PA4

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
#define SPI3_NSS_PIN            PA15

#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6
#define SPI4_NSS_PIN            PE4

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define I2C_DEVICE              (I2CDEV_1)

#define USE_MAG
#define USE_MAG_HMC5883

#define USE_BARO
#define USE_BARO_DPS310

#define USE_GYRO
#define USE_ACC
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PD0
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN           SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI1

#define GYRO_1_ALIGN            CW0_DEG 

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            SPI3_NSS_PIN
#define FLASH_SPI_INSTANCE      SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
 
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI4
#define MAX7456_SPI_CS_PIN      SPI4_NSS_PIN

#ifdef USE_DMA_SPEC
// #define UART1_TX_DMA_OPT         0
// #define UART2_TX_DMA_OPT         1
// #define UART3_TX_DMA_OPT         2
// #define UART4_TX_DMA_OPT         3
// #define UART5_TX_DMA_OPT         4
// #define UART6_TX_DMA_OPT         5
// #define UART7_TX_DMA_OPT         6
// #define UART8_TX_DMA_OPT         7
#define ADC1_DMA_OPT                8
#define ADC3_DMA_OPT                9
#else
#define ADC1_DMA_STREAM             DMA2_Stream0
#define ADC3_DMA_STREAM             DMA2_Stream1
#endif

#define USE_ADC
#define USE_ADC_INTERNAL

#define ADC1_INSTANCE               ADC1
#define ADC2_INSTANCE               ADC2
#define ADC3_INSTANCE               ADC3

#define RSSI_ADC_PIN                PC5
#define VBAT_ADC_PIN                PC3
#define CURRENT_METER_ADC_PIN       PC2

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define CURRENT_METER_SCALE_DEFAULT 200

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP)

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 7

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )
