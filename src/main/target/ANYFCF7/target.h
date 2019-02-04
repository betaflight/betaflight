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

#define TARGET_BOARD_IDENTIFIER "ANY7"

#define USBD_PRODUCT_STRING "AnyFCF7"

#define LED0_PIN   PB7
#define LED1_PIN   PB6

#define USE_BEEPER
#define BEEPER_PIN PB2 // Unused pin, can be mapped to elsewhere
#define BEEPER_INVERTED

#define GYRO_1_CS_PIN         PA4
#define GYRO_1_SPI_INSTANCE   SPI1

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_1_ALIGN       CW270_DEG

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN       CW270_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN       PC4
#define USE_EXTI

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE           (I2CDEV_2)

//#define MAG_HMC5883_ALIGN CW270_DEG_FLIP
//#define MAG_HMC5883_ALIGN CW90_DEG

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define BARO_I2C_INSTANCE           (I2CDEV_2)

#define USABLE_TIMER_CHANNEL_COUNT 16

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN   PA8

#define USE_UART1
#define UART1_RX_PIN PA10
#define UART1_TX_PIN PA9

#define USE_UART2
#define UART2_RX_PIN PD6
#define UART2_TX_PIN PD5

#define USE_UART3
#define UART3_RX_PIN PD9
#define UART3_TX_PIN PD8

#define USE_UART4
#define UART4_RX_PIN PC11
#define UART4_TX_PIN PC10

#define USE_UART5
#define UART5_RX_PIN PD2
#define UART5_TX_PIN PC12

#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

#define USE_UART7
#define UART7_RX_PIN PE7
#define UART7_TX_PIN PE8

#define USE_UART8
#define UART8_RX_PIN PE0
#define UART8_TX_PIN PE1

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 11 //VCP, USART1, USART2, USART3, UART4, UART5, USART6, USART7, USART8, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB14 // (Hardware=0, PPM)

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI3_NSS_PIN            PD2
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define SPI4_NSS_PIN            PE11
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      SPI3_NSS_PIN
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PD3
#define SDCARD_SPI_INSTANCE                 SPI4
#define SDCARD_SPI_CS_PIN                   SPI4_NSS_PIN
#define SPI4_TX_DMA_OPT                     0     // DMA 2 Stream 1 Channel 4

#define USE_I2C
#define USE_I2C_DEVICE_2  // External I2C
#define USE_I2C_DEVICE_4  // Onboard I2C
#define I2C_DEVICE                  (I2CDEV_2)

#define USE_ADC
#define VBAT_ADC_PIN                PC0
#define CURRENT_METER_ADC_PIN       PC1
#define RSSI_ADC_PIN                PC2

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(12) )
