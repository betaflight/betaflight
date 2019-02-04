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

#if defined(STACKX)
#define TARGET_BOARD_IDENTIFIER "SXF4"
#define USBD_PRODUCT_STRING     "Stack-X F4"
#else
#define TARGET_BOARD_IDENTIFIER "FDF4"
#define USBD_PRODUCT_STRING     "FishDroneF4"
#endif

#define LED0_PIN                PC13
#define LED1_PIN                PC14

#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define INVERTER_PIN_UART6      PC8

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_1_ALIGN            CW90_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define ACC_1_ALIGN             CW90_DEG

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PA8
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6 // VCP, USART1, USART3, USART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB0  // (HARDARE=0,PPM)

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

// *************** FLASH *****************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_FLASH_M25P16
#define USE_FLASHFS
#define FLASH_CS_PIN            PD2
#define FLASH_SPI_INSTANCE      SPI3

// *************** SDCARD *****************************
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PB7
#define SDCARD_SPI_INSTANCE     SPI3
#define SDCARD_SPI_CS_PIN       PB9
#define SPI3_TX_DMA_OPT                     0     // DMA 1 Stream 5 Channel 0

// *************** RTC6705 *************************
#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI
#define RTC6705_SPICLK_PIN      PB4
#define RTC6705_SPI_MOSI_PIN    PB5
#define RTC6705_CS_PIN          PB3

// *************** ADC *****************************
#define USE_ADC
#define VBAT_ADC_PIN            PC0
#define RSSI_ADC_PIN            PC1
#if defined(STACKX)
#define CURRENT_METER_ADC_PIN   PA1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#endif

// *************** FEATURES ************************
#define DEFAULT_FEATURES        (FEATURE_OSD)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

// *************** Others **************************
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) )
