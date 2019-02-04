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

#define TARGET_BOARD_IDENTIFIER "CLR4"

#define USBD_PRODUCT_STRING "CLRACINGF4"

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000" // Remove this at the next major release (?)
#endif

#define LED0_PIN                  PB5
#define USE_BEEPER
#define BEEPER_PIN                PB4
#define BEEPER_INVERTED
#define BEEPER_PWM_HZ             3800 // Beeper PWM frequency in Hz

#define ENABLE_DSHOT_DMAR         false // Motors 3 / 4 conflict with LED_STRIP if enabled

#define INVERTER_PIN_UART1        PC0 // PC0 used as inverter select GPIO

#define CAMERA_CONTROL_PIN        PB9    // define dedicated camera_osd_control pin


#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN           PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_ACC
#define USE_GYRO

//  MPU 6000
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000

// ICM-20602
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500

#define GYRO_1_CS_PIN             PA4
#define GYRO_1_SPI_INSTANCE       SPI1
#define GYRO_1_ALIGN              CW0_DEG
#define ACC_1_ALIGN               CW0_DEG

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_PIN             PB7
#define SDCARD_SPI_INSTANCE           SPI2
#define SDCARD_SPI_CS_PIN             SPI2_NSS_PIN
#define SPI2_TX_DMA_OPT                      0     // DMA 1 Stream 4 Channel 0

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3


#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN   PC5

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3,USART4, USART6, SOFT_SERIAL1

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB9  // (HARDARE=0,PPM)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2
#define RSSI_ADC_PIN            PC3
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_TRANSPONDER

#define DEFAULT_RX_FEATURE          FEATURE_RX_SERIAL
#define DEFAULT_FEATURES            ( FEATURE_OSD  )
#define CURRENT_METER_SCALE_DEFAULT 250

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)|BIT(13)))
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS  (  TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(11))
