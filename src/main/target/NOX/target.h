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

#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "NOX1"
#define USBD_PRODUCT_STRING "NoxF4V1"

#define LED0_PIN                PA4

#define USE_BEEPER
#define BEEPER_PIN              PC13
#define BEEPER_INVERTED

#define INVERTER_PIN_UART2      PC14

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU6000

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU6000

#define GYRO_1_CS_PIN           PB12
#define GYRO_1_SPI_INSTANCE     SPI2

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA8
#define USE_MPU_DATA_READY_SIGNAL

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BARO_SPI_INSTANCE       SPI2
#define BARO_CS_PIN             PA9

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PA10

#define USE_VCP
#define USE_USB_DETECT
//#define USB_DETECT_PIN   PC5

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_RX_PIN      PA2    // Backdoor timer on UART2_TX, used for ESC telemetry
#define SOFTSERIAL1_TX_PIN      PA2    // Workaround for softserial not initializing with only RX

#define USE_SOFTSERIAL2
#define SOFTSERIAL2_RX_PIN      NONE
#define SOFTSERIAL2_TX_PIN      NONE

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART2, SOFTSERIAL1, SOFTSERIAL2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB10

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_SPI_INSTANCE      SPI1
#define FLASH_CS_PIN            PA15
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_ADC
#define CURRENT_METER_ADC_PIN   NONE // PA6 Available from TP33
#define VBAT_ADC_PIN            PA5  // 11:1 (10K + 1K) divider

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ESC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ESC

#define SERIALRX_UART           SERIAL_PORT_USART2

#define USE_TRANSPONDER

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_SOFTSERIAL | FEATURE_ESC_SENSOR)

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)|BIT(11)))
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(9) )
