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

#define TARGET_BOARD_IDENTIFIER "KISSV2"

#define USBD_PRODUCT_STRING "KISSFCV2F7"

#define LED0_PIN   PA8  // blue
#define LED1_PIN   PC8 // blingbling
#define LED1_INVERTED

#define USE_BEEPER
#define BEEPER_PIN              PC9
#define BEEPER_INVERTED

#define GYRO_1_CS_PIN           PB12
#define GYRO_1_SPI_INSTANCE     SPI2

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW90_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_1_ALIGN             CW90_DEG


#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15


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
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

//#define USE_UART6
//#define UART6_RX_PIN            PC7
//#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_TX_PIN      PB4
#define SOFTSERIAL1_RX_PIN      NONE //PB5

#define SERIAL_PORT_COUNT       6

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define AVOID_UART2_FOR_PWM_PPM

#define SPEKTRUM_BIND_PIN        UART2_RX_PIN

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define VBAT_SCALE_DEFAULT      160
#define VBAT_ADC_PIN            PB1

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 8

#define USED_TIMERS  ( TIM_N(2) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) )
