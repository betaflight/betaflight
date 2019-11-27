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
#define TARGET_BOARD_IDENTIFIER         "INFINITY"
#define USBD_PRODUCT_STRING             "INFINITY"

#define USE_TARGET_CONFIG

#define LED0_PIN                  PC0

#define USE_BEEPER
#define BEEPER_PIN              PB6
#define BEEPER_PWM_HZ             1000 // Beeper PWM frequency in Hz

#define INVERTER_PIN_UART1        PA10

// DEFINE SPI USAGE
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

// DEFINE GARO
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

//  MPU 6000
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_CS_PIN             PB12
#define GYRO_1_SPI_INSTANCE       SPI2
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN             CW90_DEG

//OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PA4
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

/*---------- VTX POWER SWITCH---------*/
#define USE_PINIO
#define PINIO1_PIN              PC5 // VTX power switcher
#define USE_PINIOBOX

// DEFINE UART AND VCP
#define USE_VCP

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

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART2,USART3,USART4, USART5,USART6

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1
#define SERIALMSP_UART          SERIAL_PORT_USART6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PD2  // (HARDARE=0,PPM)


#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_FLASH_W25M
#define USE_FLASH_W25M512
#define USE_FLASH_W25N01G
#define USE_FLASH_W25M02G
#define FLASH_CS_PIN            SPI3_NSS_PIN
#define FLASH_SPI_INSTANCE      SPI3

//DEFINE BATTERY METER 
#define USE_ADC
#define CURRENT_METER_ADC_PIN   PC3
#define VBAT_ADC_PIN            PC4
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE VOLTAGE_METER_ADC
#define USE_TRANSPONDER

// DEFINE DEFAULT FEATURE
#define DEFAULT_RX_FEATURE          FEATURE_RX_SERIAL
#define DEFAULT_FEATURES            (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_LED_STRIP)

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

// DEFINE TIMERS
#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS  (  TIM_N(1)  | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(11))
