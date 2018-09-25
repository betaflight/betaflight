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

#define TARGET_BOARD_IDENTIFIER "RRF4"
#define USBD_PRODUCT_STRING     "Worm FC"

//LEDs
#define LED0_PIN                PA15
#define LED1_PIN                PC14

#define ENABLE_DSHOT_DMAR       true

//BEEPER
#define USE_BEEPER
#define BEEPER_PIN              PB14
#define BEEPER_INVERTED

// MPU6500 interrupt
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

// ACC section -- start
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define ACC_1_ALIGN             CW180_DEG_FLIP
// ACC section -- end

// GYRO section -- start
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_1_ALIGN            CW180_DEG_FLIP
// GYRO section -- end

//BARO
#define USE_BARO
#define USE_BARO_SPI_LPS
#define LPS_SPI_INSTANCE SPI3
#define LPS_CS_PIN PB8

//UARTs
#define INVERTER_PIN_UART6      PB13
#define INVERTER_PIN_UART3      PB12

#define USE_VCP
#define USB_DETECT_PIN          PA9
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART3, USART4, USART6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB9

//SPI
#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

//OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PC0
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define VBAT_ADC_PIN            PC1
//#define RSSI_ADC_PIN            PC2
#define CURRENT_METER_ADC_PIN   PC2

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

//SD CARD
#define USE_SDCARD
#define USE_SDCARD_SDIO

#define SDIO_DMA          DMA2_Stream3
#define SDCARD_SPI_CS_PIN NONE //This is not used on SDIO, has to be kept for now to keep compiler happy
#define SDCARD_DETECT_PIN PB15

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
