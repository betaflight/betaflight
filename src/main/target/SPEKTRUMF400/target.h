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

#define TARGET_BOARD_IDENTIFIER "SKF4"

#define USBD_PRODUCT_STRING "SpektrumF400"

#define LED0_PIN                PA15
#define LED1_PIN                PC8

#define USE_BEEPER
#define BEEPER_PIN              PC2
#define BEEPER_INVERTED

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN PA4
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN PA6
#define SPI1_MOSI_PIN PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN PB12
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

//#define FLASH_CS_PIN            PB12
//#define FLASH_SPI_INSTANCE      SPI2

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250
#define GYRO_1_ALIGN       CW270_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU9250
#define ACC_1_ALIGN       CW270_DEG

#define GYRO_1_CS_PIN           PB12
#define GYRO_1_SPI_INSTANCE     SPI2

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
//#define UART5_RX_PIN PC12     // XXX ???
#define UART5_TX_PIN            PC12

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       8

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8

#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PB10, Shared with UART3_TX
#define I2C2_SDA                NONE // PB11, Shared with UART3_RX

#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            1  // DMA 2 Stream 4 Channel 0 (compat default)
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC3

#define TARGET_XTAL_MHZ         12

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(5)|TIM_N(8)|TIM_N(9))
