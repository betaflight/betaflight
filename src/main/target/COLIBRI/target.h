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

#define TARGET_BOARD_IDENTIFIER "COLI"

#define USBD_PRODUCT_STRING "Colibri"
#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000"
#endif

#define TARGET_XTAL_MHZ         16

#define LED0_PIN                PC14
#define LED1_PIN                PC13

#define USE_BEEPER
#define BEEPER_PIN              PC5

#define INVERTER_PIN_UART2      PB2 // PB2 used as inverter select GPIO

#define GYRO_1_CS_PIN           PC4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_1_ALIGN             CW270_DEG_FLIP

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW270_DEG_FLIP

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC0
#define USE_MPU_DATA_READY_SIGNAL

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW270_DEG_FLIP

#define MAG_INT_EXTI            PC1
#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH

#define USE_BARO
#define USE_BARO_MS5611

#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16


#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PA9

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

//#define USE_UART4
#define UART4_RX_PIN PC11
#define UART4_TX_PIN PC10

//#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6 //VCP, UART1, UART2, UART3, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA10  // (HARDARE=0,PPM)

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define USE_I2C
#define USE_I2C_DEVICE_3
#define I2C_DEVICE              (I2CDEV_3)
#define I2C3_SCL                PA8
#define I2C3_SDA                PC9

// alternative defaults for Colibri/Gemini target
#define USE_TARGET_CONFIG

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(10) | TIM_N(11))
