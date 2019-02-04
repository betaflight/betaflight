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

#define TARGET_BOARD_IDENTIFIER "BFF4"
#define USBD_PRODUCT_STRING     "BetaFlightF4"

#define LED0_PIN                PB5

#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

// PC13 used as inverter select GPIO for UART2
#define INVERTER_PIN_UART2      PC13

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_1_ALIGN            CW180_DEG

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define ACC_1_ALIGN             CW180_DEG



// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define FLASH_CS_PIN            PA15
#define FLASH_SPI_INSTANCE      SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PC5


#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2


#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART2, USART3, USART6, SOFTSERIAL1, SOFTSERIAL2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8 // (Hardware=0, PPM)

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

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PB10, UART3_TX
#define I2C2_SDA                NONE // PB11, UART3_RX
#define I2C_DEVICE              (I2CDEV_2)

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define SBUS_TELEMETRY_UART     SERIAL_PORT_USART2

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD )

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)))
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 12

#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(10) )
