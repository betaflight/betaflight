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

// OMNIBUSF4FW for Version 2
// OMNIBUSF4FW1/OFW1 for Public Test Version
// (Not a valid target, .mk file must be supplied by a user)

#if defined(OMNIBUSF4FW)
#define TARGET_BOARD_IDENTIFIER "OBFW"
#define USBD_PRODUCT_STRING "OmnibusF4 Fireworks"
#elif defined(OMNIBUSF4FW1)
#define TARGET_BOARD_IDENTIFIER "OFW1"
#define USBD_PRODUCT_STRING "OmnibusF4 FWProto1"
#endif

#if defined(OMNIBUSF4FW)
#define LED0_PIN                PA8
#elif defined(OMNIBUSF4FW1)
#define LED0_PIN                PB5
#endif

#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define USE_DUAL_GYRO

#define GYRO_1_CS_PIN           PD2
#define GYRO_1_SPI_INSTANCE     SPI3
#define GYRO_2_CS_PIN           PA4
#define GYRO_2_SPI_INSTANCE     SPI1

#define GYRO_1_ALIGN            CW180_DEG
#define ACC_1_ALIGN             CW180_DEG
#define GYRO_2_ALIGN            CW0_DEG_FLIP
#define ACC_2_ALIGN             CW0_DEG_FLIP

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG

#define USE_BARO
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI3
#define BMP280_CS_PIN           PB3
#define DEFAULT_BARO_SPI_BMP280

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define FLASH_SPI_INSTANCE      SPI2
#define FLASH_CS_PIN            PB12

#define USE_VCP
#define VBUS_SENSING_PIN        PC5

#define USE_UART1
#define UART1_RX_PIN            PA10  // Inverted serial RX
#define UART1_TX_PIN            PA9   // Non-inverted Half-duplex (exclusive with serial RX

#define USE_UART2
#define UART2_RX_PIN            NONE
#define UART2_TX_PIN            PA2   // Half-duplex

#define USE_UART3
#define UART3_RX_PIN            PB11  // Shared with I2C2 (SDA)
#define UART3_TX_PIN            PB10  // Shared with I2C2 (SCL)

#define USE_UART4
#define UART4_RX_PIN            PA1   // Simplex RX, ESC telemetry
#define UART4_TX_PIN            NONE

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
//#define SOFTSERIAL1_TX_PIN      PA9   // SmartPort, shared with UART1_TX, doesn't work (timer collision)

#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       8 // VCP, UART1,2,3,4,6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8  // (PPM)

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

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
#define I2C2_SCL                NONE // PB10, shared with UART3TX
#define I2C2_SDA                NONE // PB11, shared with UART3RX
#define I2C_DEVICE              (I2CDEV_2)

#define CAMERA_CONTROL_PIN      PB9

#define USE_ADC
#define ADC_INSTANCE            ADC2
#define CURRENT_METER_ADC_PIN   PC1  // Direct from CRNT pad (part of onboard sensor for Pro)
#define VBAT_ADC_PIN            PC2  // 11:1 (10K + 1K) divider
#define RSSI_ADC_PIN            PA0  // Direct from RSSI pad

// Allegro Systems ACS781KLRTR-150U-T
#define CURRENT_METER_SCALE_DEFAULT  176
#define CURRENT_METER_OFFSET_DEFAULT -18500

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_TRANSPONDER

#define USE_SONAR

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

#define DEFAULT_FEATURES        (FEATURE_OSD)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)|BIT(13)))
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 15
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11))
