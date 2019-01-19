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

#ifdef AG3XF4
#define TARGET_BOARD_IDENTIFIER "AGX4"
#define USBD_PRODUCT_STRING "Asgard32 F4"
#endif

#ifdef AG3XF7
#define TARGET_BOARD_IDENTIFIER "AGX7"
#define USBD_PRODUCT_STRING "Asgard32 F7"
#endif

#define ENABLE_DSHOT_DMAR       true

// Note, beeper is on the LED pin
#define LED0_PIN                PC13

#define USE_BEEPER
#define BEEPER_PIN              NONE
#define BEEPER_INVERTED

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define USE_MULTI_GYRO

#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA4
#define GYRO_1_EXTI_PIN         NONE

#define ACC_1_ALIGN             CW0_DEG_FLIP
#define GYRO_1_ALIGN            CW0_DEG_FLIP

#define GYRO_2_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           PC15
#define GYRO_2_EXTI_PIN         NONE

#define ACC_2_ALIGN             CW0_DEG_FLIP
#define GYRO_2_ALIGN            CW0_DEG_FLIP

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_LIS3MDL

#define USE_BARO
#define USE_BARO_SPI_BMP280
#define DEFAULT_BARO_SPI_BMP280
#define BARO_SPI_INSTANCE       SPI2
#define BARO_CS_PIN             PB9

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// Done, VBUS is set
#define USE_VCP
#define VBUS_SENSING_PIN        PB2

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            NONE // PA3, shared with CAMERA_CONTROL_PIN
#define UART2_TX_PIN            NONE // PA2, Labelled S/A, Used for LED strip

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1  // ESC Telemetry
#define UART4_TX_PIN            NONE // Not connected

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_TX_PIN      NONE // PA9 for TX1

#define USE_SOFTSERIAL2
#define SOFTSERIAL2_TX_PIN      NONE // PB14 for M7

#define SERIAL_PORT_COUNT       9 //VCP, UART1, UART2, UART3, UART4, UART5, UART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8  // PPM

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11
#define I2C_DEVICE              (I2CDEV_2)

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC0
#define RSSI_ADC_PIN            PC4
#define EXTERNAL1_ADC_PIN       PA0

#define USE_TRANSPONDER

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
//#define USE_RANGEFINDER_TF

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_SOFTSERIAL | FEATURE_ESC_SENSOR)

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ESC

#define TARGET_IO_PORTA (0xffff & ~(BIT(13)|BIT(14)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff)
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 15
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(12))
