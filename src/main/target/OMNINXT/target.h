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
#define USE_HARDWARE_REVISION_DETECTION

#ifdef OMNINXT4
#define TARGET_BOARD_IDENTIFIER "ONX4"
#define USBD_PRODUCT_STRING "OmniNXT4"
#else
#define TARGET_BOARD_IDENTIFIER "ONX7"
#define USBD_PRODUCT_STRING "OmniNXT7"
#endif

#define LED0_PIN                PB2

#define USE_BEEPER
#define BEEPER_PIN              PC13
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       true

#ifdef OMNINXT4
#define USE_INVERTER
#define INVERTER_PIN_UART2      PC5
#endif

#define USE_ACC
#define USE_GYRO
// MPU interrupts
//#define USE_EXTI
//#define USE_GYRO_EXTI
//#define USE_MPU_DATA_READY_SIGNAL

// For debugging with NUC405RG
#define USE_FAKE_ACC
#define USE_FAKE_GYRO

#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000

#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500

#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PB12          // Onboard IMU
#define GYRO_1_ALIGN            CW0_DEG
#define ACC_1_ALIGN             CW0_DEG
#define GYRO_1_EXTI_PIN         NONE

#define GYRO_2_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           PA8           // External IMU
#define GYRO_2_ALIGN            CW270_DEG
#define ACC_2_ALIGN             CW270_DEG
#define GYRO_2_EXTI_PIN         NONE

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

#define USE_MAG
#define MAG_I2C_INSTANCE        (I2CDEV_1) 
#define USE_MAG_HMC5883
#define USE_MAG_LIS3MDL

#define USE_BARO
#define USE_BARO_SPI_LPS
#define BARO_SPI_INSTANCE       SPI2
#define BARO_CS_PIN             PA10
#define DEFAULT_BARO_SPI_LPS

#define BARO_I2C_INSTANCE       (I2CDEV_1)
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_SPI_INSTANCE      SPI2
#define FLASH_CS_PIN            PC14

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_VCP
#define VBUS_SENSING_PIN        PC15

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11          // PB11, shared with I2C2
#define UART3_TX_PIN            PB10          // PB10, shared with I2C2

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            NONE          // PC12, alt SPI3_MOSI

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       9             // VCP, UART x 6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB7

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PC3

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10          // PC10, alt UART3_TX, UART4_TX
#define SPI3_MISO_PIN           PC11          // PC11, alt UART3_RX, UART4_RX
#define SPI3_MOSI_PIN           PC12          // PC12, alt UART5_TX

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8           // PB8, alt MST8
#define I2C1_SDA                PB9           // PB9, alt MST7

#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE          // PB10 alt UART3TX
#define I2C2_SDA                NONE          // PB11 alt UART3RX

#define USE_I2C_DEVICE_3
#define I2C3_SCL                NONE
#define I2C3_SDA                NONE

#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            1  // DMA 2 Stream 4 Channel 0 (compat default)
#define ADC1_DMA_OPT            1  // DMA 2 Stream 4 Channel 0 (compat default)
#define VBAT_ADC_PIN            PC0  // 11:1 (10K + 1K) divider
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC4
#define EXTERNAL1_ADC_PIN       PA4

#define USE_TRANSPONDER

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define USE_RANGEFINDER_TF

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_LED_STRIP)

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13))) // Less SWC and SWD
#define TARGET_IO_PORTB (0xffff)
#define TARGET_IO_PORTC (0xffff)
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 21
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11) | TIM_N(12))
