/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "MKF4"
#define USBD_PRODUCT_STRING  "MatekF4"

#define LED0_PIN                PB9
#define LED1_PIN                PA14

#define BEEPER                  PC13
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       true

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define MPU6000_CS_PIN          PC2
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU6500_CS_PIN          PC2
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_EXTI
#define MPU_INT_EXTI            PC3
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG

#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW180_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG

#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

// *************** Baro **************************
#define USE_I2C

#if defined(BARO_I2C1)
// Useful for MATEKF405_OSD, since it does not have the SCL / SDA pads
#define USE_I2C_DEVICE_3

#define I2C_DEVICE              (I2CDEV_3)
#define I2C3_SCL                PC9        // S4 pad
#define I2C3_SDA                PA8        // S6 pad
#define BARO_I2C_INSTANCE       (I2CDEV_3)
#else
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6        // SCL pad
#define I2C1_SDA                PB7        // SDA pad
#define BARO_I2C_INSTANCE       (I2CDEV_1)
#endif

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           PC0
#define M25P16_SPI_INSTANCE     SPI3

// *************** SD Card **************************
#define USE_SDCARD
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SDCARD_SPI_INSTANCE     SPI3
#define SDCARD_SPI_CS_PIN       PC1

// SPI3 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream7
#define SDCARD_DMA_CHANNEL                  0

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB10

// *************** UART *****************************
#define USE_VCP
#define VBUS_SENSING_PIN        PB12
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       8

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

// *************** ADC *****************************
#define USE_ADC
#define ADC1_DMA_STREAM         DMA2_Stream0
#define VBAT_ADC_PIN            PC5
#define CURRENT_METER_ADC_PIN   PC4
#define RSSI_ADC_PIN            PB1

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 179

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN PA3
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(5)|TIM_N(8)|TIM_N(9))
