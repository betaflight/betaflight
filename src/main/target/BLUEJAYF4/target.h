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
#define TARGET_BOARD_IDENTIFIER "BJF4"
#define USE_TARGET_CONFIG
#define TARGET_VALIDATECONFIG
#define TARGET_PREINIT

#define USBD_PRODUCT_STRING     "BlueJayF4"

#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN                  PB2

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define LED0_PIN                PB6
#define LED1_PIN                PB5
#define LED2_PIN                PB4

#define BEEPER                  PC1
#define BEEPER_OPT              PB7
#define BEEPER_INVERTED

#define INVERTER_PIN_UART6      PB15
//#define INVERTER_PIN_UART1     PC9

#define UART1_INVERTER          PC9

// MPU6500 interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PC5
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

#define MPU6500_CS_PIN          PC4
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW0_DEG

#define USE_GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW0_DEG

#define USE_MAG
#define USE_MAG_HMC5883
//#define USE_MAG_AK8963
#define HMC5883_I2C_INSTANCE    I2CDEV_1

#define USE_BARO
#define USE_BARO_MS5611
#define MS5611_I2C_INSTANCE     I2CDEV_1

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PD2
#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PA15

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
#define SDCARD_DMA_CHANNEL                  0

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define M25P16_CS_PIN           PB7
#define M25P16_SPI_INSTANCE     SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
//#define VBUS_SENSING_PIN PA8
//#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

// Provisioning for UART4 on motor outputs 1 & 2
// Keep pins NONE here to avoid UART4 showing up unless explicitly resource-mapped.
#define USE_UART4
#define UART4_RX_PIN            NONE // PA1
#define UART4_TX_PIN            NONE // PA0

#define USE_SOFTSERIAL1
// Since PB0 (motor 5) and PB1 (motor 6) are assigned with N-channels, these pin can not handle input.
// Therefore, receiving function of SOFTSERIAL1 can only be assigned to PB3 (DEBUG).
// Default defined here is to use DEBUG for half-duplex serial, suitable for VTX (SmartAudio or Tramp) controls.
// For non-half-duplex requirement (full-duplex or simplex in either direction), assign PB3 to RX and PB0 or PB1 to TX.
#define SOFTSERIAL1_RX_PIN      NONE
#define SOFTSERIAL1_TX_PIN      PB3  // DEBUG

#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       7

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PC7  // (HARDARE=0,PPM)

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB7
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define USE_I2C_PULLUP

#define USE_ADC
#define VBAT_ADC_PIN            PC3
#define CURRENT_METER_ADC_PIN   PC2

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA             0xffff
#define TARGET_IO_PORTB             0xffff
#define TARGET_IO_PORTC             0xffff
#define TARGET_IO_PORTD             (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT     8
#define USED_TIMERS                   ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(9) )
