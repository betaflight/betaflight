/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define USE_TARGET_CONFIG

#if defined(OMNIBUSF4SD)
#define TARGET_BOARD_IDENTIFIER "OBSD"
#elif defined(OBF4NANOBB)
#define TARGET_BOARD_IDENTIFIER "NANO"
#elif defined(LUXF4OSD)
#define TARGET_BOARD_IDENTIFIER "LUX4"
#elif defined(DYSF4PRO)
#define TARGET_BOARD_IDENTIFIER "DYS4"
#elif defined(XRACERF4)
#define TARGET_BOARD_IDENTIFIER "XRF4"
#elif defined(EXUAVF4PRO)
#define TARGET_BOARD_IDENTIFIER "EXF4"
#else
#define TARGET_BOARD_IDENTIFIER "OBF4"
#define OMNIBUSF4BASE // For config.c
#endif

#if defined(LUXF4OSD)
#define USBD_PRODUCT_STRING "LuxF4osd"
#elif defined(DYSF4PRO)
#define USBD_PRODUCT_STRING "DysF4Pro"
#elif defined(XRACERF4)
#define USBD_PRODUCT_STRING "XRACERF4"
#elif defined(EXUAVF4PRO)
#define USBD_PRODUCT_STRING "ExuavF4Pro"
#elif defined(OBF4NANOBB)
#define USBD_PRODUCT_STRING "OBF4Nano"
#else
#define USBD_PRODUCT_STRING "OmnibusF4"
#endif

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000" // Remove this at the next major release (?)
#endif

#define LED0_PIN                PB5
//#define LED1_PIN                PB4 // Remove this at the next major release
#define BEEPER                  PB4
#define BEEPER_INVERTED

#if defined(OMNIBUSF4SD) || defined(DYSF4PRO)
#define ENABLE_DSHOT_DMAR       true
#endif

#ifdef OMNIBUSF4SD
// These inverter control pins collide with timer channels on CH5 and CH6 pads.
// Users of these timers/pads must un-map the inverter assignment explicitly.
#define INVERTER_PIN_UART6      PC8 // Omnibus F4 V3 and later
#define INVERTER_PIN_UART3      PC9 // Omnibus F4 Pro Corners
#elif defined(EXUAVF4PRO) || defined (OBF4NANOBB)
#define INVERTER_PIN_UART6      PC8
#else
#define INVERTER_PIN_UART1      PC0 // PC0 used as inverter select GPIO XXX this is not used --- remove it at the next major release
#endif

#define USE_ACC
#define USE_ACC_SPI_MPU6000

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#if defined(OMNIBUSF4SD) || defined(OBF4NANOBB)
#define GYRO_MPU6000_ALIGN       CW270_DEG
#define ACC_MPU6000_ALIGN        CW270_DEG
#elif defined(XRACERF4) || defined(EXUAVF4PRO)
#define GYRO_MPU6000_ALIGN       CW90_DEG
#define ACC_MPU6000_ALIGN        CW90_DEG
#else
#define GYRO_MPU6000_ALIGN       CW180_DEG
#define ACC_MPU6000_ALIGN        CW180_DEG
#endif

// Support for iFlight OMNIBUS F4 V3
// Has ICM20608 instead of MPU6000
// OMNIBUSF4SD is linked with both MPU6000 and MPU6500 drivers
#if defined (OMNIBUSF4SD)
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500
#define MPU6500_CS_PIN          MPU6000_CS_PIN
#define MPU6500_SPI_INSTANCE    MPU6000_SPI_INSTANCE
#define GYRO_MPU6500_ALIGN      GYRO_MPU6000_ALIGN
#define ACC_MPU6500_ALIGN       ACC_MPU6000_ALIGN
#endif

#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG

//#define USE_MAG_NAZA                   // Delete this on next major release
//#define MAG_NAZA_ALIGN CW180_DEG_FLIP  // Ditto

#define USE_BARO
#if defined(OMNIBUSF4SD) || defined(OBF4NANOBB)
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI3
#define BMP280_CS_PIN           PB3 // v1
#endif
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define BARO_I2C_INSTANCE       (I2CDEV_2)

#if defined(OMNIBUSF4SD) || defined(OBF4NANOBB)
#define DEFAULT_BARO_SPI_BMP280
#else
#define DEFAULT_BARO_BMP280
#endif

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#if defined(OMNIBUSF4SD)
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN               PB7
#define SDCARD_SPI_INSTANCE             SPI2
#define SDCARD_SPI_CS_PIN               SPI2_NSS_PIN
// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX                   DMA1_Stream4
#define SDCARD_DMA_CHANNEL                      0
#elif defined(LUXF4OSD) || defined(OBF4NANOBB)
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2
#define USE_FLASHFS
#define USE_FLASH_M25P16
#else
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN           SPI3_NSS_PIN
#define M25P16_SPI_INSTANCE     SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16
#endif // OMNIBUSF4

#define USE_VCP
#define VBUS_SENSING_PIN PC5

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#if defined(EXUAVF4PRO)
#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0
#endif

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#if defined(EXUAVF4PRO)
#define SERIAL_PORT_COUNT       7 // VCP, USART1, USART3, USART4, USART6, SOFTSERIAL x 2
#else
#define SERIAL_PORT_COUNT       6 // VCP, USART1, USART3, USART6, SOFTSERIAL x 2
#endif

#define USE_ESCSERIAL
#if defined(OMNIBUSF4SD)
#define ESCSERIAL_TIMER_TX_PIN  PB8  // (Hardware=0)
#else
#define ESCSERIAL_TIMER_TX_PIN  PB14 // (Hardware=0)
#endif

#define USE_SPI
#define USE_SPI_DEVICE_1

#if defined(OMNIBUSF4SD) || defined(LUXF4OSD) || defined(OBF4NANOBB)
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#endif

#define USE_SPI_DEVICE_3
#if defined(OMNIBUSF4SD)
  #define SPI3_NSS_PIN          PA15
#else
  #define SPI3_NSS_PIN          PB3
#endif
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PB10, shared with UART3TX
#define I2C2_SDA                NONE // PB11, shared with UART3RX
#if defined(OMNIBUSF4) || defined(OMNIBUSF4SD)
#define USE_I2C_DEVICE_3
#define I2C3_SCL                NONE // PA8, PWM6
#define I2C3_SDA                NONE // PC9, CH6
#endif
#define I2C_DEVICE              (I2CDEV_2)

#define USE_ADC
#define ADC_INSTANCE            ADC2
//#define ADC_INSTANCE            ADC1

#define CURRENT_METER_ADC_PIN   PC1  // Direct from CRNT pad (part of onboard sensor for Pro)
#define VBAT_ADC_PIN            PC2  // 11:1 (10K + 1K) divider
#ifdef DYSF4PRO
#define RSSI_ADC_PIN            PC3  // Direct from RSSI pad
#else
#define RSSI_ADC_PIN            PA0  // Direct from RSSI pad
#endif

#define USE_TRANSPONDER

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN     PA1
#define RANGEFINDER_HCSR04_ECHO_PIN        PA8
#define USE_RANGEFINDER_TF

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

#define DEFAULT_FEATURES        (FEATURE_OSD)

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)|BIT(13)))
#define TARGET_IO_PORTD BIT(2)

#if defined(OMNIBUSF4SD) || defined(EXUAVF4PRO) || defined(OBF4NANOBB)
#define USABLE_TIMER_CHANNEL_COUNT 15
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(10) | TIM_N(12) | TIM_N(8) | TIM_N(9))
#else
#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9))
#endif
