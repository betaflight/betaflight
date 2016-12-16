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

#define TARGET_BOARD_IDENTIFIER "FYF7"

#define CONFIG_START_FLASH_ADDRESS (0x080C0000)

#define USBD_PRODUCT_STRING "FuryF7"

#define USE_DSHOT
#define USE_ESC_SENSOR

#define LED0                    PB5
#define LED1                    PB4

#define BEEPER                  PD10
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define ICM20689_CS_PIN          PA4
#define ICM20689_SPI_INSTANCE    SPI1

#define GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_ICM20689_ALIGN      CW180_DEG

#define ACC
#define USE_ACC_SPI_ICM20689
#define ACC_ICM20689_ALIGN       CW180_DEG

//#define BARO
//#define USE_BARO_MS5611
//#define MS5611_I2C_INSTANCE     I2CDEV_1

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

//#define USE_SPI_DEVICE_2
//#define SPI2_NSS_PIN            PB12
//#define SPI2_SCK_PIN            PB13
//#define SPI2_MISO_PIN           PB14
//#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_SPI_DEVICE_4
#define SPI4_NSS_PIN            PE11
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14

#define USE_SDCARD
#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PD2
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line2
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource2
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOD
#define SDCARD_DETECT_EXTI_IRQn             EXTI2_IRQn

#define SDCARD_SPI_INSTANCE                 SPI4
#define SDCARD_SPI_CS_PIN                   SPI4_NSS_PIN

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 422kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 8 // 13.5MHz

#define SDCARD_DMA_CHANNEL_TX               DMA2_Stream1
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF1_5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA2
#define SDCARD_DMA_CHANNEL                  DMA_CHANNEL_4

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           PA15
#define M25P16_SPI_INSTANCE     SPI3

#define USE_VCP
#define VBUS_SENSING_PIN        PC5

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       4 //VCP, USART1, USART3, USART6

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)  // PB6-SCL, PB7-SDA
#define USE_I2C_PULLUP
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER
#define VBAT_ADC_PIN            PC1
#define RSSI_ADC_PIN            PC2
#define CURRENT_METER_ADC_PIN   PC3

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SENSORS_SET (SENSOR_ACC)

#define DEFAULT_FEATURES        (FEATURE_BLACKBOX)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define SPEKTRUM_BIND
// USART3 Rx, PB11
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 5
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(8))
