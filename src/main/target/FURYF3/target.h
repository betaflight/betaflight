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

#ifdef FURYF3OSD
    #define TARGET_BOARD_IDENTIFIER "FY3O"
//    #define USBD_PRODUCT_STRING     "FuryF3OSD"
#else
    #define TARGET_BOARD_IDENTIFIER "FYF3"
//    #define USBD_PRODUCT_STRING     "FuryF3"
    #undef USE_OSD
#endif

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT
#define CONFIG_PREFER_ACC_ON

#define LED0_PIN                PC14

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI            PA3
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define ICM20689_CS_PIN          PA4
#define ICM20689_SPI_INSTANCE    SPI1

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_ICM20689_ALIGN      CW180_DEG

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG

#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW90_DEG

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define ACC_ICM20689_ALIGN       CW180_DEG

#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW90_DEG

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#ifdef FURYF3OSD
    // include the max7456 driver
    #define USE_MAX7456
    #define MAX7456_SPI_INSTANCE    SPI1
    #define MAX7456_SPI_CS_PIN      PC13
    #define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
    #define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

    #define USE_FLASHFS
    #define USE_FLASH_M25P16
    #define M25P16_CS_PIN           PB12
    #define M25P16_SPI_INSTANCE     SPI2

    #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

    #define DEFAULT_FEATURES        (FEATURE_OSD)
    #define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#else

    #define USE_SDCARD

    #define SDCARD_DETECT_INVERTED

    #define SDCARD_DETECT_PIN                   PB2
    #define SDCARD_SPI_INSTANCE                 SPI2
    #define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

    // SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
    #define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
    // Divide to under 25MHz for normal operation:
    #define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

    // Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
    #define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5

    // Performance logging for SD card operations:
    // #define AFATFS_USE_INTROSPECTIVE_LOGGING

    #define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

    #define USE_BARO
    #define USE_BARO_MS5611

#endif

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1) // SDA (PB9/AF4), SCL (PB8/AF4)
#define I2C1_SCL            PB8
#define I2C1_SDA            PB9

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 6

#define SOFTSERIAL1_RX_PIN      PB0
#define SOFTSERIAL1_TX_PIN      PB1

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB3  // (HARDARE=0,PPM)

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define RSSI_ADC_PIN            PA1
#define CURRENT_METER_ADC_PIN   PA2

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_ESC_SENSOR
#define REMAP_TIM17_DMA

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16) | TIM_N(17))
