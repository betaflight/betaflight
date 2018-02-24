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

// Removed to make the firmware fit into flash (in descending order of priority):
#undef USE_DSHOT_DMAR           // OMNIBUS (F3) does not benefit from burst Dshot
#undef USE_GYRO_OVERFLOW_CHECK
#define USE_GYRO_BIQUAD_RC_FIR2
#define USE_GYRO_FAST_KALMAN

#undef USE_SERIALRX_XBUS
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_MAVLINK

#undef USE_RTC_TIME
#undef USE_COPY_PROFILE_CMS_MENU
#undef USE_RX_MSP


#define TARGET_BOARD_IDENTIFIER "OMNI" // https://en.wikipedia.org/wiki/Omnibus

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define LED0_PIN                PB3

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI PC13
#define USE_MPU_DATA_READY_SIGNAL

#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6000_CS_PIN          PA4

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW90_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW90_DEG

#define BMP280_SPI_INSTANCE     SPI1
#define BMP280_CS_PIN           PA13

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280

//#define USE_RANGEFINDER
//#define USE_RANGEFINDER_HCSR04
//#define RANGEFINDER_HCSR04_ECHO_PIN          PB1
//#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB0

#define USB_DETECT_PIN          PB5

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (PWM5)
#define UART3_RX_PIN            PB11 // PB11 (PWM6)

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                NONE // PB6 (PWM8)
#define I2C1_SDA                NONE // PB7 (PWM7)
#define I2C_DEVICE              (I2CDEV_1)

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB4  // (HARDARE=0,PPM)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

// OSD define info:
//   feature name (includes source) -> MAX_OSD, used in target.mk

// include the max7456 driver
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB1
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)
//#define MAX7456_DMA_CHANNEL_TX            DMA1_Channel3
//#define MAX7456_DMA_CHANNEL_RX            DMA1_Channel2
//#define MAX7456_DMA_IRQ_HANDLER_ID        DMA1_CH3_HANDLER

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC14

#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

#define USE_ESC_SENSOR

// DSHOT output 4 uses DMA1_Channel5, so don't use it for the SDCARD until we find an alternative
#ifndef USE_DSHOT
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
#endif

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define VBAT_ADC_PIN                PA0
#define CURRENT_METER_ADC_PIN       PA1
#define ADC_INSTANCE                ADC1

//#define RSSI_ADC_PIN                PB1
//#define ADC_INSTANCE                ADC3

#define USE_TRANSPONDER
#define REDUCE_TRANSPONDER_CURRENT_DRAW_WHEN_USB_CABLE_PRESENT

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        (FEATURE_OSD)

// Disable rarely used buttons in favor of flash space
//#define USE_BUTTONS
//#define BUTTON_A_PIN            PB1
//#define BUTTON_B_PIN            PB0

//#define AVOID_UART3_FOR_PWM_PPM // Disable this for using UART3

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 8 // PPM + 6 Outputs (2 shared with UART3)
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(15))
