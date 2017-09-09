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

#define TARGET_BOARD_IDENTIFIER "SP4E"
#define TARGET_CONFIG

#ifndef SPRACINGF4EVO_REV
#define SPRACINGF4EVO_REV 2
#endif

#define USBD_PRODUCT_STRING     "SP Racing F4 EVO"

#define LED0                    PA0

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define INVERTER_PIN_UART2      PB2

#define USE_EXTI
#define MPU_INT_EXTI            PC13
#define USE_MPU_DATA_READY_SIGNAL

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH

#define GYRO
#define USE_GYRO_SPI_MPU6500

#define ACC
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6500_ALIGN       CW0_DEG
#define GYRO_MPU6500_ALIGN      CW0_DEG

#define BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883

#define USB_IO

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define SERIAL_PORT_COUNT       6

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define UART4_TX_PIN            PC10
#define UART4_RX_PIN            PC11

#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#if (SPRACINGF4EVO_REV >= 2)
    #define I2C1_SCL            PB8
    #define I2C1_SDA            PB9
#else
    #define I2C1_SCL            PB6
    #define I2C1_SDA            PB7
#endif

#define USE_SPI
#define USE_SPI_DEVICE_1 // MPU
#define USE_SPI_DEVICE_2 // SDCard
#define USE_SPI_DEVICE_3 // External

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_NSS_PIN            PA15 // NC
#define SPI3_SCK_PIN            PB3  // NC
#define SPI3_MISO_PIN           PB4  // NC
#define SPI3_MOSI_PIN           PB5  // NC

#define VTX_RTC6705
#define VTX_RTC6705_OPTIONAL    // SPI3 on an F4 EVO may be used for RTC6705 VTX control.

#define RTC6705_CS_PIN          SPI3_NSS_PIN
#define RTC6705_SPI_INSTANCE    SPI3

#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC14

#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF4
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

#define MPU6500_CS_PIN                      SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE                SPI1

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC0
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// PC4 - NC - Free for ADC12_IN14 / VTX CS
// PC5 - NC - Free for ADC12_IN15 / VTX Enable / OSD VSYNC

//#define OSD
//#define USE_MAX7456
//#define USE_OSD_OVER_MSP_DISPLAYPORT

#define LED_STRIP
#define WS2811_PIN                      PA1
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST6_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream6
#define WS2811_DMA_CHANNEL              DMA_Channel_3   // ???

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_LED_STRIP)
#define SERIALRX_UART           SERIAL_PORT_USART2
#define TELEMETRY_UART          SERIAL_PORT_USART5
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT  11
#define MAX_PWM_OUTPUT_PORTS        8

#if (SPRACINGF4NEO_REV >= 2)
#define USED_TIMERS  (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(9))
#else
#define USED_TIMERS  (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(9))
#endif

