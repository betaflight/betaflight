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

#define TARGET_BOARD_IDENTIFIER             "SP4N"
#define USE_TARGET_CONFIG

#ifndef SPRACINGF4NEO_REV
    #define SPRACINGF4NEO_REV 3
#endif

#define USBD_PRODUCT_STRING                 "SP Racing F4 NEO"

#if (SPRACINGF4NEO_REV >= 3)
    #define LED0_PIN                        PA0
    #define LED1_PIN                        PB1
#endif
#if (SPRACINGF4NEO_REV == 2)
    #define LED0_PIN                        PB9
    #define LED1_PIN                        PB2
#endif
#if (SPRACINGF4NEO_REV == 1)
    #define LED0_PIN                        PB9
    #define LED1_PIN                        PB2
#endif

#define BEEPER                              PC15
#define BEEPER_INVERTED

#if (SPRACINGF4NEO_REV >= 2)
    #define INVERTER_PIN_UART2              PB2
#else
    #define INVERTER_PIN_UART2              PA0
#endif

#define USE_EXTI
#define MPU_INT_EXTI                        PC13

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500

#define USE_ACC
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6500_ALIGN                   CW0_DEG
#define GYRO_MPU6500_ALIGN                  CW0_DEG

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define SERIAL_PORT_COUNT                   6

#define UART1_TX_PIN                        PA9
#define UART1_RX_PIN                        PA10

#define UART2_TX_PIN                        PA2
#define UART2_RX_PIN                        PA3

#define UART3_TX_PIN                        PB10
#define UART3_RX_PIN                        PB11

#define UART4_TX_PIN                        PC10
#define UART4_RX_PIN                        PC11

#define UART5_TX_PIN                        PC12
#define UART5_RX_PIN                        PD2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN              PA3  // (Hardware=0)

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE                          (I2CDEV_1) // PB6/SCL, PB7/SDA

#if (SPRACINGF4NEO_REV >= 3)
    #define I2C1_SCL                        PB8
    #define I2C1_SDA                        PB9
#else
    #define I2C1_SCL                        PB6
    #define I2C1_SDA                        PB7
#endif

#define USE_SPI
#define USE_SPI_DEVICE_1 // MPU
#define USE_SPI_DEVICE_2 // SDCard
#define USE_SPI_DEVICE_3 // External (MAX7456 & RTC6705)

#define SPI1_NSS_PIN                        PA4
#define SPI1_SCK_PIN                        PA5
#define SPI1_MISO_PIN                       PA6
#define SPI1_MOSI_PIN                       PA7

#define SPI2_NSS_PIN                        PB12
#define SPI2_SCK_PIN                        PB13
#define SPI2_MISO_PIN                       PB14
#define SPI2_MOSI_PIN                       PB15

#define SPI3_NSS_PIN                        PA15
#define SPI3_SCK_PIN                        PB3
#define SPI3_MISO_PIN                       PB4
#define SPI3_MOSI_PIN                       PB5

// Bus Switched Device, Device B.
#define USE_VTX_RTC6705
#define VTX_RTC6705_OPTIONAL    // VTX/OSD board is OPTIONAL

#define RTC6705_CS_PIN                      PC4
#define RTC6705_SPI_INSTANCE                SPI3
#define RTC6705_POWER_PIN                   PC3

#define USE_RTC6705_CLK_HACK
#define RTC6705_CLK_PIN                     SPI3_SCK_PIN

// Bus Switched Device, Device A.
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE                SPI3
#define MAX7456_SPI_CS_PIN                  PA15

//#define MAX7456_DMA_CHANNEL_TX              DMA1_Stream5
//#define MAX7456_DMA_CHANNEL_RX              DMA1_Stream0
//#define MAX7456_DMA_IRQ_HANDLER_ID          DMA1_ST0_HANDLER

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC14

#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// SPI3 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL                  0

#define MPU6500_CS_PIN                      SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE                SPI1

#define USE_ADC
#define ADC_INSTANCE                        ADC1
#define ADC1_DMA_STREAM                     DMA2_Stream0

#define VBAT_ADC_PIN                        PC1
#define CURRENT_METER_ADC_PIN               PC2
#define RSSI_ADC_PIN                        PC0

#define CURRENT_METER_SCALE_DEFAULT         300

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_TRANSPONDER

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE                  FEATURE_RX_SERIAL
#define DEFAULT_FEATURES                    (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_LED_STRIP)

#define GPS_UART                            SERIAL_PORT_USART3

#define SERIALRX_UART                       SERIAL_PORT_USART2
#define SERIALRX_PROVIDER                   SERIALRX_SBUS

#define TELEMETRY_UART                      SERIAL_PORT_UART5
#define TELEMETRY_PROVIDER_DEFAULT          FUNCTION_TELEMETRY_SMARTPORT

#define USE_BUTTONS // Physically located on the optional OSD/VTX board.
#if (SPRACINGF4NEO_REV >= 3)
    #define BUTTON_A_PIN                    PB0
#else
    #define BUTTON_A_PIN                    PB8
#endif

// FIXME While it's possible to use the button on the OSD/VTX board for binding enabling it here will break binding unless you have the OSD/VTX connected.
//#define BINDPLUG_PIN                        BUTTON_A_PIN

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA                     0xffff
#define TARGET_IO_PORTB                     0xffff
#define TARGET_IO_PORTC                     0xffff
#define TARGET_IO_PORTD                     (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT          14 // 4xPWM, 6xESC, 2xESC via UART3 RX/TX, 1xLED Strip, 1xIR.
#define USED_TIMERS                         (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(9))
