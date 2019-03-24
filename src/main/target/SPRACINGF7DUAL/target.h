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

#define TARGET_BOARD_IDENTIFIER "SP7D"
#define USE_TARGET_CONFIG

#ifndef SPRACINGF7DUAL_REV
#define SPRACINGF7DUAL_REV 2
#endif

#define USBD_PRODUCT_STRING     "SP Racing F7 DUAL"

#define TEST_SOUND // for factory testing audio output

#define USE_MULTI_GYRO
//#define DEBUG_MODE DEBUG_DUAL_GYRO_DIFF

#define ENABLE_DSHOT_DMAR       true

#define LED0_PIN                PC4

#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define GYRO_2_EXTI_PIN         PC14

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500

#define USE_ACC
#define USE_ACC_SPI_MPU6500

#if (SPRACINGF7DUAL_REV >= 2)
#define GYRO_1_ALIGN        CW0_DEG
#define ACC_1_ALIGN         CW0_DEG

#define GYRO_2_ALIGN        CW270_DEG
#define ACC_2_ALIGN         CW270_DEG
#else
#define GYRO_1_ALIGN        CW180_DEG
#define ACC_1_ALIGN         CW180_DEG

#define GYRO_2_ALIGN        CW270_DEG
#define ACC_2_ALIGN         CW270_DEG
#endif

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define DEFAULT_BARO_BMP280

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_LIS3MDL

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

#if (SPRACINGF7DUAL_REV <= 1)
    #define TARGET_USART_CONFIG
#endif

// TODO
// #define USE_ESCSERIAL
// #define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_SPI

#define USE_SPI_DEVICE_1 // 2xMPU
#define SPI1_NSS_PIN            PA15
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2 // MAX7456
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

#define USE_SPI_DEVICE_3 // SDCARD
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PC3
#define SPI3_TX_DMA_OPT                     1  // DMA 1 Stream 7 Channel 0

#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI
#define VTX_RTC6705_OPTIONAL    // VTX/OSD board is OPTIONAL

#define RTC6705_SPI_MOSI_PIN                PB0  // Shared with PWM8
#define RTC6705_CS_PIN                      PB6  // Shared with PWM5
#define RTC6705_SPICLK_PIN                  PB1  // Shared with PWM7
#define RTC6705_POWER_PIN                   PB7  // Shared with PWM6

#define GYRO_1_CS_PIN                       SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE                 SPI1
#define GYRO_2_CS_PIN                       PB2
#define GYRO_2_SPI_INSTANCE                 SPI1

#define USE_ADC
// It's possible to use ADC1 or ADC3 on this target, same pins.
//#define ADC_INSTANCE                        ADC1
//#define ADC1_DMA_OPT                        0  // DMA 2 Stream 0 Channel 0 


// Using ADC3 frees up DMA2_Stream0 for SPI1_RX (not necessarily, SPI1_RX has DMA2_Stream2 as well)
#define ADC_INSTANCE                        ADC3
#define ADC3_DMA_OPT                        0  // DMA 2 Stream 0 Channel 2 


#define VBAT_ADC_PIN                        PC1
#define CURRENT_METER_ADC_PIN               PC2
#define RSSI_ADC_PIN                        PC0

#define CURRENT_METER_SCALE_DEFAULT         300

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_OSD
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_MSP_CURRENT_METER

#define USE_PID_AUDIO

#define USE_TRANSPONDER

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
//#define RX_CHANNELS_TAER
#define DEFAULT_RX_FEATURE                  FEATURE_RX_SERIAL
#define DEFAULT_FEATURES                    (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP)

#define SERIALRX_UART                       SERIAL_PORT_USART2
#define SERIALRX_PROVIDER                   SERIALRX_SBUS

#define SPEKTRUM_BIND_PIN       UART2_RX_PIN

#define USE_BUTTONS
#define BUTTON_A_PIN            UART5_RX_PIN
#define BUTTON_A_PIN_INVERTED

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

//#define USABLE_TIMER_CHANNEL_COUNT 17 // 4xPWM, 8xESC, 2xESC via UART3 RX/TX, 1xLED Strip, 1xIR, CAM_C
#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9))

