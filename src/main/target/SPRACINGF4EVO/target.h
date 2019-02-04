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

#define TARGET_BOARD_IDENTIFIER "SP4E"
#define USE_TARGET_CONFIG

#ifndef SPRACINGF4EVO_REV
#define SPRACINGF4EVO_REV 2
#endif

#define USBD_PRODUCT_STRING     "SP Racing F4 EVO"

#define LED0_PIN                PA0

#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define INVERTER_PIN_UART2      PB2

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#ifdef SPRACINGF4EVODG
#define GYRO_2_EXTI_PIN         PC5 // GYRO 2 / NC on prototype boards, but if it was it'd be here.

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#endif

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH

#define USE_GYRO
// actual gyro is ICM20602 which is detected by the MPU6500 code
#define USE_GYRO_SPI_MPU6500

#define USE_ACC
#define USE_ACC_SPI_MPU6500

#ifndef SPRACINGF4EVODG
#define ACC_1_ALIGN                 CW0_DEG
#define GYRO_1_ALIGN                CW0_DEG
#else
#define GYRO_1_ALIGN                CW0_DEG
#define GYRO_2_ALIGN                CW0_DEG
#define ACC_1_ALIGN                 CW0_DEG
#define ACC_2_ALIGN                 CW0_DEG
#endif

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
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

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA3  // (HARDARE=0,PPM)

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#if (SPRACINGF4EVO_REV >= 2)
    #define I2C1_SCL                PB8
    #define I2C1_SDA                PB9
#else
    #define I2C1_SCL                PB6
    #define I2C1_SDA                PB7
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

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#if !defined(SPRACINGF4EVODG)
#define USE_VTX_RTC6705
#define VTX_RTC6705_OPTIONAL    // SPI3 on an F4 EVO may be used for RTC6705 VTX control.

#define RTC6705_CS_PIN          SPI3_NSS_PIN
#define RTC6705_SPI_INSTANCE    SPI3
#endif

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC14
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN
#define SPI2_TX_DMA_OPT                     0     // DMA 1 Stream 4 Channel 0

#ifndef SPRACINGF4EVODG
#define GYRO_1_CS_PIN           SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI1
#else
#define GYRO_1_CS_PIN           SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           SPI3_NSS_PIN
#define GYRO_2_SPI_INSTANCE     SPI3
#endif


#define USE_ADC
// It's possible to use ADC1 or ADC3 on this target, same pins.
//#define ADC_INSTANCE            ADC1
//#define ADC1_DMA_OPT    0  // DMA 2 Stream 0 Channel 0 


// Using ADC3 frees up DMA2_Stream0 for SPI1_RX
#define ADC_INSTANCE            ADC3
#define ADC3_DMA_OPT    1  // DMA 2 Stream 1 Channel 2 


#define VBAT_ADC_PIN            PC1
#define CURRENT_METER_ADC_PIN   PC2
#define RSSI_ADC_PIN            PC0

// PC3 - NC - Free for ADC12_IN13 / VTX Enable
// PC4 - NC - Free for ADC12_IN14 / VTX CS
// PC5 - NC - Free for ADC12_IN15 / OSD VSYNC / G2 MPU INT

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define USE_OSD
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_MSP_CURRENT_METER

#define USE_TRANSPONDER

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP)
#define SERIALRX_UART           SERIAL_PORT_USART2
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 16 // 4xPWM, 8xESC, 2xESC via UART3 RX/TX, 1xLED Strip, 1xIR.
#if (SPRACINGF4EVO_REV >= 2)
#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(9))
#else
#define USE_TIM10_TIM11_FOR_MOTORS
#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11))
#endif

