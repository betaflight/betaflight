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

#define TARGET_BOARD_IDENTIFIER "KTH7"
#define USBD_PRODUCT_STRING     "KakuteH7"

#define LED0_PIN                PC2

#define USE_BEEPER
#define BEEPER_PIN              PC13
#define BEEPER_INVERTED

#define USE_ACC
#define USE_GYRO
#define USE_EXTI

// MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000

// ICM-20689
#define USE_ACC_SPI_ICM20689
#define USE_GYRO_SPI_ICM20689
#define GYRO_1_CS_PIN           SPI4_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI4
#define GYRO_1_ALIGN            CW270_DEG
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PE1
#define USE_MPU_DATA_READY_SIGNAL

//USB
#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PA8

//UART
#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10
#define USE_UART2
#define UART2_TX_PIN            PD5
#define UART2_RX_PIN            PD6
#define USE_UART3
#define UART3_TX_PIN            PD8
#define UART3_RX_PIN            PD9
#define USE_UART4
#define UART4_TX_PIN            PD1
#define UART4_RX_PIN            PD0
#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7
#define USE_UART7
#define UART7_TX_PIN            NONE
#define UART7_RX_PIN            PE7
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 9 //VCP,UART1,UART2,UART3,UAER4,UART6,UART7

//SPI
#define USE_SPI
#define USE_SPI_DEVICE_1   //SD Card
#define USE_SPI_DEVICE_2   //OSD
#define USE_SPI_DEVICE_4   //IMU
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define SPI4_NSS_PIN            PE4
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6

//OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

//SD
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PA3
#define SDCARD_SPI_INSTANCE                 SPI1
#define SDCARD_SPI_CS_PIN                   SPI1_NSS_PIN
#define SPI1_TX_DMA_OPT                     13
#define SDCARD_DMA_STREAM_TX_FULL           DMA2_Stream5

//I2C
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7
#define USE_BARO
#define USE_BARO_BMP280
#define BARO_I2C_INSTANCE     I2C_DEVICE
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define MAG_I2C_INSTANCE      I2C_DEVICE

//ADC
#define USE_DMA
#define ADC1_DMA_OPT            8
#define ADC3_DMA_OPT            9
#define USE_ADC
#define USE_ADC_INTERNAL     //ADC3
#define ADC1_INSTANCE           ADC1
#define ADC3_INSTANCE           ADC3
#define VBAT_ADC_PIN            PC0   //ADC123    VBAT
#define CURRENT_METER_ADC_PIN   PC1   //ADC123    CURR
#define RSSI_ADC_PIN            PC5   //ADC12      RSSI
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define VBAT_SCALE_DEFAULT            110
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT   168

//PINIO
#define USE_PINIO
#define PINIO1_PIN              PE13  // BlueTooth power switch
#define USE_PINIOBOX
#define USE_TARGET_CONFIG
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART6
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define USE_ESCSERIAL
//#define ESCSERIAL_TIMER_TX_PIN  PE13 // ( Hardware=0)

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff


#define USABLE_TIMER_CHANNEL_COUNT 10

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5)| TIM_N(8) )


