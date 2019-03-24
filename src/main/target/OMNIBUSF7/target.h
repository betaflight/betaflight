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

//OMNIBUSF7 TARGETS-------------------------
#define USE_TARGET_CONFIG
#if defined (FPVM_BETAFLIGHTF7)
#define TARGET_BOARD_IDENTIFIER "FBF7"
#define USBD_PRODUCT_STRING "FPVM_BETAFLIGHTF7"
#elif defined (OMNIBUSF7V2)
#define TARGET_BOARD_IDENTIFIER "OB72"
#define USBD_PRODUCT_STRING "OmnibusF7V2"
#else
#define TARGET_BOARD_IDENTIFIER "OBF7"
#define USBD_PRODUCT_STRING "OmnibusF7"
#endif

//LED & BEEPER------------------------------
#define LED0_PIN                PE0

#define USE_BEEPER
#define BEEPER_PIN              PD15
#define BEEPER_INVERTED


//GYRO & ACC--------------------------------
#define USE_ACC
#define USE_GYRO
#define USE_MULTI_GYRO
// ICM-20608-G
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500

// MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000

//#define USE_MPU_DATA_READY_SIGNAL
#define USE_EXTI
#define USE_GYRO_EXTI

#if defined(OMNIBUSF7V2)
#define GYRO_1_SPI_INSTANCE     SPI3
#define GYRO_1_CS_PIN           PA15
#define GYRO_2_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           PA4
#define GYRO_1_ALIGN            CW90_DEG
#define GYRO_2_ALIGN            ALIGN_DEFAULT
#define ACC_1_ALIGN             CW90_DEG
#define ACC_2_ALIGN             ALIGN_DEFAULT
#define GYRO_1_EXTI_PIN         PD0           // MPU6000
#define GYRO_2_EXTI_PIN         PE8           // ICM20608

#elif defined(FPVM_BETAFLIGHTF7)
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA4
#define GYRO_2_SPI_INSTANCE     SPI3
#define GYRO_2_CS_PIN           PA15
#define GYRO_1_ALIGN            CW90_DEG
#define ACC_1_ALIGN             CW90_DEG
#define GYRO_2_ALIGN            CW270_DEG
#define ACC_2_ALIGN             CW270_DEG
#define GYRO_1_EXTI_PIN         PD0           // Assume the same as OMNIBUSF7V2, need to verify
#define GYRO_2_EXTI_PIN         PE8           // Ditto

#else
#define GYRO_1_SPI_INSTANCE     SPI3
#define GYRO_1_CS_PIN           PA15
#define GYRO_2_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           PA4
#define GYRO_1_ALIGN            ALIGN_DEFAULT
#define ACC_1_ALIGN             ALIGN_DEFAULT
#define GYRO_2_ALIGN            ALIGN_DEFAULT
#define ACC_2_ALIGN             ALIGN_DEFAULT
#define GYRO_1_EXTI_PIN         PE8           // ICM20608
#define GYRO_2_EXTI_PIN         PD0           // MPU6000
#endif

//UARTS-------------------------------------
#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PC4

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

//#define AVOID_UART2_FOR_PWM_PPM // PPM is not working on RC pin anyway
#define USE_UART2
#if defined (FPVM_BETAFLIGHTF7)
#define UART2_TX_PIN            PA2
#else
#define UART2_TX_PIN            NONE
#endif
#define UART2_RX_PIN            PA3

// Assigned to shared output I2C2
#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#ifdef OMNIBUSF7V2
#define USE_UART7
#define UART7_RX_PIN            PE7
#endif

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#ifdef OMNIBUSF7V2
#define SERIAL_PORT_COUNT 8
#else
#define SERIAL_PORT_COUNT 7
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA2 // (Unwired UART2_TX)

//SPI---------------------------------------
#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define SPI4_NSS_PIN            PE4
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6

//OSD----------------------------------------
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#ifdef FPVM_BETAFLIGHTF7
//FLASH--------------------------------------
#define FLASH_CS_PIN         SPI4_NSS_PIN
#define FLASH_SPI_INSTANCE   SPI4

#define USE_FLASHFS
#define USE_FLASH_M25P16

#else

//SD-----------------------------------------
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PE3
#define SDCARD_SPI_INSTANCE                 SPI4
#define SDCARD_SPI_CS_PIN                   SPI4_NSS_PIN
#define SPI4_TX_DMA_OPT                     0     // DMA 2 Stream 1 Channel 4

#endif

//I2C---------------------------------------
#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL                NONE        // PB10 (UART3_TX)
#define I2C2_SDA                NONE        // PB11 (UART3_RX)

//BARO & Mag--------------------------------
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BARO_SPI_INSTANCE       SPI1
#define BARO_CS_PIN             PA1

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL

//ADC---------------------------------------
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            1  // DMA 2 Stream 4 Channel 0 (compat default)
#define CURRENT_METER_ADC_PIN   PC2
#define VBAT_ADC_PIN            PC3
#define RSSI_ADC_PIN            PC5

// Additional sensors ----------------------
#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN     PB10 // TX3 for testing
#define RANGEFINDER_HCSR04_ECHO_PIN        PB11 // RX3 for testing

//DEFAULTS----------------------------------

#define DEFAULT_FEATURES        (FEATURE_OSD)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#ifdef FPVM_BETAFLIGHTF7
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define SERIALRX_UART           SERIAL_PORT_USART6
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#else
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define SERIALRX_UART           SERIAL_PORT_USART2
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#endif

//PORT'S & TIMERS---------------------------
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#ifdef FPVM_BETAFLIGHTF7
#define USABLE_TIMER_CHANNEL_COUNT 13
#else
#define USABLE_TIMER_CHANNEL_COUNT 12
#endif

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(9) )
