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

#define TARGET_BOARD_IDENTIFIER "CLH7"
#define USBD_PRODUCT_STRING "CLRacingH7 V1.0/H743"

#define USE_TARGET_CONFIG

#define USE_EXTI

#define LED0_PIN                PB8

#define USE_BEEPER
#define BEEPER_PIN              PB9
#define BEEPER_INVERTED
#undef USE_BEEPER_PWM

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PD6
#define UART2_TX_PIN            PD5

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PD0
#define UART4_TX_PIN            PD1

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

// PE7 and PE8 used for QSPI2
//#define USE_UART7
//#define UART7_RX_PIN            PE7
//#define UART7_TX_PIN            PE8

#define USE_UART8
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1

#define USE_VCP

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       10 // 1 VCP + 7 UART + 2 SS

#define USE_SPI

// GYRO1
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

// GYRO2
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

// MAX
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

// Spare
#define USE_SPI_DEVICE_4
#define SPI4_NSS_PIN            PE11
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14

//#define USE_SPI_DEVICE_5
#define SPI5_SCK_PIN            NONE
#define SPI5_MISO_PIN           NONE
#define SPI5_MOSI_PIN           NONE

//#define USE_SPI_DEVICE_6
#define SPI6_SCK_PIN            NONE
#define SPI6_MISO_PIN           NONE
#define SPI6_MOSI_PIN           NONE


#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1
#define FLASH_QUADSPI_INSTANCE  QUADSPI

#define QUADSPI1_MODE           QUADSPI_MODE_BK2_ONLY
#define QUADSPI1_CS_FLAGS       (QUADSPI_BK1_CS_NONE | QUADSPI_BK2_CS_HARDWARE | QUADSPI_CS_MODE_SEPARATE)
#define QUADSPI1_SCK_PIN        PB2
#define QUADSPI1_BK1_CS_PIN     NONE
#define QUADSPI1_BK1_IO0_PIN    NONE
#define QUADSPI1_BK1_IO1_PIN    NONE
#define QUADSPI1_BK1_IO2_PIN    NONE
#define QUADSPI1_BK1_IO3_PIN    NONE
#define QUADSPI1_BK2_CS_PIN     PC11
#define QUADSPI1_BK2_IO0_PIN    PE7
#define QUADSPI1_BK2_IO1_PIN    PE8
#define QUADSPI1_BK2_IO2_PIN    PE9
#define QUADSPI1_BK2_IO3_PIN    PE10

//#define USE_I2C
//#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define I2C_DEVICE (I2CDEV_1)

//#define USE_MAG
//#define USE_MAG_HMC5883         // Test case of I2C device
//#define USE_MAG_SPI_HMC5883     // Test case of SPI mag
//#define HMC5883_SPI_INSTANCE    NULL
//#define HMC5883_CS_PIN          NONE

//#define USE_BARO
//#define USE_BARO_BMP280         // Test case of I2C device
//#define USE_BARO_MS5611
//#define USE_BARO_SPI_BMP280     // Test case of SPI baro
//#define BMP280_SPI_INSTANCE     NULL
//#define BMP280_CS_PIN           NONE

#define USE_GYRO
#define USE_MULTI_GYRO
#define USE_ACC
#define USE_GYRO_EXTI

#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250
#define USE_ACC_SPI_MPU9250
#define USE_ACC_SPI_MPU6500

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_EXTI_PIN         PC3
#define GYRO_1_ALIGN            ALIGN_DEFAULT

#define GYRO_2_CS_PIN           PB12
#define GYRO_2_SPI_INSTANCE     SPI2
#define GYRO_2_EXTI_PIN         PE15
#define GYRO_2_ALIGN            ALIGN_DEFAULT

#define USE_FLASHFS
#define USE_FLASH_CHIP
#define USE_FLASH_W25N01G
#define FLASH_SPI_INSTANCE      NULL
#define FLASH_CS_PIN            NONE


#define USE_BRUSHED_ESC_AUTODETECT  // Detect if brushed motors are connected and set defaults appropriately to avoid motors spinning on boot
#define USE_GYRO_REGISTER_DUMP  // Adds gyroregisters command to cli to dump configured register values

//#define USE_RANGEFINDER
//#define USE_RANGEFINDER_HCSR04
//#define USE_RANGEFINDER_TF

#define USE_PPM
#define USE_PWM

#define USE_SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define USE_SERIALRX_FPORT
#define USE_SERIALRX_JETIEXBUS

#define USE_TELEMETRY
#define USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_LTM
#define USE_TELEMETRY_SMARTPORT

#define USE_LED_STRIP
#define USE_TRANSPONDER

#define USE_ACRO_TRAINER
#define USE_BLACKBOX
#define USE_RUNAWAY_TAKEOFF     // Runaway Takeoff Prevention (anti-taz)

#define USE_SERVOS

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD)
#define USE_OSD
#define USE_CMS

#define USE_VTX_COMMON
#define USE_VTX_CONTROL
#define USE_VTX_TRAMP
#define USE_VTX_SMARTAUDIO
#define USE_CAMERA_CONTROL

#define USE_GPS
#define USE_GPS_RESCUE

#define USE_I2C_OLED_DISPLAY
#define USE_MSP_DISPLAYPORT
#define USE_OSD_OVER_MSP_DISPLAYPORT

#define USE_DMA

// H7 allows all stream to handle any peripheral
// No need for channel definition; each peripheral i/o has a unique channel thanks to MUX.
//#define UART1_TX_DMA_STREAM     DMA1_Stream0
//#define UART2_TX_DMA_STREAM     DMA1_Stream1
//#define UART3_TX_DMA_STREAM     DMA1_Stream2
//#define UART4_TX_DMA_STREAM     DMA1_Stream3
//#define UART5_TX_DMA_STREAM     DMA1_Stream4
//#define UART6_TX_DMA_STREAM     DMA1_Stream5
//#define UART7_TX_DMA_STREAM     DMA1_Stream6
//#define UART8_TX_DMA_STREAM     DMA1_Stream7

#define USE_ADC
#define USE_ADC_INTERNAL

#define ADC1_INSTANCE ADC1
#define ADC2_INSTANCE ADC2
#define ADC3_INSTANCE ADC3
//#define ADC1_DMA_STREAM DMA2_Stream0 // Don't define this while USE_DMA_SPEC
//#define ADC2_DMA_STREAM DMA2_Stream1 // Don't define this while USE_DMA_SPEC
//#define ADC3_DMA_STREAM DMA2_Stream2 // Don't define this while USE_DMA_SPEC
#define ADC1_DMA_OPT 10
#define ADC2_DMA_OPT 11
#define ADC3_DMA_OPT 13

#define VBAT_ADC_PIN            PC0  // ADC1
#define CURRENT_METER_ADC_PIN   PC1  // ADC1
#define RSSI_ADC_PIN            PC2 // ADC2

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ESC

#define USE_ESC_SENSOR

#define USE_DSHOT
#define USE_DSHOT_DMAR

#define USE_GYRO_DATA_ANALYSE
#undef USE_USB_MSC
#undef USE_OVERCLOCK
#define USE_RTC_TIME
#define USE_RCDEVICE

#define USE_PINIO
#define USE_PINIOBOX

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB0

#define DEFAULT_FEATURE (FEATURE_OSD|FEATURE_LED_STRIP|FEATURE_ESC_SENSOR)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_FPORT

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 16

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12) | TIM_N(13) | TIM_N(14) | TIM_N(15) | TIM_N(16) | TIM_N(17))
