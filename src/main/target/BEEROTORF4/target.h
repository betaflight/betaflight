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
#define TARGET_BOARD_IDENTIFIER "BRF4"

#define USBD_PRODUCT_STRING "BeeRotorF4"

#define LED0_PIN                PB4

#define USE_BEEPER
#define BEEPER_PIN              PB3
#define BEEPER_INVERTED

// Tim_UP 1 (motors 1 & 2) conflicts with Tim 4 Ch 3 (LED_STRIP)
#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_OFF

// ICM20689 interrupt
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA8
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN           SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_ACC
#define USE_ACC_SPI_ICM20689

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_1_ALIGN            CW270_DEG

#define USE_BARO
#define USE_BARO_BMP280

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      SPI3_NSS_PIN

#define SPI3_TX_DMA_OPT         0  // DMA 1 Stream 5 Channel 0
#define SPI3_RX_DMA_OPT         0  // DMA 1 Stream 0 Channel 0

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_PIN       PC3
#define SDCARD_DETECT_INVERTED
#define SDCARD_SPI_INSTANCE     SPI2
#define SDCARD_SPI_CS_PIN       SPI2_NSS_PIN
#define SPI2_TX_DMA_OPT         0  // DMA 1 Stream 4 Channel 0

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PC5

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

//SerialRX
#define USE_UART2
#define UART2_RX_PIN            PA3 //Shared with PPM
#define UART2_TX_PIN            PA2

#define INVERTER_PIN_UART2      PC15

//Telemetry
#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define INVERTER_PIN_UART3      PC14

#define SERIAL_PORT_COUNT 4

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA3  // (HARDARE=0,PPM)

#define USE_SPI

//ICM20689
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

//SDCard
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//MAX7456 / SPI RX
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
//#define SPI_RX_CS_PIN           PD2

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_ADC
#define ADC_INSTANCE            ADC1  // Default added
#define ADC1_DMA_OPT            0  // DMA 2 Stream 0 Channel 0 


#define VBAT_ADC_PIN            PC0

#define CURRENT_METER_ADC_PIN   PC1

#define RSSI_ADC_PIN            PC2

#define USE_TRANSPONDER

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_AIRMODE )
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) )
