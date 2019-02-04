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
#define TARGET_BOARD_IDENTIFIER "F4BY"

#define USBD_PRODUCT_STRING     "Swift-Flyer F4BY"

#define LED0_PIN                PE3 // Blue LED
#define LED1_PIN                PE2 // Red LED
#define LED2_PIN                PE1 // Blue LED

#define USE_BEEPER
#define BEEPER_PIN              PE5

#define INVERTER_PIN_UART6      PD3



// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PB0
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_1_ALIGN             CW90_DEG

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW90_DEG

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define MAG_HMC5883_ALIGN       CW90_DEG

#define USE_BARO
#define USE_BARO_MS5611

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE     SPI2
#define SDCARD_SPI_CS_PIN       PE15
#define SPI2_TX_DMA_OPT                     0     // DMA 1 Stream 4 Channel 0

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PA9

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
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

//#define USE_UART5  - error in DMA!!!
//#define UART5_RX_PIN            PD2
//#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       8 //VCP, UART1, UART2, UART3, UART4, UART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA0  // (HARDARE=8)

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define SPI1_NSS_PIN            NONE

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            NONE
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5


#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#define USE_I2C_PULLUP
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define VBAT_ADC_PIN            PC3
#define CURRENT_METER_ADC_PIN   PC2
#define RSSI_ADC_PIN            PC1

#undef USE_LED_STRIP

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS             ( TIM_N(1) |TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(9) )
