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

#define USE_TARGET_CONFIG
#undef USE_MSP_DISPLAYPORT

#ifdef MLTEMPF4
#define TARGET_BOARD_IDENTIFIER "MLTE"
#else
#define TARGET_BOARD_IDENTIFIER "MLTY"
#endif

#define USBD_PRODUCT_STRING "MotoLabF4"

#define LED0_PIN                PC3
//#define LED1                    PC4

#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_1_ALIGN            CW180_DEG

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define ACC_1_ALIGN             CW180_DEG

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC5
#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW
//#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define USE_SPI_DEVICE_3        // MAX7456 on V1.2
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      SPI3_NSS_PIN
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_VCP
//#define USB_DETECT_PIN          PC15
//#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

// Pins are available, not connected
//#define USE_UART3
//#define UART3_RX_PIN            PB11
//#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12    // not connected

#ifdef MLTEMPF4
#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART2, UART4, UART5, USART6
#else
#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART2, UART4, UART5
#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI
#define RTC6705_SPI_MOSI_PIN    PC6
#define RTC6705_SPICLK_PIN      PC2
#define RTC6705_CS_PIN          PC7
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8

#define USE_SPI
#define USE_SPI_DEVICE_1        // MPU6000
#define USE_SPI_DEVICE_2        // SDcard

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC13
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   PB12
#define SDCARD_SPI_CS_CFG                   IOCFG_OUT_OD
#define SPI2_TX_DMA_OPT                     0     // DMA 1 Stream 4 Channel 0

// Pins are available unless USART3 is connected, not connected
//#define USE_I2C
//#define I2C_DEVICE (I2CDEV_2)

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
// Reserved pins, not connected
//#define RSSI_ADC_PIN            PC2

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_UART5
#define SBUS_TELEMETRY_UART     SERIAL_PORT_UART4
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD)
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 140

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) )
