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

#define TARGET_BOARD_IDENTIFIER "MIF3"


#undef USE_SERIAL_RX
#undef USE_CRSF_CMS_TELEMETRY
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_IBUS_EXTENDED
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_SRXL
#undef USE_PWM

#undef USE_GYRO_OVERFLOW_CHECK

#define LED0_PIN                PB5

#define USE_BEEPER
#define BEEPER_PIN              PC14

#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_1_ALIGN            CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6050

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA13
#define USE_MPU_DATA_READY_SIGNAL

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PB3
#define UART2_RX_PIN            PB4

//#define UART3_TX_PIN            PB10

#define SERIAL_PORT_COUNT       3

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE             (I2CDEV_1)
#define I2C1_SDA               PA14
#define I2C1_SCL               PA15

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   PB12
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA1

// Can be used on motor 1 / 2 pads (A02 / A03):
#define EXTERNAL1_ADC_PIN       NONE
#define RSSI_ADC_PIN            NONE

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1

#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_SFHSS_SPI
#define USE_RX_REDPINE_SPI
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKY_X
#define USE_RX_FRSKY_SPI_TELEMETRY

#define RX_NSS_PIN              PA4

#define RX_SPI_EXTI_PIN         PB0

#define RX_SPI_LED_PIN          PB6

#define USE_RX_CC2500_SPI_PA_LNA

#define RX_CC2500_SPI_TX_EN_PIN   PB1
#define RX_CC2500_SPI_LNA_EN_PIN  PB11


#define USE_RX_CC2500_SPI_DIVERSITY

#define RX_CC2500_SPI_ANT_SEL_PIN PB2

#define RX_SPI_BIND_PIN          PC13

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN   PB9 // Motor 6, can't use escserial for hexa

#define DEFAULT_FEATURES        (FEATURE_AIRMODE | FEATURE_TELEMETRY)

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(15))
