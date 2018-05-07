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


#define TARGET_BOARD_IDENTIFIER "BVFS"
#define USBD_PRODUCT_STRING     "BETAFPVF3FS"

#define LED0_PIN                PB3
#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI PC13
#define USE_MPU_DATA_READY_SIGNAL
#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6000_CS_PIN          PA4
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define USE_VCP
#define USE_UART3
#define SERIAL_PORT_COUNT       2
#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_RX_SPI
#define USE_RX_FLYSKY
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_A7105_FLYSKY_2A
#define FLYSKY_2A_CHANNEL_COUNT 14
#define RX_SPI_INSTANCE         SPI2
#define RX_NSS_PIN              SPI2_NSS_PIN
#define RX_IRQ_PIN              PA8
#define BINDPLUG_PIN            PA9
#define USE_RX_FLYSKY_SPI_LED
#define RX_FLYSKY_SPI_LED_PIN   PA10

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB1
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define VBAT_SCALE_DEFAULT      61

#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_RX_SPI | FEATURE_MOTOR_STOP)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))
#define USABLE_TIMER_CHANNEL_COUNT 4
#define USED_TIMERS             (TIM_N(2) | TIM_N(8) | TIM_N(15))
