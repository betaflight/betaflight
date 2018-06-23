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

#if defined(CRAZYBEEF3FS)
#define TARGET_BOARD_IDENTIFIER "CBFS"
#define USBD_PRODUCT_STRING     "CrazyBee F3 FS"
#else //CRAZYBEEF3FS
#define TARGET_BOARD_IDENTIFIER "CBFR"
#define USBD_PRODUCT_STRING     "CrazyBee F3 FR"
#endif

#define ENABLE_DSHOT_DMAR       true

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
#define GYRO_MPU6000_ALIGN      CW90_DEG
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW90_DEG

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

#if defined(CRAZYBEEF3FS)
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
#else
#define USE_RX_SPI
#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_FRSKY_SPI_TELEMETRY
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI	
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKY_X
#define RX_SPI_INSTANCE         SPI2
#define RX_NSS_PIN              SPI2_NSS_PIN
#define RX_SCK_PIN              SPI2_SCK_PIN
#define RX_MISO_PIN             SPI2_MISO_PIN
#define RX_MOSI_PIN             SPI2_MOSI_PIN
#define RX_FRSKY_SPI_GDO_0_PIN  PA8
#define RX_FRSKY_SPI_LED_PIN    PA10
#define BINDPLUG_PIN            PA9
#endif

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB1
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA1
#define ADC_INSTANCE            ADC1
#define CURRENT_METER_SCALE_DEFAULT 2350

#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_RX_SPI | FEATURE_MOTOR_STOP)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))
#define USABLE_TIMER_CHANNEL_COUNT 4
#define USED_TIMERS             (TIM_N(2) | TIM_N(8) | TIM_N(15))
