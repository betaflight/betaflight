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

#define TARGET_BOARD_IDENTIFIER "AFF3" // AlienFlight F3.
#define USE_TARGET_CONFIG
#define TARGET_BUS_INIT
#define REMAP_TIM17_DMA


#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN                  PB2

// LED's V1
#define LED0_PIN                PB4
#define LED1_PIN                PB5

// LED's V2
#define LED0_A                  PB8
#define LED1_A                  PB9

#define USE_BEEPER
#define BEEPER_PIN              PA5

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL

// Using MPU6050 for the moment.
#define USE_GYRO
#define USE_GYRO_MPU6050
#define USE_GYRO_SPI_MPU6500

#define GYRO_1_ALIGN            CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6050
#define USE_ACC_SPI_MPU6500


// No baro support.
//#define USE_BARO
//#define USE_BARO_MS5611

// option to use MPU9150 or MPU9250 integrated AK89xx Mag
#define USE_MAG
#define USE_MAG_AK8963
#define MAG_AK8963_ALIGN        CW180_DEG_FLIP

#define USE_VCP
#define USE_UART1 // Not connected - TX (PB6) RX PB7 (AF7)
#define USE_UART2 // Receiver - RX (PA3)
#define USE_UART3 // Not connected - 10/RX (PB11) 11/TX (PB10)
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define AVOID_UART2_FOR_PWM_PPM

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB15  // (HARDARE=0)

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define USE_SPI
#define USE_SPI_DEVICE_3

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI3

#define USE_ADC

#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PA4
#define VBAT_SCALE_DEFAULT      20

#define BINDPLUG_PIN            PB12

#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART2

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17) )
