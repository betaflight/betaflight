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

#define TARGET_BOARD_IDENTIFIER "CJM1" // CJMCU
#define USE_HARDWARE_REVISION_DETECTION
#define USE_TARGET_CONFIG
#define TARGET_BUS_INIT

#define LED0_PIN                PC14
#define LED1_PIN                PC13
#define LED2_PIN                PC15

#undef USE_BEEPER

#define USE_GYRO
#define USE_GYRO_MPU6050

#define USE_ACC
#define USE_ACC_MPU6050

//#define USE_MAG
//#define USE_MAG_HMC5883

#define BRUSHED_MOTORS

#define USE_UART1
#define USE_UART2

#define SERIAL_PORT_COUNT       2

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67


#define USE_RX_NRF24
#ifdef USE_RX_NRF24

#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define RX_CE_PIN               PA4
#define RX_NSS_PIN              PA11
#define RX_SPI_EXTI_PIN         PA8
// CJMCU has NSS on PA11, rather than the standard PA4
#define SPI1_NSS_PIN            RX_NSS_PIN

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_RX_NRF24
#define USE_RX_CX10
#define USE_RX_H8_3D
//#define USE_RX_INAV // Temporary disabled to make some room in flash
//#define USE_RX_SYMA
#define USE_RX_V202
//#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_SYMA_X5
//#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_SYMA_X5C
//#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_INAV
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_H8_3D
//#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_CX10A
//#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_V202_1M

#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
//#define USE_TELEMETRY
//#define USE_TELEMETRY_LTM
//#define USE_TELEMETRY_NRF24_LTM
#ifdef USE_PWM
#undef USE_PWM
#endif

#ifdef USE_PPM
#undef USE_PPM
#endif

#ifdef USE_SERIAL_RX
#undef USE_SERIAL_RX
#endif
#else

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define USE_RX_MSP

#endif //USE_RX_NRF24

#define BRUSHED_MOTORS
#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP
#undef USE_SERIAL_PASSTHROUGH
#undef USE_CLI

// Since the CJMCU PCB has holes for 4 motors in each corner we can save same flash space by disabling support for other mixers.
#define USE_QUAD_MIXER_ONLY
#undef USE_SERVOS

// Number of available PWM outputs
//#define MAX_PWM_OUTPUT_PORTS    4

// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
