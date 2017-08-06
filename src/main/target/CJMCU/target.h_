/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "CJM1" // CJMCU
#define USE_HARDWARE_REVISION_DETECTION

#define LED0                    PC14
#define LED1                    PC13
#define LED2                    PC15

#undef BEEPER

#define GYRO
#define USE_GYRO_MPU6050

#define ACC
#define USE_ACC_MPU6050

//#define MAG
//#define USE_MAG_HMC5883

#define USE_UART1
#define USE_UART2
#define SERIAL_PORT_COUNT       2

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_RX_NRF24
#ifdef USE_RX_NRF24

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define RX_CE_GPIO_CLK_PERIPHERAL    RCC_APB2Periph_GPIOA
#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define RX_IRQ_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define RX_CE_PIN               PA4
#define RX_NSS_PIN              PA11
#define RX_SCK_PIN              PA5
#define RX_MISO_PIN             PA6
#define RX_MOSI_PIN             PA7
#define RX_IRQ_PIN              PA8
// CJMCU has NSS on PA11, rather than the standard PA4
#define SPI1_NSS_PIN            RX_NSS_PIN
#define SPI1_SCK_PIN            RX_SCK_PIN
#define SPI1_MISO_PIN           RX_MISO_PIN
#define SPI1_MOSI_PIN           RX_MOSI_PIN

#define USE_RX_NRF24
//#define USE_RX_CX10
#define USE_RX_H8_3D
//#define USE_RX_INAV
//#define USE_RX_SYMA
//#define USE_RX_V202
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_SYMA_X5
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_SYMA_X5C
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_INAV
#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_H8_3D
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_CX10A
//#define RX_SPI_DEFAULT_PROTOCOL NRF24RX_V202_1M

#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
//#define TELEMETRY
//#define TELEMETRY_LTM
//#define TELEMETRY_NRF24_LTM
#undef USE_RX_PWM
#undef USE_RX_PPM
#undef SERIAL_RX
#undef SKIP_TASK_STATISTICS

#else

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define USE_RX_MSP
#define SPEKTRUM_BIND
#define BIND_PIN                PA3 // UART2, PA3

#endif //USE_RX_NRF24

#define BRUSHED_MOTORS
#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP
#undef USE_SERIAL_PASSTHROUGH

// Since the CJMCU PCB has holes for 4 motors in each corner we can save same flash space by disabling support for other mixers.
#define USE_QUAD_MIXER_ONLY
#undef USE_SERVOS

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    4

// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
