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

#define BRUSHED_MOTORS

#define LED0
#define LED0_GPIO GPIOC
#define LED0_PIN Pin_14 // PC14 (LED)
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOC

#define LED1
#define LED1_GPIO GPIOC
#define LED1_PIN Pin_13 // PC13 (LED)
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOC

#define LED2
#define LED2_GPIO GPIOC
#define LED2_PIN Pin_15 // PC15 (LED)
#define LED2_PERIPHERAL RCC_APB2Periph_GPIOC

#undef BEEPER

#define GYRO
#define USE_GYRO_MPU6050

#define ACC
#define USE_ACC_MPU6050

#define MAG
#define USE_MAG_HMC5883

#define USE_USART1
#define USE_USART2
#define SERIAL_PORT_COUNT 2

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define NRF24_SPI_INSTANCE       SPI1
#define USE_NRF24_SPI1

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define NRF24_CE_GPIO                   GPIOA
#define NRF24_CE_PIN                    GPIO_Pin_4
#define NRF24_CE_GPIO_CLK_PERIPHERAL    RCC_APB2Periph_GPIOA
#define NRF24_CSN_GPIO                  GPIOA
#define NRF24_CSN_PIN                   GPIO_Pin_11
#define NRF24_CSN_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define NRF24_IRQ_GPIO                  GPIOA
#define NRF24_IRQ_PIN                   GPIO_Pin_8
#define NRF24_IRQ_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA

#define USE_RX_NRF24
#define USE_RX_V202
#define USE_RX_SYMA
#define USE_RX_CX10
#define NRF24_DEFAULT_PROTOCOL NRF24RX_SYMA_X5C

#define DEFAULT_RX_FEATURE FEATURE_RX_NRF24
//#define DEFAULT_RX_FEATURE FEATURE_RX_PPM

#define DEFAULT_FEATURES FEATURE_MOTOR_STOP

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PORT  GPIOA
#define BIND_PIN   Pin_3


// Since the CJMCU PCB has holes for 4 motors in each corner we can save same flash space by disabling support for other mixers.
#define USE_QUAD_MIXER_ONLY
#undef USE_SERVOS

#if (FLASH_SIZE <= 64)
//#define SKIP_TASK_STATISTICS
#define SKIP_RX_PWM_PPM
#define SKIP_CLI_COMMAND_HELP
#undef SERIAL_RX
#undef BLACKBOX
#endif

#undef SKIP_RX_MSP

// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))

#define USED_TIMERS     (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
