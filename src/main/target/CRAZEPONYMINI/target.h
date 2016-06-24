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

#define TARGET_BOARD_IDENTIFIER "CPM1" // CrazePony MINI

#define LED0
#define LED0_GPIO       GPIOA
#define LED0_PIN        Pin_11 // PA11 (M1 LED)
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOA

#define LED1
#define LED1_GPIO       GPIOA
#define LED1_PIN        Pin_8 // PA8  (M2 LED)
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOA

#define LED2
#define LED2_GPIO       GPIOB
#define LED2_PIN        (Pin_1 | Pin_3) // PB1  (M3+M4 LED)
#define LED2_PERIPHERAL RCC_APB2Periph_GPIOB


#define ACC
#define USE_ACC_MPU6050

#define GYRO
#define USE_GYRO_MPU6050

#define BARO
#define USE_BARO_MS5611

#define MAG
#define USE_MAG_HMC5883

#define BRUSHED_MOTORS

#define USE_USART1
#define SERIAL_PORT_COUNT 1

#define USE_ADC

#define BOARD_HAS_VOLTAGE_DIVIDER
#define VBAT_ADC_GPIO               GPIOB
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
#define VBAT_ADC_CHANNEL            ADC_Channel_8

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

#define DEFAULT_RX_FEATURE FEATURE_RX_NRF24
#define USE_RX_NRF24
#define USE_RX_SYMA
#define USE_RX_V202
#define NRF24_DEFAULT_PROTOCOL NRF24RX_V202_1M

#define USE_SPI
#define USE_SPI_DEVICE_1

#define NRF24_SPI_INSTANCE       SPI1
#define USE_NRF24_SPI1

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define NRF24_CE_GPIO                   GPIOA
#define NRF24_CE_PIN                    GPIO_Pin_12
#define NRF24_CE_GPIO_CLK_PERIPHERAL    RCC_APB2Periph_GPIOA
#define NRF24_CSN_GPIO                  GPIOA
#define NRF24_CSN_PIN                   GPIO_Pin_4
#define NRF24_CSN_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define NRF24_IRQ_GPIO                  GPIOA
#define NRF24_IRQ_PIN                   GPIO_Pin_15
#define NRF24_IRQ_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA

#define SKIP_RX_MSP
#define SKIP_INFLIGHT_ADJUSTMENTS
#define SKIP_RX_PWM_PPM
#undef SERIAL_RX
#undef BLACKBOX

// Since the CrazePony MINI PCB has holes for 4 motors in each corner we can save same flash space by disabling support for other mixers.
#undef USE_SERVOS
#define USE_QUAD_MIXER_ONLY

#if (FLASH_SIZE > 64)
#define BLACKBOX
#else
#define SKIP_TASK_STATISTICS
#define SKIP_CLI_COMMAND_HELP
#endif


// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff

#define USED_TIMERS         TIM_N(2)

#define TIMER_APB1_PERIPHERALS RCC_APB1Periph_TIM2
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
