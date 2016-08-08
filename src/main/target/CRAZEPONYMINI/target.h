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

#define BRUSHED_MOTORS

#define LED0                    PA11
#define LED1                    PA8
#define LED2                    PB1

#define ACC
#define USE_ACC_MPU6050

#define GYRO
#define USE_GYRO_MPU6050

#define BARO
#define USE_BARO_MS5611

//#define MAG
//#define USE_MAG_HMC5883

#define USE_UART1
#define SERIAL_PORT_COUNT       1

#define USE_ADC
#define VBAT_ADC_PIN            PB0

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)

#define DEFAULT_RX_FEATURE      FEATURE_RX_NRF24
#define USE_RX_NRF24
//#define USE_RX_CX10
//#define USE_RX_H8_3D
#define USE_RX_SYMA
#define USE_RX_V202
#define NRF24_DEFAULT_PROTOCOL  NRF24RX_V202_1M

#define USE_SPI
#define USE_SPI_DEVICE_1

#define NRF24_SPI_INSTANCE       SPI1
#define USE_NRF24_SPI1

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define NRF24_CE_PIN                    PA12
#define NRF24_CE_GPIO_CLK_PERIPHERAL    RCC_APB2Periph_GPIOA
#define NRF24_CSN_PIN                   PA4
#define NRF24_CSN_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define NRF24_IRQ_PIN                   PA15
#define NRF24_IRQ_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA

#define SKIP_RX_MSP
#define SKIP_INFLIGHT_ADJUSTMENTS
#define SKIP_RX_PWM_PPM
#undef SERIAL_RX
#undef BLACKBOX

// Since the CrazePony MINI PCB has holes for 4 motors in each corner we can save same flash space by disabling support for other mixers.
#undef USE_SERVOS
#define USE_QUAD_MIXER_ONLY


// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             TIM_N(2)
