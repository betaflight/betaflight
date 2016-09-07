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

#define TARGET_BOARD_IDENTIFIER "AFF1" // AlienFlight F1.
#define TARGET_CONFIG

#define LED0                    PB3
#define LED1                    PB4

#define BEEPER                  PA12

#define USE_EXTI
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_MPU6050

#define GYRO_MPU6050_ALIGN      CW0_DEG

#define ACC
#define USE_ACC_MPU6050

#define ACC_MPU6050_ALIGN       CW0_DEG

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT       3

#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PB1
#define VBAT_ADC_PIN            PA4
#define RSSI_ADC_PIN            PA1
#define EXTERNAL1_ADC_PIN       PA5


#define LED_STRIP
#define WS2811_TIMER                    TIM3
#define WS2811_PIN                      PA6
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC6
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH6_HANDLER

#undef GPS

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

#define HARDWARE_BIND_PLUG
// Hardware bind plug at PB5 (Pin 41)
#define BINDPLUG_PIN            PB5

#define BRUSHED_MOTORS
#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SPEKTRUM2048
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
