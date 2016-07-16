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

#define TARGET_BOARD_IDENTIFIER "KISSFC"

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN | SERIAL_INVERTED | SERIAL_BIDIR)

#define LED0    PB1
#define BEEPER    PB13

#define USABLE_TIMER_CHANNEL_COUNT 12

#define EXTI2_TS_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL


#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN CW180_DEG

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN CW180_DEG


#define USE_VCP
#define USE_USART1
#define USE_USART2
#define USE_USART3
#define SERIAL_PORT_COUNT 4


#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_3 
#define UART2_RX_PIN        GPIO_Pin_4
#define UART2_GPIO          GPIOB
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource3
#define UART2_RX_PINSOURCE  GPIO_PinSource4

#define UART3_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

//#define USE_ADC
#define ADC_INSTANCE                ADC2
#define VBAT_ADC_PIN                PA4
#define CURRENT_METER_ADC_PIN       PA5
#define RSSI_ADC_PIN                PB2


#define SPEKTRUM_BIND
#define BIND_PIN   PB4

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTF	(BIT(4))

#define USED_TIMERS     (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17))
