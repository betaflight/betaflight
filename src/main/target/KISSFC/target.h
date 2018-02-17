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

#define TARGET_BOARD_IDENTIFIER "KISS"

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN | SERIAL_INVERTED | SERIAL_BIDIR)

#define USE_ESC_SENSOR

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA13  // (Hardware=6, common to KISSFC & KISSCC)
#define REMAP_TIM17_DMA

#define LED0_PIN                PB1

#define BEEPER                  PB13
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI            PB2
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#ifdef KISSCC
#define USE_TARGET_CONFIG

#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW90_DEG

#define USE_ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW90_DEG

#define TARGET_DEFAULT_MIXER    MIXER_QUADX_1234

#undef USE_LED_STRIP
#else
#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW180_DEG

#define USE_ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW180_DEG
#endif

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PB3
#define UART2_RX_PIN            PB4

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#ifdef KISSCC
#define SOFTSERIAL1_TX_PIN      PA13
#else
#define SOFTSERIAL1_TX_PIN      PA13 // AUX1
#define SOFTSERIAL2_TX_PIN      PA15 // ROLL
#endif

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define VBAT_SCALE_DEFAULT      160
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA2
//#define RSSI_ADC_PIN            PB2

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define AVOID_UART2_FOR_PWM_PPM

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTF         (BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17))
