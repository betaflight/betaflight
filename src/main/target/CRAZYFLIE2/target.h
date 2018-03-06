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

/*
 * This target is for the Crazyflie 2.0 nanocopter board
 *
 * For details on using this target with the Crazyflie see:
 * https://wiki.bitcraze.io/misc:hacks:betaflight
 *
 * Target code written and maintained by Sean Kelly (theseankelly@outlook.com)
 */

#pragma once

#if defined(CRAZYFLIE2BQ)
#define TARGET_BOARD_IDENTIFIER "CFBQ"
#define USBD_PRODUCT_STRING     "Crazyflie 2.0 (BigQuad Deck)"
#else
#define TARGET_BOARD_IDENTIFIER "CF20"
#define USBD_PRODUCT_STRING     "Crazyflie 2.0"
#endif

#define USABLE_TIMER_CHANNEL_COUNT 14

#if defined(CRAZYFLIE2BQ)
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(14) )
#else
#define USED_TIMERS             ( TIM_N(2) | TIM_N(4) )
#endif

#define LED0_PIN                PD2
#define LED1_PIN                PC0
#define LED2_PIN                PC3

// Using STM32F405RG, 64 pin package (LQFP64)
// 16 pins per port, ports A, B, C, and also PD2
#define TARGET_IO_PORTA         0xFFFF
#define TARGET_IO_PORTB         0xFFFF
#define TARGET_IO_PORTC         0xFFFF
#define TARGET_IO_PORTD         (BIT(2))

#define USE_VCP

#if defined(CRAZYFLIE2BQ)
#define USE_UART1
#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define USE_UART3
#define UART3_TX_PIN            PC10
#define UART3_RX_PIN            PC11
#endif

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#if defined(CRAZYFLIE2BQ)
#define SERIAL_PORT_COUNT       4
#else
#define SERIAL_PORT_COUNT       2
#endif

#define USE_I2C
#define USE_I2C_DEVICE_3
#define I2C_DEVICE              (I2CDEV_3)

// MPU9250 has the AD0 pin held high so the
// address is 0x69 instead of the default 0x68
#define MPU_ADDRESS             0x69

#define USE_GYRO
#define USE_GYRO_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

#define USE_MAG
#define USE_MPU9250_MAG // Enables bypass configuration on the MPU9250 I2C bus
#define USE_MAG_AK8963
#define MAG_AK8963_ALIGN        CW270_DEG

#define USE_EXTI
#define MPU_INT_EXTI            PC13

#define USE_SERIALRX_TARGET_CUSTOM
#define SERIALRX_UART           SERIAL_PORT_USART6
#define SERIALRX_PROVIDER       SERIALRX_TARGET_CUSTOM
#define RX_CHANNELS_TAER

#if defined(CRAZYFLIE2BQ)
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#else
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#endif

#if defined(CRAZYFLIE2BQ)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define BEEPER                  PC12

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC1
#define CURRENT_METER_ADC_PIN   PA5
#define VBAT_ADC_PIN            PA6
#else
#define BRUSHED_MOTORS
#endif
