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

#define TARGET_BOARD_IDENTIFIER "IFF3"

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define LED0_PIN                PB3

#define USE_EXTI
#define MPU_INT_EXTI            PC13
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW270_DEG

#define USE_BARO
#define USE_BARO_BMP085

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 5

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA0  // (HARDARE=0,PPM)

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5
#define RSSI_ADC_PIN            PB2

#undef USE_LED_STRIP

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

/*
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_SERIAL_4WAY_SK_BOOTLOADER

#if !(defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) || defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
#ifdef USE_VCP
#define USE_SERIAL_1WIRE_VCP
#else
#define USE_SERIAL_1WIRE
#endif
#endif

#ifdef USE_SERIAL_1WIRE
#define S1W_TX_GPIO         GPIOA
#define S1W_TX_PIN          GPIO_Pin_9
#define S1W_RX_GPIO         GPIOA
#define S1W_RX_PIN          GPIO_Pin_10
#endif
*/

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA     0xffff
#define TARGET_IO_PORTB     0xffff
#define TARGET_IO_PORTC     (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF     (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS         (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) |TIM_N(17))
