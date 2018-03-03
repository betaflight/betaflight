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

#define TARGET_BOARD_IDENTIFIER "DOGE"

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

// tqfp48 pin 34
#define LED0_PIN                PA13
// tqfp48 pin 37
#define LED1_PIN                PA14
// tqfp48 pin 38
#define LED2_PIN                PA15

#define BEEPER                  PB2
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

// tqfp48 pin 39
#define SPI1_SCK_PIN            PB3
// tqfp48 pin 40
#define SPI1_MISO_PIN           PB4
// tqfp48 pin 41
#define SPI1_MOSI_PIN           PB5
// tqfp48 pin 3
#define SPI1_NSS_PIN            PC14

// tqfp48 pin 26
#define SPI2_SCK_PIN            PB13
// tqfp48 pin 27
#define SPI2_MISO_PIN           PB14
// tqfp48 pin 28
#define SPI2_MOSI_PIN           PB15
// tqfp48 pin 25
#define SPI2_NSS_PIN            PB12

// tqfp48 pin 3
#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI1
#define MPU6000_CS_PIN          SPI1_NSS_PIN
#define MPU6000_SPI_INSTANCE    SPI1

// tqfp48 pin 25
#define BMP280_CS_PIN           SPI2_NSS_PIN
#define BMP280_SPI_INSTANCE     SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_SPI_SHARED
#define M25P16_CS_PIN           PC15
#define M25P16_SPI_INSTANCE     SPI2

#define USE_GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW270_DEG

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8 // (Hardware=0)

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC24_DMA_REMAP // moves ADC2 DMA from DMA2ch1 to DMA2ch3.
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5

// mpu_int definition in sensors/initialisation.c
#define USE_EXTI
#define MPU_INT_EXTI            PC13
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_ESC_SENSOR

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// !!TODO - check the TARGET_IO_PORTs are correct
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

// timer definitions in drivers/timer.c
// channel mapping in drivers/pwm_mapping.c
// only 6 outputs available on hardware
#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15))
