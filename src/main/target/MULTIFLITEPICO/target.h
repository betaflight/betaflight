/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "MFPB"
#define USE_TARGET_CONFIG


#define LED0_PIN                PB3

#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_1_ALIGN            CW90_DEG

#define USE_ACC
#define USE_ACC_MPU6050
#define ACC_1_ALIGN             CW90_DEG

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#define USE_MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW270_DEG

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH
#define MAG_INT_EXTI            PC14

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB0
#define RANGEFINDER_HCSR04_ECHO_PIN          PB1

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA0  // (HARDARE=0,PPM)

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

/*
#define SOFTSERIAL1_RX_PIN      PB4 // PWM 5
#define SOFTSERIAL1_TX_PIN      PB5 // PWM 6

#define SOFTSERIAL2_RX_PIN      PB0 // PWM 7
#define SOFTSERIAL2_TX_PIN      PB1 // PWM 8
*/

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

//#define FLASH_CS_PIN            PB12
//#define FLASH_SPI_INSTANCE      SPI2

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5
#define RSSI_ADC_PIN            PB2

#define REMAP_TIM17_DMA

// UART1 TX uses DMA1_Channel4, which is also used by dshot on motor 4
#if defined(USE_UART1_TX_DMA) && defined(USE_DSHOT)
#undef USE_UART1_TX_DMA
#endif

//#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_FEATURES        (FEATURE_MOTOR_STOP)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define BRUSHED_MOTORS
#define SERIALRX_PROVIDER       SERIALRX_SPEKTRUM2048
#define SERIALRX_UART           SERIAL_PORT_USART3
#define RX_CHANNELS_TAER

#define BINDPLUG_PIN            PA13

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) | TIM_N(17) )
