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

#define TARGET_BOARD_IDENTIFIER "AR32" // AiR32


#define LED0_PIN                PB5 // Blue LED - PB5

#define USE_BEEPER
#define BEEPER_PIN              PA0

// MPU6050 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA15
#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_ACC

// MPU6050 support has been dropped according to board wiki
// https://github.com/betaflight/betaflight/wiki/Board---AIR32
//#define USE_GYRO_MPU6050
//#define USE_ACC_MPU6050

#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000

#define GYRO_1_CS_PIN           PB12
#define GYRO_1_SPI_INSTANCE     SPI2

#define GYRO_1_ALIGN            CW180_DEG
#define ACC_1_ALIGN             CW180_DEG

//#define USE_BARO
//#define USE_BARO_MS5611

//#define USE_MAG
//#define USE_MAG_HMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA4  // (HARDARE=0)

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PB3
#define UART2_RX_PIN            PB4

#define UART3_TX_PIN            PB10 //(AF7)
#define UART3_RX_PIN            PB11 //(AF7)

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define USE_SPI
#define USE_SPI_DEVICE_2

#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define ADC_INSTANCE            ADC2
#define VBAT_ADC_PIN            PA5
//#define CURRENT_METER_ADC_PIN   PA5
#define RSSI_ADC_PIN            PB2

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
// #define TARGET_IO_PORTF (BIT(0)|BIT(1))
// !!TODO - check the following line is correct
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(16) |  TIM_N(17))
