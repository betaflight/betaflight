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

#define TARGET_BOARD_IDENTIFIER "CC3D" // CopterControl 3D

#define LED0_PIN                PB3

#define INVERTER_PIN_UART1      PB2 // PB2 (BOOT1) used as inverter select GPIO

#define USE_BEEPER
#define BEEPER_PIN              PA15

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA3
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN       CW270_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_1_ALIGN       CW270_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL

//#define USE_I2C
//#define I2C_DEVICE (I2CDEV_2) // Flex port - SCL/PB10, SDA/PB11

// External I2C BARO
//#define USE_BARO
//#define USE_BARO_MS5611
//#define USE_BARO_BMP085
//#define USE_BARO_BMP280

// External I2C MAG
//#define USE_MAG
//#define USE_MAG_HMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#ifndef CC3D_OPBL
#define SOFTSERIAL1_TX_PIN      PB5 // PWM 2
#define SOFTSERIAL1_RX_PIN      PB0 // PWM 3
#endif

#ifdef USE_UART1_RX_DMA
#undef USE_UART1_RX_DMA
#endif

#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PB1
#define VBAT_ADC_PIN            PA0
#define RSSI_ADC_PIN            PB0

//#define USE_RANGEFINDER
//#define USE_RANGEFINDER_HCSR04
//#define RANGEFINDER_HCSR04_ECHO_PIN          PB0
//#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB5

#undef USE_MAG

#ifdef CC3D_OPBL
//#undef USE_SERVOS
#undef USE_BARO
#undef USE_RANGEFINDER
#undef USE_RANGEFINDER_HCSR04
#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
//#undef USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
//#undef USE_SERIALRX_SBUS       // Frsky and Futaba receivers
//#undef USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#undef USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#undef USE_SERIALRX_SUMD       // Graupner Hott protocol
#undef USE_SERIALRX_SUMH       // Graupner legacy protocol
#undef USE_SERIALRX_XBUS       // JR
#endif

//#undef USE_LED_STRIP
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

// IO - from schematics
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(14) )

#define PARTIAL_REMAP_TIM3

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
