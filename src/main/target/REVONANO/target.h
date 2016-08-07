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
#define TARGET_BOARD_IDENTIFIER "REVN"

#define CONFIG_START_FLASH_ADDRESS (0x08060000) //0x08060000 to 0x08080000 (FLASH_Sector_7)

#define USBD_PRODUCT_STRING "Revo Nano"
#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8010000"
#endif

#define LED0                    PC14
#define LED1                    PC13

#define BEEPER                  PC13

#define INVERTER                PC15
#define INVERTER_USART          USART2 //Sbus on USART 2 of nano.

#define MPU9250_CS_PIN          PB12
#define MPU9250_SPI_INSTANCE    SPI2

#define ACC
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU9250_ALIGN       CW270_DEG

#define GYRO
#define USE_GYRO_MPU9250
#define USE_GYRO_SPI_MPU9250
#define GYRO_MPU9250_ALIGN      CW270_DEG

//#define MAG
//#define USE_MAG_HMC5883

#define BARO
#define USE_BARO_MS5611

// MPU9250 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PA15
#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready (mag disabled)

#define USE_VCP
#define VBUS_SENSING_PIN        PA9

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define SERIAL_PORT_COUNT       3 //VCP, USART1, USART2

#define USE_SPI
//#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
//#define USE_SPI_DEVICE_3

#define USE_I2C
#define I2C_DEVICE (I2CDEV_3)

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PA7
#define VBAT_ADC_PIN            PA6
#define RSSI_ADC_PIN            PA5

#define GPS
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) )

