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
#define TARGET_BOARD_IDENTIFIER "PIK4"
#define USBD_PRODUCT_STRING     "PikoF4"
#define TARGET_CONFIG
/*--------------LED----------------*/
#define LED0_PIN                PA15
#define LED1_PIN                PB6
#define LED_STRIP
/*---------------------------------*/

/*------------BEEPER---------------*/
#define BEEPER                  PA14
#define BEEPER_INVERTED
/*---------------------------------*/

/*------------SENSORS--------------*/
// MPU interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PC4
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1

#define ICM20689_CS_PIN         PA4
#define ICM20689_SPI_INSTANCE   SPI1

#define GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_ICM20689_ALIGN     CW180_DEG

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW180_DEG

#define ACC
#define USE_ACC_SPI_ICM20689
#define ACC_ICM20689_ALIGN      CW180_DEG

#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG
/*---------------------------------*/

/*------------FLASH----------------*/
#define M25P16_CS_PIN           PB3
#define M25P16_SPI_INSTANCE     SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
/*---------------------------------*/

/*-----------USB-UARTs-------------*/
#define USE_VCP
//#define VBUS_SENSING_PIN PA8
//#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#define INVERTER_PIN_UART3      PC8

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       4

#define CMS
#define USE_MSP_DISPLAYPORT
/*---------------------------------*/

/*-------------SPIs----------------*/
#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
/*---------------------------------*/

/*-------------ADCs----------------*/
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define VBAT_ADC_PIN            PC2
/*---------------------------------*/

/*-------------ESCs----------------*/
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA3  // (HARDARE=0)
/*---------------------------------*/

/*--------DEFAULT VALUES-----------*/
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
/*---------------------------------*/

/*--------------TIMERS-------------*/
#define USABLE_TIMER_CHANNEL_COUNT  5
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) )
/*---------------------------------*/
