/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "UAVP"

#define USBD_PRODUCT_STRING "UAVP-NG HW-0.30-mini"

#define LED0_PIN                PE5
#define LED1_PIN                PE7
#define LED2_PIN                PE6
#define LED3_PIN                PE8

#define USE_BEEPER
#define BEEPER_PIN              PB0
#define BEEPER_INVERTED

#define USE_ACC
#define USE_ACC_SPI_MPU6000

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000

#define GYRO_1_CS_PIN          PA4
#define GYRO_1_SPI_INSTANCE    SPI1

// TODO
#define GYRO_1_ALIGN           CW180_DEG
#define ACC_1_ALIGN            CW180_DEG

// MPU6000 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN          PE0
#define USE_MPU_DATA_READY_SIGNAL

#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN        CW90_DEG
#define MAG_I2C_INSTANCE         I2CDEV_2
#define HMC5883_I2C_INSTANCE     I2CDEV_2
#define MAG_INT_EXTI             PE12
#define USE_MAG_DATA_READY_SIGNAL

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_SPI_MS5611
#define BARO_CS_PIN             PE1
#define BARO_SPI_INSTANCE       SPI1

#if 0 // TODO: Enable SDCard and blackbox logging
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN               PE2
#define SDCARD_SPI_INSTANCE             SPI2
#define SDCARD_SPI_CS_PIN               SPI2_NSS_PIN
#warning Missing channel for F4/F7 spec dma 1 stream 4; DMA_OPT assumed as 0
#define SPI2_TX_DMA_OPT                         0     // DMA 1 Stream 4 Channel unknown
#define SDCARD_DMA_CHANNEL                      DMA_Channel_0
#endif

#define USE_VCP
//#define VBUS_SENSING_PIN PC5

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PD6
#define UART2_TX_PIN            PD5

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART6
#define UART6_RX_PIN            PC6
#define UART6_TX_PIN            PC7

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART2, USART3, USART6

// TODO
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA2

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

/// I2C Configuration
#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define I2C_DEVICE              (I2CDEV_2)

#define USE_ADC
#define VBAT_ADC_PIN            PC1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

#define TARGET_IO_PORTA (0xffff & ~(BIT(0)|BIT(1)|BIT(10)|BIT(13)|BIT(14)|BIT(15)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)|BIT(3)|BIT(4)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)|BIT(13)))
#define TARGET_IO_PORTD (0xffff & ~(BIT(0)|BIT(1)))
#define TARGET_IO_PORTE (0xffff & ~(BIT(15)))

#define USABLE_TIMER_CHANNEL_COUNT 10

// Used Timers
//#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(12))
// Defined Timers in timer.c
#define USED_TIMERS ( TIM_N(1) | TIM_N(3) | TIM_N(4))
