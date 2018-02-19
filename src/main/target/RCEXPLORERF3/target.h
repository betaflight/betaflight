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

#define TARGET_BOARD_IDENTIFIER "REF3"

//Making it fit into flash:
#undef USE_RTC_TIME
#undef USE_COPY_PROFILE_CMS_MENU
#undef USE_RX_MSP


#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define LED0_PIN    PB4
#define LED1_PIN    PB5

#define BEEPER      PA0
#define BEEPER_INVERTED

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define MPU_INT_EXTI PA15

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000

#define MPU6000_CS_PIN          PB12
#define MPU6000_SPI_INSTANCE    SPI2

#define USE_ACC

#define ACC_MPU6000_ALIGN CW180_DEG
#define GYRO_MPU6000_ALIGN CW180_DEG

#define USE_BARO
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883 // External

#define MAG_AK8975_ALIGN CW180_DEG

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN           PA6   // RC_CH7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
#define RANGEFINDER_HCSR04_ECHO_PIN              PB1   // RC_CH8 (PB1) - only 3.3v ( add a 1K Ohms resistor )

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8  // (HARDARE=0)

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)

#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define ADC_INSTANCE                ADC2
#define VBAT_ADC_PIN                PA5
#define CURRENT_METER_ADC_PIN       PB2
#define RSSI_ADC_PIN                PA6

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

#define USE_GPS
#define USE_GPS_UBLOX
#define USE_GPS_NMEA

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(17))
