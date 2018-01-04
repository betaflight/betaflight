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

#define TARGET_BOARD_IDENTIFIER "CLBR"
#define BST_DEVICE_NAME         "COLIBRI RACE"
#define BST_DEVICE_NAME_LENGTH  12
#define TARGET_BUS_INIT

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define LED0_PIN                PC15
#define LED1_PIN                PC14
#define LED2_PIN                PC13

#define BEEPER                  PB13
#define BEEPER_INVERTED

// MPU6500 interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PA5
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5
#define SPI1_NSS_PIN            PA4

#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI1

#define MPU6000_CS_PIN          SPI1_NSS_PIN
#define MPU6000_SPI_INSTANCE    SPI1

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

#define USE_BARO
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_AK8963
#define USE_MAG_AK8975

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
/* USE_SOFTSERIALx has a conflict with USE_BST that prevents FC from being recognized from configurator
 Drop USE_SOFTSERIALx until resolution is provided.
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       6
*/
#define SERIAL_PORT_COUNT       4

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8  // (HARDARE=0,PPM)

#define UART1_TX_PIN            PC4
#define UART1_RX_PIN            PC5

#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL_PIN            PA9
#define I2C2_SDA_PIN            PA10

#define USE_BST
#define BST_DEVICE              (BSTDEV_1)
/* Configure the CRC peripheral to use the polynomial x8 + x7 + x6 + x4 + x2 + 1 */
#define BST_CRC_POLYNOM         0xD5

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#define DEFAULT_FEATURES        (FEATURE_AIRMODE | FEATURE_LED_STRIP)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15))
