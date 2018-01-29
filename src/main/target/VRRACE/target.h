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

#define TARGET_BOARD_IDENTIFIER "VRRA"

#define USBD_PRODUCT_STRING "VRRACE"

#define LED0_PIN PD14
#define LED1_PIN PD15
#define BEEPER PA0
#define BEEPER_INVERTED

#define INVERTER_PIN_UART6 PD7

#define MPU6500_CS_PIN        PE10
#define MPU6500_SPI_INSTANCE  SPI2

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN CW270_DEG

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN CW270_DEG

// MPU6500 interrupts
#define USE_EXTI
#define MPU_INT_EXTI PD10
#define USE_MPU_DATA_READY_SIGNAL

/*
#define USE_BARO
#define USE_BARO_MS5611
#define MS5611_I2C_INSTANCE I2CDEV_1

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PD2
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   PB12
*/

/*
#define SDCARD_DETECT_PIN                   PD2

#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PB3
*/

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
/*
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
*/
// Divide to under 25MHz for normal operation:
/*
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz
*/

/*
#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL                  0
*/


/*
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN         PB3
#define M25P16_SPI_INSTANCE   SPI3
*/

#define USE_VCP
#define VBUS_SENSING_PIN PA9
//#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN PB7
#define UART1_TX_PIN PB6

#define USE_UART2
#define UART2_RX_PIN PD6
#define UART2_TX_PIN PD5

#define USE_UART3
#define UART3_RX_PIN PD9
#define UART3_TX_PIN PD8

#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

#define USE_SOFTSERIAL1

#define SOFTSERIAL1_RX_PIN      PE13 // PWM 3
#define SOFTSERIAL1_TX_PIN      PE11 // PWM 2

#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 8 //VCP, USART1, USART2, USART3, USART6, SOFTSERIAL x 2

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PE9  // (HARDARE=0,PPM)


#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PE10
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

/*
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define USE_I2C_PULLUP
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
*/

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define VBAT_ADC_PIN                PC0
#define RSSI_ADC_PIN                PB1
#define CURRENT_METER_ADC_PIN       PA5

#undef USE_LED_STRIP

#define DEFAULT_FEATURES        (FEATURE_SOFTSERIAL | FEATURE_TELEMETRY)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

//#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) )
