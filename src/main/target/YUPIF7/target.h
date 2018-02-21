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
#define TARGET_BOARD_IDENTIFIER "YPF7"

#define USBD_PRODUCT_STRING     "YUPIF7"

#define LED0_PIN                PB4

//define camera control
#define CAMERA_CONTROL_PIN      PB7

#define BEEPER                  PB14
#define BEEPER_PWM_HZ           3150 // Beeper PWM frequency in Hz

// Gyro interrupt
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define MPU_INT_EXTI            PC4

// ICM 20689
#define ICM20689_CS_PIN         PA4
#define ICM20689_SPI_INSTANCE   SPI1

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define ACC_ICM20689_ALIGN      CW90_DEG

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_ICM20689_ALIGN     CW90_DEG

// Serial ports
#define USE_VCP
#define VBUS_SENSING_PIN        PA8

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_RX_PIN      PB0 // PWM5
#define SOFTSERIAL1_TX_PIN      PB1 // PWM7

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3, USART5, USART6, SOFTSERIAL1

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PC8 // (Hardware=0, PPM)

//SPI ports
#define USE_SPI

#define USE_SPI_DEVICE_1 //Gyro & OSD
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3 //Dataslash
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PB5

/* I2C Port
#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_1
#define I2C2_SCL                PB8
#define I2C2_SDA                PB9
#define I2C_DEVICE              (I2CDEV_1)
*/

// OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PA14
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

// Dataflash
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           SPI3_NSS_PIN
#define M25P16_SPI_INSTANCE     SPI3

// ADC inputs
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define RSSI_ADC_GPIO_PIN               PC0
#define VBAT_ADC_PIN                    PC1
#define CURRENT_METER_ADC_PIN           PC2
#define CURRENT_METER_SCALE_DEFAULT     235

// Default configuration
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define TELEMETRY_UART          SERIAL_PORT_USART1
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD)

// Target IO and timers
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12))
