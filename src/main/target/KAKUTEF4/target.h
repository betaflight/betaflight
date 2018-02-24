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
#if defined(KAKUTEF4V2)
#define TARGET_BOARD_IDENTIFIER "KTV2"
#define USBD_PRODUCT_STRING "KakuteF4-V2"
#else
#define TARGET_BOARD_IDENTIFIER "KTV1"
#define USBD_PRODUCT_STRING "KakuteF4-V1"
#endif

#define USE_TARGET_CONFIG

#define LED0_PIN                PB5
#define LED1_PIN                PB4
#define LED2_PIN                PB6

#define BEEPER                  PC9
#define BEEPER_INVERTED
#define INVERTER_PIN_UART3      PB15

// ICM20689 interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PC5
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define  ICM20689_CS_PIN          PC4
#define ICM20689_SPI_INSTANCE    SPI1

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define ACC_ICM20689_ALIGN       CW270_DEG

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_ICM20689_ALIGN      CW270_DEG

#ifdef KAKUTEF4V2        // There is invertor on RXD3(PB11), so PB10/PB11 can't be used as I2C2.
#define USE_I2C          //No other I2C pins are  fanned out, So V1 don't support I2C  peripherals.
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8        // SCL pad
#define I2C1_SDA                PB9        // SDA pad
#define BARO_I2C_INSTANCE       I2C_DEVICE
#define MAG_I2C_INSTANCE        I2C_DEVICE

#define USE_MAG
#define USE_MAG_HMC5883                   //External, connect to I2C1
#define MAG_HMC5883_ALIGN       CW180_DEG

#define USE_BARO
#define USE_BARO_MS5611                  //External, connect to I2C1
#define USE_BARO_BMP280                  //onboard
#endif

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PB14
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define M25P16_CS_PIN           PB3
#define M25P16_SPI_INSTANCE     SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
#define VBUS_SENSING_PIN        PA8
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#ifdef KAKUTEF4V2                // Uart4 and Uart5 are fanned out on v2
#define USE_UART4                // Uart4 can be used for GPS or  RunCam Split
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5               //Uart5 can be used for ESC sensor
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            NONE

#define USE_SOFTSERIAL1         //M1~M4 and LedTrip can be redefined as Softserial
#define SERIAL_PORT_COUNT 7     //vcp, uart1, uart3, uart4, uart5, uart6, softSerial1
#else
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 6   //vcp, uart1, uart3,, uart6, softSerial1, softSerial2
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PC7  // (HARDARE=0,PPM)

#define USE_SPI

#define USE_SPI_DEVICE_1 //ICM20689
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3 //dataflash
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define VBAT_ADC_PIN                PC3
#define VBAT_ADC_CHANNEL            ADC_Channel_13

#define CURRENT_METER_ADC_PIN       PC2
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_12

#define RSSI_ADC_PIN                PC1
#define RSSI_ADC_CHANNEL            ADC_Channel_11

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD )
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3
#define TELEMETRY_UART          SERIAL_PORT_USART1

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD        (BIT(2))

#ifdef KAKUTEF4V2
#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) |  TIM_N(8))
#else
#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5)  |  TIM_N(8))
#endif
