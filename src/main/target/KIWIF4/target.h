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

#define TARGET_BOARD_IDENTIFIER "KIWI"
#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)

#define USBD_PRODUCT_STRING     "KIWIF4"

#define LED0                    PB5
#define LED1                    PB4


#define BEEPER                  PA8

#define INVERTER_PIN_USART1 PC0 // PC0 used as inverter select GPIO

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define GYRO

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE                SPI2
#define MAX7456_SPI_CS_PIN                  PB12
//#define MAX7456_DMA_CHANNEL_TX              DMA1_Stream5
//#define MAX7456_DMA_CHANNEL_RX              DMA1_Stream0
//#define MAX7456_DMA_IRQ_HANDLER_ID          DMA1_ST0_HANDLER


#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           SPI3_NSS_PIN
#define M25P16_SPI_INSTANCE     SPI3

#define USE_VCP
#define VBUS_SENSING_PIN        PC5
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

#define SERIAL_PORT_COUNT       4 //VCP, USART1, USART3, USART6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_DSHOT

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12



/* #define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)  // PB6-SCL, PB7-SDA
#define USE_I2C_PULLUP
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7 */

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER
#define VBAT_ADC_PIN            PC1
#define RSSI_ADC_PIN            PC2
#define CURRENT_METER_ADC_PIN   PC3

#define DEFAULT_FEATURES        (FEATURE_VBAT | FEATURE_BLACKBOX)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define LED_STRIP


#define SPEKTRUM_BIND
// USART3 Rx, PB11
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 12

#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) )

#define CMS
#define USE_MSP_DISPLAYPORT

