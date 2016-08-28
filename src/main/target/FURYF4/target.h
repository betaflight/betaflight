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

#define TARGET_BOARD_IDENTIFIER "FYF4"
#define TARGET_CONFIG

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)

#define USBD_PRODUCT_STRING     "FuryF4"

#define LED0                    PB5
#define LED1                    PB4

#define BEEPER                  PA8
#define BEEPER_INVERTED

#define INVERTER PC0 // PC0 used as inverter select GPIO
#define INVERTER_USART USART1

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready (mag disabled)
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG

#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW180_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG

#define BARO
#define USE_BARO_MS5611
#define MS5611_I2C_INSTANCE     I2CDEV_1

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PD2
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   PB12

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz


//#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
//#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF5
//#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
//#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF4
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           PB3
#define M25P16_SPI_INSTANCE     SPI3

#define USABLE_TIMER_CHANNEL_COUNT 5

#define USE_VCP
#define VBUS_SENSING_PIN        PC5
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       4 //VCP, USART1, USART3, USART6

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)  // PB6-SCL, PB7-SDA
#define USE_I2C_PULLUP
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER
#define VBAT_ADC_PIN            PC1
#define RSSI_ADC_GPIO_PIN       PC2
#define CURRENT_METER_ADC_PIN   PC3

#define DEFAULT_FEATURES        (FEATURE_VBAT | FEATURE_BLACKBOX)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define LED_STRIP
#define WS2811_PIN                      PA0
#define WS2811_TIMER                    TIM5
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_IT                   DMA_IT_TCIF2
#define WS2811_DMA_CHANNEL              DMA_Channel_6
#define WS2811_TIMER_CHANNEL            TIM_Channel_1

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SPEKTRUM_BIND
// USART3 Rx, PB11
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(9))

