/*
 * Supports the GY-91 MPU9250 and BMP280 development board via SPI1
 *
 * Put the MAX7456 on SPI2 instead of an SDCARD
 *  MAX7456 CS -> PB12 (default)
 *  Uses the default pins for SPI2:
 *    #define SPI2_NSS_PIN    PB12
 *    #define SPI2_SCK_PIN    PB13
 *    #define SPI2_MISO_PIN   PB14
 *    #define SPI2_MOSI_PIN   PB15
 *
 * @author Nathan Tsoi
 *
 * This software is free software: you can redistribute it and/or modify
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

#define TARGET_BOARD_IDENTIFIER "SDF3" // STM Discovery F3

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define LED0                    PE8 // Blue LEDs - PE8/PE12
#define LED0_INVERTED
#define LED1                    PE10  // Orange LEDs - PE10/PE14
#define LED1_INVERTED

#define BEEPER                  PE9 // Red LEDs - PE9/PE13
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//#define USE_SD_CARD
//
//#define SD_DETECT_PIN           PC14
//#define SD_CS_PIN               PB12
//#define SD_SPI_INSTANCE         SPI2

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

//#define M25P16_CS_GPIO          GPIOB
//#define M25P16_CS_PIN           GPIO_Pin_12
//#define M25P16_SPI_INSTANCE     SPI2
// SPI1
// PB5  SPI1_MOSI
// PB4  SPI1_MISO
// PB3  SPI1_SCK
// PA15 SPI1_NSS

// SPI2
// PB15 SPI2_MOSI
// PB14 SPI2_MISO
// PB13 SPI2_SCK
// PB12 SPI2_NSS

#define GYRO
#define USE_GYRO_L3GD20
#define L3GD20_SPI              SPI1
#define L3GD20_CS_PIN           PE3
#define GYRO_L3GD20_ALIGN       CW270_DEG

// Support the GY-91 MPU9250 dev board
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define MPU6500_CS_PIN          PC14
#define MPU6500_SPI_INSTANCE    SPI2
#define GYRO_MPU6500_ALIGN      CW270_DEG_FLIP

#define ACC
#define USE_ACC_LSM303DLHC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG_FLIP

//#define BARO
//#define BMP280_CS_PIN         PB12
//#define BMP280_SPI_INSTANCE   SPI2
//#define USE_BARO_BMP280
//#define USE_BARO_SPI_BMP280

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

//#define USE_SDCARD
//#define USE_SDCARD_SPI2
//
//#define SDCARD_SPI_INSTANCE     SPI2
//#define SDCARD_SPI_CS_PIN       PB12
//// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
//#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
//// Divide to under 25MHz for normal operation:
//#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 2
//
//// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
//#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
//#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA1_FLAG_TC5

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define MAG
#define USE_MAG_HMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define SERIAL_PORT_COUNT       3

// uart2 gpio for shared serial rx/ppm
//#define UART2_TX_PIN        GPIO_Pin_5 // PD5
//#define UART2_RX_PIN        GPIO_Pin_6 // PD6
//#define UART2_GPIO          GPIOD
//#define UART2_GPIO_AF       GPIO_AF_7
//#define UART2_TX_PINSOURCE  GPIO_PinSource5
//#define UART2_RX_PINSOURCE  GPIO_PinSource6

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#define LED_STRIP
#define WS2811_PIN                      PB8 // TIM16_CH1
#define WS2811_TIMER                    TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#define LED_STRIP_TIMER                 TIM16

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - 303 in 100pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         0x00ff

#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16) | TIM_N(17))

