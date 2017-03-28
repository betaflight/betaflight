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

#if defined(CL_RACINGF4)
#define TARGET_BOARD_IDENTIFIER "CLR4"
#elif defined(OMNIBUSF4SD)
#define TARGET_BOARD_IDENTIFIER "OBSD"
#elif defined(VGOODDHF4)
#define TARGET_BOARD_IDENTIFIER "DHF4"
#else
#define TARGET_BOARD_IDENTIFIER "OBF4"
#endif

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)

#if defined(CL_RACINGF4)
#define USBD_PRODUCT_STRING "CL_RACINGF4"
#elif defined(VGOODDHF4)
#define USBD_PRODUCT_STRING "VgooddhF4"
#else
#define USBD_PRODUCT_STRING "OmnibusF4"
#endif

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000" // Remove this at the next major release (?)
#endif

#define LED0                    PB5
//#define LED1                    PB4 // Remove this at the next major release
#define BEEPER                  PB4
#define BEEPER_INVERTED

#ifdef OMNIBUSF4SD
#define INVERTER_PIN_UART6      PC8 // Omnibus F4 V3 and later
#else
#define INVERTER_PIN_UART1      PC0 // PC0 used as inverter select GPIO XXX this is not used --- remove it at the next major release
#endif

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define ACC
#define USE_ACC_SPI_MPU6000

#define GYRO
#define USE_GYRO_SPI_MPU6000

#if defined(CL_RACINGF4)
#define GYRO_MPU6000_ALIGN      CW0_DEG
#define ACC_MPU6000_ALIGN       CW0_DEG
#elif defined(OMNIBUSF4SD)
#define GYRO_MPU6000_ALIGN      CW270_DEG
#define ACC_MPU6000_ALIGN       CW270_DEG
#else
#define GYRO_MPU6000_ALIGN      CW180_DEG
#define ACC_MPU6000_ALIGN       CW180_DEG
#endif

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG

//#define USE_MAG_NAZA                   // Delete this on next major release
//#define MAG_NAZA_ALIGN CW180_DEG_FLIP  // Ditto

#define BARO
#define USE_BARO_MS5611
#if defined(OMNIBUSF4SD)
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI3
#define BMP280_CS_PIN           PB3 // v1
#endif

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#if defined(OMNIBUSF4SD) || defined(CL_RACINGF4)
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define USE_SDCARD
#define USE_SDCARD_SPI2
#if defined(OMNIBUSF4SD)
#define SDCARD_DETECT_INVERTED
#endif
#define SDCARD_DETECT_PIN               PB7
#define SDCARD_SPI_INSTANCE             SPI2
#define SDCARD_SPI_CS_PIN               SPI2_NSS_PIN
// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF4
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0
#elif defined(VGOODDHF4)
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2
#define USE_FLASHFS
#define USE_FLASH_M25P16
#else
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN           SPI3_NSS_PIN
#define M25P16_SPI_INSTANCE     SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16
#endif // OMNIBUSF4

#define USE_VCP
#define VBUS_SENSING_PIN PC5

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

#if defined(CL_RACINGF4)
#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART3,USART4, USART6,
#else
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3, USART6, SOFTSERIAL x 2
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_SPI
#define USE_SPI_DEVICE_1

#if defined(OMNIBUSF4SD) || defined(CL_RACINGF4) || defined(VGOODDHF4)
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#endif

#define USE_SPI_DEVICE_3
#if defined(OMNIBUSF4SD) || defined(CL_RACINGF4)
  #define SPI3_NSS_PIN            PA15
#else
  #define SPI3_NSS_PIN            PB3
#endif
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

//#define USE_I2C
//#define I2C_DEVICE (I2CDEV_1)

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2
#if defined(CL_RACINGF4)
#define RSSI_ADC_PIN            PC3
#else
//#define RSSI_ADC_PIN            PA0
#endif

#define USE_ESC_SENSOR

#define LED_STRIP

#define SENSORS_SET (SENSOR_ACC)

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

#define AVOID_UART1_FOR_PWM_PPM
#if defined(CL_RACINGF4)
#define DEFAULT_FEATURES         (FEATURE_BLACKBOX |FEATURE_CURRENT_METER | FEATURE_TELEMETRY| FEATURE_VBAT | FEATURE_OSD )
#else
#define DEFAULT_FEATURES        (FEATURE_BLACKBOX | FEATURE_VBAT | FEATURE_OSD)
#endif

#define SPEKTRUM_BIND
#define BIND_PIN                UART1_RX_PIN

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

#ifdef CL_RACINGF4
#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS  ( TIM_N(4) | TIM_N(8) )
#else
#ifdef OMNIBUSF4SD
#define USABLE_TIMER_CHANNEL_COUNT 13
#else
#define USABLE_TIMER_CHANNEL_COUNT 12
#endif
#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9))
#endif
