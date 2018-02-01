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

#if defined(AIRBOTF4)
#define TARGET_BOARD_IDENTIFIER "AIR4"
#define USBD_PRODUCT_STRING     "AirbotF4"

#elif defined(AIRBOTF4SD)
#define TARGET_BOARD_IDENTIFIER "A4SD"
#define USBD_PRODUCT_STRING     "AirbotF4SD"

#elif defined(REVOLT)
#define TARGET_BOARD_IDENTIFIER "RVLT"
#define USBD_PRODUCT_STRING     "Revolt"
#define TARGET_DEFAULT_MIXER    MIXER_QUADX_1234

#elif defined(SOULF4)
#define TARGET_BOARD_IDENTIFIER "SOUL"
#define USBD_PRODUCT_STRING     "DemonSoulF4"

#elif defined(PODIUMF4)
#define TARGET_BOARD_IDENTIFIER "PODI"
#define USBD_PRODUCT_STRING     "PodiumF4"

#else
#define TARGET_BOARD_IDENTIFIER "REVO"
#define USBD_PRODUCT_STRING     "Revolution"

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000"
#endif

#endif

#define LED0_PIN                PB5
#if defined(PODIUMF4)
#define LED1_PIN                PB4
#define LED2_PIN                PB6
#endif

// Disable LED1, conflicts with AirbotF4/Flip32F4/Revolt beeper
#if defined(AIRBOTF4) || defined(AIRBOTF4SD)
#define BEEPER                  PB4
#define BEEPER_INVERTED
#elif defined(REVOLT)
#define BEEPER                  PB4
#elif defined(SOULF4)
#define BEEPER                  PB6
#define BEEPER_INVERTED
#else
#define LED1_PIN                PB4
// Leave beeper here but with none as io - so disabled unless mapped.
#define BEEPER                  NONE
#endif

#if defined(REVOLT)
#define ENABLE_DSHOT_DMAR       true
#endif

// PC0 used as inverter select GPIO
#ifdef AIRBOTF4SD
#define INVERTER_PIN_UART6      PD2
#else
#define INVERTER_PIN_UART1      PC0
#endif

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_GYRO
#define USE_ACC

#ifdef AIRBOTF4SD
#undef MPU6000_CS_PIN
#define MPU6000_CS_PIN          PB13
#define USE_GYRO_SPI_ICM20601
#define ICM20601_CS_PIN         PA4 // served through MPU6500 code
#define ICM20601_SPI_INSTANCE   SPI1
#define USE_DUAL_GYRO
#define GYRO_0_CS_PIN           MPU6000_CS_PIN
#define GYRO_1_CS_PIN           ICM20601_CS_PIN
#endif

#if defined(SOULF4)
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG

#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#elif defined(REVOLT) || defined(PODIUMF4)

#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW0_DEG

#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW0_DEG

#else

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG

#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG

#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

#endif

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

// Configure MAG and BARO unconditionally.
#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#if defined(AIRBOTF4) || defined(AIRBOTF4SD)
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI1
#define BMP280_CS_PIN           PC13
#endif

#if defined(AIRBOTF4SD)
// SDCARD support for AIRBOTF4SD
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PC0
#define SDCARD_SPI_INSTANCE     SPI3
#define SDCARD_SPI_CS_PIN       SPI3_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
#define SDCARD_DMA_CHANNEL                  0

#else

#define M25P16_CS_PIN           PB3
#define M25P16_SPI_INSTANCE     SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16

#endif // AIRBOTF4SD


#define USE_VCP
#if defined(PODIUMF4)
#define VBUS_SENSING_PIN        PA8
#else
#define VBUS_SENSING_PIN        PC5
#endif

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#if defined(REVOLT) || defined(REVO)
#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0
#endif // REVOLT || REVO

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#if defined(REVOLT) || defined(REVO)
#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART3, UART4,  USART6, SOFTSERIAL x 2
#else
#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3, USART6, SOFTSERIAL x 2
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB14  // (HARDARE=0,PPM)

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#if defined(AIRBOTF4) || defined(AIRBOTF4SD)
// On AIRBOTF4 and AIRBOTF4SD, I2C2 and I2C3 are configurable
#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PB10, shared with UART3TX
#define I2C2_SDA                NONE // PB11, shared with UART3RX
#define USE_I2C_DEVICE_3
#define I2C3_SCL                NONE // PA8, PWM6
#define I2C3_SDA                NONE // PC9, CH6
#define I2C_DEVICE              (I2CDEV_2)
#else
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#endif

#define USE_ADC
#if !defined(PODIUMF4)
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2
#else
#define VBAT_ADC_PIN            PC3
#define VBAT_ADC_CHANNEL        ADC_Channel_13
#endif

#if defined(AIRBOTF4SD)
#define RSSI_ADC_PIN            PA0
#endif

#define USE_TRANSPONDER

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#if defined(PODIUMF4)
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define DEFAULT_FEATURES        FEATURE_TELEMETRY
#endif

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#ifdef REVOLT
#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(12) )
#elif defined(AIRBOTF4) || defined(AIRBOTF4SD)
#define USABLE_TIMER_CHANNEL_COUNT 13
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(12) )
#else
#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(12) )
#endif
