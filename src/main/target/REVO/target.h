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

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)

#if defined(AIRBOTF4)
#define TARGET_BOARD_IDENTIFIER "AIR4"
#define USBD_PRODUCT_STRING     "AirbotF4"

#elif defined(REVOLT)
#define TARGET_BOARD_IDENTIFIER "RVLT"
#define USBD_PRODUCT_STRING     "Revolt"

#elif defined(SOULF4)
#define TARGET_BOARD_IDENTIFIER "SOUL"
#define USBD_PRODUCT_STRING     "DemonSoulF4"

#else
#define TARGET_BOARD_IDENTIFIER "REVO"
#define USBD_PRODUCT_STRING     "Revolution"

#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000"
#endif

#endif

#define USE_DSHOT
#define USE_ESC_SENSOR

#define LED0                    PB5
// Disable LED1, conflicts with AirbotF4/Flip32F4/Revolt beeper
#if defined(AIRBOTF4)
#define BEEPER                  PB4
#define BEEPER_INVERTED
#elif defined(REVOLT)
#define BEEPER                  PB4
#elif defined(SOULF4)
#define BEEPER                  PB6
#define BEEPER_INVERTED
#else
#define LED1                    PB4
// Leave beeper here but with none as io - so disabled unless mapped.
#define BEEPER                  NONE
#endif

// PC0 used as inverter select GPIO
#define INVERTER                PC0
#define INVERTER_USART          USART1

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1

#if defined(SOULF4)
#define ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#elif defined(REVOLT)

#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW0_DEG

#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW0_DEG

#else
#define ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG

#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG

#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#endif

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#if !defined(AIRBOTF4) && !defined(REVOLT) && !defined(SOULF4)
#define MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG

#define BARO
#define USE_BARO_MS5611

#endif

#define M25P16_CS_PIN           PB3
#define M25P16_SPI_INSTANCE     SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
#define VBUS_SENSING_PIN        PC5

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

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2
//#define RSSI_ADC_PIN            PA0

#define LED_STRIP

#define SENSORS_SET             (SENSOR_ACC)

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_BLACKBOX)

#define SPEKTRUM_BIND
// USART3,
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#ifdef REVOLT
#define USABLE_TIMER_CHANNEL_COUNT 11
#else
#define USABLE_TIMER_CHANNEL_COUNT 12
#endif

#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) )
