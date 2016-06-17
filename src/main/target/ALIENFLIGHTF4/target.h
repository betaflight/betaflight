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
#define TARGET_BOARD_IDENTIFIER "AFF4"

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SPEKTRUM2048
#define CONFIG_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 2

#define USBD_PRODUCT_STRING "AlienFlight F4"

#define LED0                 PC12
#define LED1                 PD2

#define BEEPER               PC13

#define INVERTER             PC15
#define INVERTER_USART       USART2

// MPU interrupt
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define MPU_INT_EXTI         PC14
#define USE_EXTI

#define MPU6500_CS_PIN       PA4
#define MPU6500_SPI_INSTANCE SPI1
#define MPU9250_CS_PIN       PA4
#define MPU9250_SPI_INSTANCE SPI1

#define ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU9250

#define ACC_MPU6500_ALIGN    CW270_DEG
#define ACC_MPU9250_ALIGN    CW270_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250

#define GYRO_MPU6500_ALIGN   CW270_DEG
#define GYRO_MPU9250_ALIGN   CW270_DEG

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_AK8963

#define MAG_HMC5883_ALIGN    CW180_DEG
#define MAG_AK8963_ALIGN     CW270_DEG

#define BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280

#define USE_SDCARD

//#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PB10
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line10
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource10
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOB
#define SDCARD_DETECT_EXTI_IRQn             EXTI15_10_IRQn

#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF4
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

//#define M25P16_CS_PIN        SPI2_NSS_PIN
//#define M25P16_SPI_INSTANCE  SPI2

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

#define USABLE_TIMER_CHANNEL_COUNT 13

#define USE_VCP

#define USE_USART1
#define USART1_RX_PIN        PA10
#define USART1_TX_PIN        PA9
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2

#define USE_USART2
#define USART2_RX_PIN        PA3
#define USART2_TX_PIN        PA2 //inverter

//#define USE_USART3
//#define USART3_RX_PIN        PB11
//#define USART3_TX_PIN        PB10

#define USE_USART4
#define USART4_RX_PIN        PC10
#define USART4_TX_PIN        PC11

//#define USE_USART5
//#define USART5_RX_PIN        PD2
//#define USART5_TX_PIN        PC12

#define SERIAL_PORT_COUNT    4

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN         PB12
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PC2
#define SPI2_MOSI_PIN        PC3

#define USE_SPI_DEVICE_3

#define USE_I2C
#define I2C_DEVICE           (I2CDEV_1)
//#define I2C_DEVICE_EXT       (I2CDEV_2)
#define I2C1_SCL             PB6
#define I2C1_SDA             PB7

#define USE_ADC
//#define BOARD_HAS_VOLTAGE_DIVIDER

#define VBAT_ADC_PIN               PC0
#define VBAT_ADC_CHANNEL           ADC_Channel_1

#define CURRENT_METER_ADC_PIN      PC1
#define CURRENT_METER_ADC_CHANNEL  ADC_Channel_0

#define RSSI_ADC_PIN               PC4
#define RSSI_ADC_CHANNEL           ADC_Channel_4

#define EXTERNAL1_ADC_GPIO_PIN     PC5
#define EXTERNAL1_ADC_CHANNEL      ADC_Channel_5

// LED strip configuration using RC5 pin.
//#define LED_STRIP
//#define LED_STRIP_TIMER               TIM8

//#define USE_LED_STRIP_ON_DMA1_CHANNEL3
//#define WS2811_GPIO                   GPIOB
//#define WS2811_GPIO_AHB_PERIPHERAL    RCC_AHBPeriph_GPIOB
//#define WS2811_GPIO_AF                GPIO_AF_3
//#define WS2811_PIN                    GPIO_Pin_15 // TIM8_CH3
//#define WS2811_PIN_SOURCE             GPIO_PinSource15
//#define WS2811_TIMER                  TIM8
//#define WS2811_TIMER_APB2_PERIPHERAL  RCC_APB2Periph_TIM8
//#define WS2811_DMA_CHANNEL            DMA1_Channel3
//#define WS2811_IRQ                    DMA1_Channel3_IRQn

// alternative defaults for AlienFlight F4 target
#define ALIENFLIGHT

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN             PA3

#define HARDWARE_BIND_PLUG
// Hardware bind plug at PB2 (Pin 28)
#define BINDPLUG_PIN         PB2

#define BRUSHED_MOTORS
#define DEFAULT_RX_FEATURE FEATURE_RX_PPM
#define DEFAULT_FEATURES (FEATURE_RX_SERIAL | FEATURE_MOTOR_STOP)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA      0xffff
#define TARGET_IO_PORTB      0xffff
#define TARGET_IO_PORTC      0xffff
#define TARGET_IO_PORTD      (BIT(2))

#define USED_TIMERS  ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8))
#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8)
