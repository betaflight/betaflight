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

#define TARGET_BOARD_IDENTIFIER "ANY7"

#define USBD_PRODUCT_STRING "AnyFCF7"

#define LED0   PB7
#define LED1   PB6

#define BEEPER   PB4
#define BEEPER_INVERTED

#define MPU6000_CS_PIN        PA4
#define MPU6000_SPI_INSTANCE  SPI1

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW270_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define MPU_INT_EXTI PC4
#define USE_EXTI

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE I2C_DEVICE_EXT
#define MAG_HMC5883_ALIGN CW270_DEG_FLIP
//#define MAG_HMC5883_ALIGN CW90_DEG

#define BARO
#define USE_BARO_MS5611

#ifdef ANYFCF7_EXTERNAL_BARO
    #define USE_BARO_BMP085
    #define USE_BARO_BMP280
    #define BARO_I2C_INSTANCE I2C_DEVICE_EXT
#endif

#define USE_PITOT_MS4525
#define PITOT_I2C_INSTANCE I2C_DEVICE_EXT

#define USABLE_TIMER_CHANNEL_COUNT 16

#define USB_IO
#define USE_VCP
#define VBUS_SENSING_PIN PA8

#define USE_UART1
#define UART1_RX_PIN PA10
#define UART1_TX_PIN PA9

#define USE_UART2
#define UART2_RX_PIN PD6
#define UART2_TX_PIN PD5

#define USE_UART3
#define UART3_RX_PIN PD9
#define UART3_TX_PIN PD8

#define USE_UART4
#define UART4_RX_PIN PC11
#define UART4_TX_PIN PC10

#define USE_UART5
#define UART5_RX_PIN PD2
#define UART5_TX_PIN PC12

#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

#define USE_UART7
#define UART7_RX_PIN PE7
#define UART7_TX_PIN PE8

#define USE_UART8
#define UART8_RX_PIN PE0
#define UART8_TX_PIN PE1

#define SERIAL_PORT_COUNT 9 //VCP, USART1, USART2, USART3, UART4, UART5, USART6, USART7, USART8

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_4

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI4_NSS_PIN            PE11
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14

#define USE_SDCARD

// This is needed for BangGood board that used wrong sdcard socket!!!
#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PD3
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line3
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource3
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOD
#define SDCARD_DETECT_EXTI_IRQn             EXTI3_IRQn

#define SDCARD_SPI_INSTANCE                 SPI4
#define SDCARD_SPI_CS_PIN                   SPI4_NSS_PIN

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 422kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 8 // 27MHz

#define SDCARD_DMA_CHANNEL_TX               DMA2_Stream1
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF1_5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA2
#define SDCARD_DMA_CHANNEL                  DMA_CHANNEL_4

#define USE_I2C
#define USE_I2C_DEVICE_4
#define I2C_DEVICE (I2CDEV_4)
#define I2C_DEVICE_EXT (I2CDEV_2)
//#define USE_I2C_PULLUP

//#define HIL

#define MAG_GPS_ALIGN           CW180_DEG_FLIP

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define NAV
#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC2
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define LED_STRIP

// LED Strip can run off Pin 6 (PA0) of the ESC outputs.
#define WS2811_PIN                      PA1
#define WS2811_TIMER                    TIM5
#define WS2811_TIMER_CHANNEL            TIM_CHANNEL_2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST4_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream4
#define WS2811_DMA_FLAG                 DMA_FLAG_TCIF4
#define WS2811_DMA_IT                   DMA_IT_TCIF4
#define WS2811_DMA_CHANNEL              DMA_CHANNEL_6
#define WS2811_DMA_IRQ                  DMA1_Stream4_IRQn
#define WS2811_TIMER_GPIO_AF            GPIO_AF2_TIM5

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_FEATURES        (FEATURE_BLACKBOX)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    15

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11))
