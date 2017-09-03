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

#define TARGET_BOARD_IDENTIFIER "MKF4"

#define USBD_PRODUCT_STRING  "MatekF4"

#define LED0                    PB9
#define LED1                    PA14

#define BEEPER                  PC13
#define BEEPER_INVERTED

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN   	    PA6
#define SPI1_MOSI_PIN   	    PA7

#define MPU6500_CS_PIN          PC2
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_EXTI
#define MPU_INT_EXTI            PC3
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW180_DEG

#define ACC
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG

// *************** SD Card **************************
#define USE_SDCARD
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN   	    PB4
#define SPI3_MOSI_PIN   	    PB5

#define SDCARD_SPI_INSTANCE     SPI3
#define SDCARD_SPI_CS_PIN       PC1

// SPI3 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               	DMA1_Stream7
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG 	DMA_FLAG_TCIF7
#define SDCARD_DMA_CLK                      	RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  	DMA_Channel_0

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN   	    PB14
#define SPI2_MOSI_PIN   	    PB15

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB10
// *************** UART *****************************
#define USB_IO
#define USE_VCP
#define VBUS_SENSING_PIN        PB12
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

//#define USE_SOFTSERIAL1
//#define SOFTSERIAL1_RX_PIN      PA15 // S5
//#define SOFTSERIAL1_TX_PIN      PA8  // S6

#define SERIAL_PORT_COUNT       6

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

// *************** I2C ****************************
// SLC clash with WS2812 LED
#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7


#define BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085

#define MAG
#define USE_MAG_MAG3110 // External
#define USE_MAG_HMC5883 // External
#define USE_MAG_AK8963  // External
#define USE_MAG_AK8975  // External
#define USE_MAG_QMC5883 // External

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1
#define ADC1_DMA_STREAM             DMA2_Stream0
#define ADC_CHANNEL_1_PIN           PC5
#define ADC_CHANNEL_2_PIN           PC4
#define ADC_CHANNEL_3_PIN           PB1
#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

#define DEFAULT_FEATURES        (FEATURE_OSD )

#define LED_STRIP
#define WS2811_PIN                      PA15 // S5 pad for iNav
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST5_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream5
#define WS2811_DMA_CHANNEL              DMA_Channel_3   // ???

#define SPEKTRUM_BIND
#define BIND_PIN                PA1 // USART4 RX

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define MAX_PWM_OUTPUT_PORTS       6
#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(5)|TIM_N(8))
