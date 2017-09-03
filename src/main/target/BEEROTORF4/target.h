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
#define TARGET_BOARD_IDENTIFIER "BRF4"

#define USBD_PRODUCT_STRING "BeeRotorF4"

#define LED0                    PB4

#define BEEPER                  PB3
#define BEEPER_INVERTED

// ICM20689 interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PA8
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

//#define ICM20689_CS_PIN          SPI1_NSS_PIN
//#define ICM20689_SPI_INSTANCE    SPI1
#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI1

#define GYRO
//#define USE_GYRO_SPI_ICM20689
//#define GYRO_ICM20689_ALIGN      CW270_DEG
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define ACC
//#define USE_ACC_SPI_ICM20689
//#define ACC_ICM20689_ALIGN       CW270_DEG
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

#define BARO
#define USE_BARO_BMP280

#define MAG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG
#define USE_MAG_MAG3110
#define USE_MAG_QMC5883

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      SPI3_NSS_PIN
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

//#define MAX7456_DMA_CHANNEL_TX              DMA1_Stream5
//#define MAX7456_DMA_CHANNEL_RX              DMA1_Stream0
//#define MAX7456_DMA_IRQ_HANDLER_ID          DMA1_ST0_HANDLER

#define USE_SDCARD
#define USE_SDCARD_SPI2

#define SDCARD_DETECT_PIN       PC3
#define SDCARD_DETECT_INVERTED
#define SDCARD_SPI_INSTANCE     SPI2
#define SDCARD_SPI_CS_PIN       SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream4
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF4
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

#define USB_IO
#define USE_VCP
#define VBUS_SENSING_ENABLED
#define VBUS_SENSING_PIN        PC5

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

//SerialRX
#define USE_UART2
#define UART2_RX_PIN            PA3 //Shared with PPM
#define UART2_TX_PIN            PA2

#define INVERTER_PIN_UART2      PC15

//Telemetry
#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define INVERTER_PIN_UART3      PC14

#define SERIAL_PORT_COUNT 4

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_SPI

//ICM20689
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

//SDCard
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//MAX7456 / SPI RX
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
//#define SPI_RX_CS_PIN           PD2

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC2
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define LED_STRIP
#define WS2811_GPIO_AF                  GPIO_AF_TIM4
#define WS2811_PIN                      PB8
#define WS2811_TIMER                    TIM4
#define WS2811_TIMER_CHANNEL            TIM_Channel_3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST7_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream7
#define WS2811_DMA_CHANNEL              DMA_Channel_2
#define WS2811_DMA_IRQ                  DMA1_Stream7_IRQn
#define WS2811_DMA_FLAG                 DMA_FLAG_TCIF7
#define WS2811_DMA_IT                   DMA_IT_TCIF7

//#define TRANSPONDER
//#define TRANSPONDER_GPIO_AF                  GPIO_AF_TIM4
//#define TRANSPONDER_PIN                      PB8
//#define TRANSPONDER_TIMER                    TIM4
//#define TRANSPONDER_TIMER_CHANNEL            TIM_Channel_3
//#define TRANSPONDER_DMA_HANDLER_IDENTIFER    DMA1_ST7_HANDLER
//#define TRANSPONDER_DMA_STREAM               DMA1_Stream7
//#define TRANSPONDER_DMA_CHANNEL              DMA_Channel_6
//#define TRANSPONDER_DMA_IRQ                  DMA1_Stream7_IRQn
//#define TRANSPONDER_DMA_FLAG                 DMA_FLAG_TCIF7
//#define TRANSPONDER_DMA_IT                   DMA_IT_TCIF7

#define DEFAULT_FEATURES        ( FEATURE_BLACKBOX | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_AIRMODE )
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DISABLE_RX_PWM_FEATURE
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define MAX_PWM_OUTPUT_PORTS    8

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(9) | TIM_N(11) )
