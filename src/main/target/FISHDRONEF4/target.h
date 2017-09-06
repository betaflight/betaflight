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

#define TARGET_BOARD_IDENTIFIER "FDV1"

#define USBD_PRODUCT_STRING "FishDroneF4NAV"

// *************** LED *****************************
#define LED0                    PC13    // Red
#define LED1                    PC14    // Yellow

// *************** BEEPER *****************************
#define BEEPER                  PC15

// *************** INVERTER *****************************
#define INVERTER_PIN_UART2     PB2

// *************** SPI *****************************
#define USE_SPI

// *************** ICM20608 *****************************
#define USE_SPI_DEVICE_1
#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1

#define ACC
#define USE_ACC_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW180_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG

// MPU6500 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

// *************** Compass *****************************
#define MAG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_MAG3110
#define USE_MAG_HMC5883
#define USE_MAG_IST8310
#define USE_MAG_QMC5883
#define MAG_IST8310_ALIGN CW270_DEG

// *************** BARO *****************************
#define BARO
#define USE_BARO_MS5611
#define MS5611_I2C_INSTANCE     I2CDEV_1

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PC2
#define SPI2_MOSI_PIN   PC3

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

// *************** TF Support *****************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#define SPI3_NSS_PIN    PB6

#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PB7
#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PB6

// SPI3 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

// *************** Flash *****************************
#define M25P16_CS_PIN           PA15
#define M25P16_SPI_INSTANCE     SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16

// *************** UART *****************************
#define USB_IO
#define USE_VCP

// provide for Telemetry module
#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

// provide for xBUS Receiver
#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

// provide for GPS module
#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define SERIAL_PORT_COUNT       4 // VCP, USART1, USART2, USART5

// *************** WS2811 *****************************
#define LED_STRIP
#define WS2811_PIN                      PB1
#define WS2811_TIMER                    TIM3
#define WS2811_TIMER_CHANNEL            TIM_Channel_4
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_FLAG                 DMA_FLAG_TCIF2
#define WS2811_DMA_IT                   DMA_IT_TCIF2
#define WS2811_DMA_CHANNEL              DMA_Channel_5
#define WS2811_DMA_IRQ                  DMA1_Stream2_IRQn

// *************** IIC *****************************
#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

// *************** ADC *****************************
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC5
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3
#define VBAT_SCALE_DEFAULT         103

// *************** RANGEFINDER *****************************
// #define USE_RANGEFINDER
// #define USE_RANGEFINDER_HCSR04
// #define RANGEFINDER_HCSR04_TRIGGER_PIN       PB10
// #define RANGEFINDER_HCSR04_ECHO_PIN          PB11
// #define USE_RANGEFINDER_SRF10

// *************** NAV *****************************
#define NAV
#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION
#define NAV_MAX_WAYPOINTS       60

// *************** Others *****************************
#define DISPLAY

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3
#define DEFAULT_FEATURES        (FEATURE_BLACKBOX | FEATURE_RX_SERIAL| FEATURE_RSSI_ADC | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TELEMETRY)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 13
#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(12) )
