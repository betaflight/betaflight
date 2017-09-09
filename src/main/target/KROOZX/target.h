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

#define TARGET_BOARD_IDENTIFIER "KROOZX"
#define USBD_PRODUCT_STRING     "KroozX"

#define USE_HARDWARE_PREBOOT_SETUP

#define LED0                    PA14 // Red LED
#define LED1                    PA13 // Green LED

#define BEEPER                  PC1
#define BEEPER_INVERTED

#define INVERTER_PIN_UART1      PB13
#define INVERTER_PIN_UART6      PB12

#define MPU6000_CS_PIN          PB2
#define MPU6000_SPI_INSTANCE    SPI1

// MPU6000 interrupts
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define MPU_INT_EXTI            PA4

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW90_DEG_FLIP

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW90_DEG_FLIP

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW270_DEG_FLIP
#define MAG_I2C_INSTANCE        I2CDEV_1

#define BARO
#define USE_BARO_MS5611
#define BARO_I2C_INSTANCE       I2CDEV_3

#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PC13
#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PA15
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

#define OSD
#ifdef USE_MSP_DISPLAYPORT
#undef USE_MSP_DISPLAYPORT
#endif
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PC4
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define OSD_CH_SWITCH           PC5

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC3
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC0
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define USB_IO
#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       6

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#define I2C_DEVICE_EXT          (I2CDEV_3)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7
#define I2C3_SCL                PA8
#define I2C3_SDA                PC9

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define RX_CHANNELS_TAER
#define DEFAULT_FEATURES        (FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_OSD)

#define LED_STRIP
#define WS2811_GPIO_AF                  GPIO_AF_TIM3
#define WS2811_PIN                      PC6
#define WS2811_TIMER                    TIM3
#define WS2811_TIMER_CHANNEL            TIM_Channel_1
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST4_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream4
#define WS2811_DMA_CHANNEL              DMA_Channel_5
#define WS2811_DMA_IRQ                  DMA1_Stream4_IRQn
#define WS2811_DMA_FLAG                 DMA_FLAG_TCIF4
#define WS2811_DMA_IT                   DMA_IT_TCIF4

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define MAX_PWM_OUTPUT_PORTS    10
#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS             (TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12))
