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

#define TARGET_BOARD_IDENTIFIER "FLCR" // FaLCoRre

#define TARGET_CONFIG

#define LED0                    PC2
#define LED1                    PB11

#define USE_HARDWARE_PREBOOT_SETUP      // FALCORE board requires some hardware to be set up before booting and detecting sensors

#define BEEPER                  PB4
#define BEEPER_PWM
#define BEEPER_PWM_FREQUENCY    2700

#define MPU6500_CS_PIN          PC0
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_EXTI
#define MPU_INT_EXTI            PB0
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN CW0_DEG

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN CW0_DEG

#define BARO
#define USE_BARO_MS5607

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_MAG3110
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE        I2C_DEVICE_EXT

#define USB_IO

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART4
#define USE_UART5
#define SERIAL_PORT_COUNT       5
#define AVOID_UART2_FOR_PWM_PPM

#define UART1_TX_PIN            PC4
#define UART1_RX_PIN            PC5

#define UART2_TX_PIN            PA2 // PA2 - Clashes with PWM6 input.
#define UART2_RX_PIN            PA3

#define UART4_TX_PIN            PC10
#define UART4_RX_PIN            PC11

#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PB5

#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#define I2C_DEVICE_EXT          (I2CDEV_2)

#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define VBAT_SCALE_DEFAULT       78
#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_INSTANCE             ADC1
#define ADC_CHANNEL_1_PIN        PF4
#define ADC_CHANNEL_2_PIN        PA1
#define VBAT_ADC_CHANNEL         ADC_CHN_1
#define RSSI_ADC_CHANNEL         ADC_CHN_2

#define LED_STRIP
#define USE_LED_STRIP_ON_DMA1_CHANNEL2
#define WS2811_PIN                      PA8
#define WS2811_DMA_STREAM               DMA1_Channel2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           PC1
#define M25P16_SPI_INSTANCE     SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PA7
#define RANGEFINDER_HCSR04_ECHO_PIN          PA2

#define DEFAULT_FEATURES        (FEATURE_BLACKBOX | FEATURE_VBAT | FEATURE_GPS | FEATURE_TELEMETRY | FEATURE_LED_STRIP)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

// available IO pins (from schematics)
#define TARGET_IO_PORTA         0xFFFF
#define TARGET_IO_PORTB         0xFFFF
#define TARGET_IO_PORTC         0xFFFF
#define TARGET_IO_PORTD         0xFFFF
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(16))

