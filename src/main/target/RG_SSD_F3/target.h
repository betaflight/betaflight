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

#define TARGET_BOARD_IDENTIFIER "RGF3" // rgSSD_F3

#define LED0_PIN                 PC1
#define LED1_PIN                 PC0

#define BEEPER                   PA8
#define BEEPER_INVERTED

#define MPU6000_CS_PIN           PB2
#define MPU6000_SPI_INSTANCE     SPI2

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI1_NSS_PIN             PB12
#define SPI1_SCK_PIN             PB3
#define SPI1_MISO_PIN            PB4
#define SPI1_MOSI_PIN            PB5

#define SPI2_NSS_PIN             PB2
#define SPI2_SCK_PIN             PB13
#define SPI2_MISO_PIN            PB14
#define SPI2_MOSI_PIN            PB15


#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW180_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW180_DEG
#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT
#define USE_TARGET_CONFIG

#define USE_EXTI
#define MPU_INT_EXTI PC13
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN        PC14

#define SDCARD_SPI_INSTANCE      SPI1
#define SDCARD_SPI_CS_GPIO       GPIOB
#define SDCARD_SPI_CS_PIN        SPI1_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel3
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 8

#define UART1_TX_PIN             PC4
#define UART1_RX_PIN             PC5
#define UART2_TX_PIN             PA2
#define UART2_RX_PIN             PA3
#define UART3_TX_PIN             PB10
#define UART3_RX_PIN             PB11
#define UART4_TX_PIN             PC10
#define UART4_RX_PIN             PC11
#define UART5_TX_PIN             PC12
#define UART5_RX_PIN             PD2

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)

#define I2C2_SCL_GPIO            GPIOA
#define I2C2_SCL_GPIO_AF         GPIO_AF_4
#define I2C2_SCL_PIN             GPIO_Pin_9
#define I2C2_SCL_PIN_SOURCE      GPIO_PinSource9
#define I2C2_SCL_CLK_SOURCE      RCC_AHBPeriph_GPIOA
#define I2C2_SDA_GPIO            GPIOA
#define I2C2_SDA_GPIO_AF         GPIO_AF_4
#define I2C2_SDA_PIN             GPIO_Pin_10
#define I2C2_SDA_PIN_SOURCE      GPIO_PinSource10
#define I2C2_SDA_CLK_SOURCE      RCC_AHBPeriph_GPIOA

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define ADC_INSTANCE             ADC2

#define VBAT_ADC_PIN             PA4
#define CURRENT_METER_ADC_PIN    PA5
#define VBAT_SCALE_DEFAULT       119

#define USE_TRANSPONDER

#define DEFAULT_RX_FEATURE FEATURE_RX_PPM

#define REMAP_TIM16_DMA
#define REMAP_TIM17_DMA
#define ADC24_DMA_REMAP

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303rc in 64pin package
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(4))

#define USED_TIMERS  (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16) | TIM_N(17))

#define USABLE_TIMER_CHANNEL_COUNT 9
