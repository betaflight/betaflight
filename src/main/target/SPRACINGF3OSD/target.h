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

#define TARGET_BOARD_IDENTIFIER "SOF3"
#define USE_TARGET_CONFIG

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define LED0_PIN                PA15

#define USE_EXTI


#define USE_GYRO
#define USE_FAKE_GYRO

#define USE_ACC
#define USE_FAKE_ACC

#define REMAP_TIM16_DMA
#define REMAP_TIM17_DMA

#define USE_VCP
#define USE_UART1
#define USE_UART3
#define SERIAL_PORT_COUNT       3

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define BUS_SWITCH_PIN          PB3 // connects and disconnects UART1 from external devices

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_1 // Flash Chip
#define USE_SPI_DEVICE_2 // MAX7456

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN
#define MAX7456_NRST_PIN        PB2

#define MAX7456_DMA_CHANNEL_TX              DMA1_Channel5
#define MAX7456_DMA_CHANNEL_RX              DMA1_Channel4
#define MAX7456_DMA_IRQ_HANDLER_ID          DMA1_CH4_HANDLER

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA2
#define CURRENT_METER_ADC_PIN   PA3
#define VOLTAGE_12V_ADC_PIN     PA0
#define VOLTAGE_5V_ADC_PIN      PA1

#define USE_TRANSPONDER

#define DEFAULT_FEATURES        (FEATURE_TRANSPONDER)

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 12 // 2xPPM, 6xPWM, UART3 RX/TX, LED Strip, IR.
#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(15) | TIM_N(16))
