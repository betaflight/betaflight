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

#define TARGET_BOARD_IDENTIFIER "MIF3"

#ifdef MIDELICF3_PROTOTYPE
//V1 prototype board
#define LED0_PIN                PB3
#else
//V1 production board
#define LED0_PIN                PA13
#endif

#define BEEPER                  PC14

#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW270_DEG

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW270_DEG

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define SERIAL_PORT_COUNT       2

#define SPEKTRUM_BIND
#define BIND_PIN                PA14
#define BINDPLUG_PIN            PC13

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE             (I2CDEV_1)

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_SPI_INSTANCE     SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN           SPI2_NSS_PIN

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_RX_CC2500

#if defined(USE_RX_CC2500)
	
#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1
#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA

#define USE_RX_FRSKYD
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKYD
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI

#define FRSKY_BIND

#define FRSKY_TELEMETRY
#define HUB

// Amplifier chip
#define PA_LNA
// Antenna switch
#define DIVERSITY
//#define SWAMPING

//Pinout
#define RX_NSS_PIN              SPI1_NSS_PIN
#define RX_SCK_PIN              SPI1_SCK_PIN
#define RX_MISO_PIN             SPI1_MISO_PIN
#define RX_MOSI_PIN             SPI1_MOSI_PIN
#define GDO_0_PIN               PB0
#define ANT_SEL_PIN             PB11
#define TX_EN_PIN               PB1
#define RX_EN_PIN               PB2
#if defined(MIDELICF3V2)
#define FRSKY_LED_PIN           PB4
#else
#define FRSKY_LED_PIN           PA8
#endif

#else
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#endif //USE_RX_CC2500

#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0

#define DEFAULT_FEATURES        (FEATURE_AIRMODE | FEATURE_TELEMETRY)

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 7
#if defined(MIDELICF3V2)
#define USED_TIMERS             (TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16))
#else
#define USED_TIMERS             (TIM_N(3) | TIM_N(4) | TIM_N(15))
#endif
