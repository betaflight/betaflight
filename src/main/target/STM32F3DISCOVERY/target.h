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

#define TARGET_BOARD_IDENTIFIER "SDF3" // STM Discovery F3

#define LED0                    PE8 // Blue LEDs - PE8/PE12
#define LED0_INVERTED
#define LED1                    PE10  // Orange LEDs - PE10/PE14
#define LED1_INVERTED

#define BEEPER                  PE9 // Red LEDs - PE9/PE13
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define GYRO
#define USE_GYRO_L3GD20
#define L3GD20_SPI              SPI1
#define L3GD20_CS_PIN           PE3
#define GYRO_L3GD20_ALIGN       CW270_DEG

// Support the GY-91 MPU9250 dev board
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define MPU6500_CS_PIN          PC14
#define MPU6500_SPI_INSTANCE    SPI2
#define GYRO_MPU6500_ALIGN      CW270_DEG_FLIP

#define ACC
#define USE_ACC_LSM303DLHC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG_FLIP

#define MAG
#define USE_MAG_HMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define SERIAL_PORT_COUNT       3

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#define LED_STRIP
#define WS2811_PIN                      PB8 // TIM16_CH1
#define WS2811_TIMER                    TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#define LED_STRIP_TIMER                 TIM16

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - 303 in 100pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         0x00ff

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16) | TIM_N(17))

