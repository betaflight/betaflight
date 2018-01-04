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

#define TARGET_BOARD_IDENTIFIER "EACH"  /* https://github.com/vladisenko/EachiWhoop */

#define LED0_PIN                PA8


#define USE_EXTI
#define MPU_INT_EXTI            PA15
#define USE_MPU_DATA_READY_SIGNAL


#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2


#define MPU6500_SPI_INSTANCE    SPI1
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5
#define MPU6500_CS_PIN          PA5


#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW90_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW90_DEG


#define USE_RX_SPI
#define USE_RX_FLYSKY
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_A7105_FLYSKY_2A
#define FLYSKY_2A_CHANNEL_COUNT 10

#define RX_SPI_INSTANCE         SPI2
#define RX_IRQ_PIN              PB12
#define SPI2_NSS_PIN            PA4
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define RX_NSS_PIN              SPI2_NSS_PIN
#define BINDPLUG_PIN            PA1


#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE (I2CDEV_2)
#define I2C2_SDA PA10
#define I2C2_SCL PA9


#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611


#define SERIAL_PORT_COUNT       3
#define USE_VCP
#define USE_UART1
#define USE_UART2


#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_RX_PIN            PA2
#define UART2_TX_PIN            PA3


#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define VBAT_SCALE_DEFAULT      40


#undef USE_SERVOS
#undef BEEPER


#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define BRUSHED_MOTORS

#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
#define DEFAULT_FEATURES        (FEATURE_MOTOR_STOP)


#define USB_DETECT_PIN          PC14

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define REMAP_TIM16_DMA
#define REMAP_TIM17_DMA

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             ( TIM_N(2) | TIM_N(8) | TIM_N(16) | TIM_N(17) )
