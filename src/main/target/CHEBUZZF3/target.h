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

#define TARGET_BOARD_IDENTIFIER "CHF3" // Chebuzz F3

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#ifndef STM32F3DISCOVERY
#define STM32F3DISCOVERY
#endif

#define LED0_PIN                PE8 // Blue LEDs - PE8/PE12
#define LED0_INVERTED
#define LED1_PIN                PE10  // Orange LEDs - PE10/PE14
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

#define USE_SDCARD

#define SDCARD_DETECT_PIN                    PC14
#define SDCARD_SPI_INSTANCE                  SPI2
#define SDCARD_SPI_CS_PIN                    SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 2

// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

//#define M25P16_CS_GPIO          GPIOB
//#define M25P16_CS_PIN           GPIO_Pin_12
//#define M25P16_SPI_INSTANCE     SPI2

#define USE_GYRO
#define USE_GYRO_L3GD20
#define USE_GYRO_MPU6050

#define L3GD20_SPI                      SPI1
#define L3GD20_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOE
#define L3GD20_CS_GPIO                  GPIOE
#define L3GD20_CS_PIN                   PE3

#define GYRO_L3GD20_ALIGN CW270_DEG
#define GYRO_MPU6050_ALIGN CW0_DEG

#define USE_ACC
#define USE_ACC_MPU6050
#define USE_ACC_LSM303DLHC
#define LSM303DLHC_I2C                       I2C1
#define LSM303DLHC_I2C_SCK_PIN               PB6
#define LSM303DLHC_I2C_SDA_PIN               PB7
#define LSM303DLHC_DRDY_PIN                  PE2
#define LSM303DLHC_I2C_INT1_PIN              PE4
#define LSM303DLHC_I2C_INT2_PIN              PE5

#define ACC_MPU6050_ALIGN       CW0_DEG

#define USE_BARO
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_AK8975
#define MAG_AK8975_ALIGN        CW90_DEG_FLIP

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 5

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8  // (HARDARE=0)

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#undef USE_LED_STRIP

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2)|BIT(5)|BIT(6)|BIT(10)|BIT(12)|BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4)|BIT(9)|BIT(10))

#define USABLE_TIMER_CHANNEL_COUNT 18
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17))
