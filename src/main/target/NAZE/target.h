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

#define TARGET_CONFIG
#define USE_HARDWARE_REVISION_DETECTION
#define TARGET_BUS_INIT

#define BOARD_HAS_VOLTAGE_DIVIDER

#define LED0                    PB3
#define LED1                    PB4

#define BEEPER                  PA12

#if defined(AFROMINI)
#define BEEPER_INVERTED
#define TARGET_BOARD_IDENTIFIER "AFMN"
#elif defined(BEEBRAIN)
#define BRUSHED_MOTORS
#define TARGET_BOARD_IDENTIFIER "BEBR"
#define TARGET_CONFIG
#define DEFAULT_FEATURES FEATURE_MOTOR_STOP
#else
#define TARGET_BOARD_IDENTIFIER "AFNA"
// Beeper configuration is handled in 'config.c', since it is dependent on hardware revision
#endif

//#define BARO_XCLR_PIN           PC13
//#define BARO_EOC_PIN            PC14

#define INVERTER_PIN_USART2       PB2 // PB2 (BOOT1) abused as inverter select GPIO

#define USE_EXTI
#define MAG_INT_EXTI            PC14
#define MPU_INT_EXTI            PC13
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MAG_DATA_READY_INTERRUPT
#define USE_MAG_DATA_READY_SIGNAL

// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

#define USE_SPI
#define USE_SPI_DEVICE_2

#define NAZE_SPI_INSTANCE       SPI2
#define NAZE_SPI_CS_PIN         PB12

// We either have this 16mbit flash chip on SPI or the MPU6500 acc/gyro depending on board revision:
#define M25P16_CS_PIN           NAZE_SPI_CS_PIN
#define M25P16_SPI_INSTANCE     NAZE_SPI_INSTANCE

#define MPU6500_CS_PIN          NAZE_SPI_CS_PIN
#define MPU6500_SPI_INSTANCE    NAZE_SPI_INSTANCE

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define GYRO
#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500

#define GYRO_MPU3050_ALIGN      CW0_DEG
#define GYRO_MPU6050_ALIGN      CW0_DEG
#define GYRO_MPU6500_ALIGN      CW0_DEG

#define ACC
#define USE_ACC_ADXL345
#define USE_ACC_BMA280
#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500

#define ACC_ADXL345_ALIGN       CW270_DEG
#define ACC_MPU6050_ALIGN       CW0_DEG
#define ACC_MMA8452_ALIGN       CW90_DEG
#define ACC_BMA280_ALIGN        CW0_DEG
#define ACC_MPU6500_ALIGN       CW0_DEG

#define BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

/*
#define MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG
*/

//#define SONAR
//#define SONAR_TRIGGER_PIN       PB0
//#define SONAR_ECHO_PIN          PB1
//#define SONAR_TRIGGER_PIN_PWM   PB8
//#define SONAR_ECHO_PIN_PWM      PB9

#define USE_UART1
#define USE_UART2
/* only 2 uarts available on the NAZE, add ifdef here if present on other boards */
//#define USE_UART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       4

#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4 // PWM 5
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5 // PWM 6
#define SOFTSERIAL_2_TIMER TIM3
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6 // PWM 7
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7 // PWM 8

#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PB1
#define VBAT_ADC_PIN            PA4
#define RSSI_ADC_PIN            PA1
#define EXTERNAL1_ADC_PIN       PA5

#define LED_STRIP

#undef GPS

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

#if !defined(BRUSHED_MOTORS)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
