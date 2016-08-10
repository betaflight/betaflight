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

#define TARGET_BOARD_IDENTIFIER "OLI1" // Olimexino

//#define OLIMEXINO_UNCUT_LED1_E_JUMPER
//#define OLIMEXINO_UNCUT_LED2_E_JUMPER

#ifdef OLIMEXINO_UNCUT_LED1_E_JUMPER
#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_5 // D13, PA5/SPI1_SCK/ADC5 - "LED1" on silkscreen, Green
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOA
#define LED0
#endif

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
// "LED2" is using one of the PWM pins (CH2/PWM2), so we must not use PWM2 unless the jumper is cut.  @See pwmInit()
#define LED1_GPIO   GPIOA
#define LED1_PIN    Pin_1 // D3, PA1/UART2_RTS/ADC1/TIM2_CH3 - "LED2" on silkscreen, Yellow
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOA
#define LED1
#endif

#define GYRO
#define USE_FAKE_GYRO
//#define USE_GYRO_L3G4200D
//#define USE_GYRO_L3GD20
//#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
//#define USE_GYRO_SPI_MPU6000
//#define USE_GYRO_SPI_MPU6500

#define ACC
#define USE_FAKE_ACC
//#define USE_ACC_ADXL345
//#define USE_ACC_BMA280
//#define USE_ACC_MMA8452
//#define USE_ACC_LSM303DLHC
#define USE_ACC_MPU6050
//#define USE_ACC_SPI_MPU6000
//#define USE_ACC_SPI_MPU6500

#define BARO
//#define USE_BARO_MS5611
#define USE_BARO_BMP085

#define MAG
#define USE_MAG_HMC5883

#define SONAR
#define SONAR_TRIGGER_PIN           Pin_0   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
#define SONAR_TRIGGER_GPIO          GPIOB
#define SONAR_ECHO_PIN              Pin_1   // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
#define SONAR_ECHO_GPIO             GPIOB
#define SONAR_TRIGGER_IO            PB0
#define SONAR_ECHO_IO               PB1


#define USE_UART1
#define USE_UART2
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 4

#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4 // PWM 5
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5 // PWM 6
#define SOFTSERIAL_2_TIMER TIM3
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6 // PWM 7
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7 // PWM 8

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

#define USE_ADC

#define ADC_INSTANCE                ADC1
#define ADC_ABP2_PERIPHERAL         RCC_APB2Periph_ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

#define ADC0_GPIO                   GPIOB
#define ADC0_GPIO_PIN               GPIO_Pin_1
#define ADC0_CHANNEL                ADC_Channel_9

#define ADC1_GPIO                   GPIOA
#define ADC1_GPIO_PIN               GPIO_Pin_4
#define ADC1_CHANNEL                ADC_Channel_4

#define ADC2_GPIO                   GPIOA
#define ADC2_GPIO_PIN               GPIO_Pin_1
#define ADC2_CHANNEL                ADC_Channel_1

#define ADC3_GPIO                   GPIOA
#define ADC3_GPIO_PIN               GPIO_Pin_5
#define ADC3_CHANNEL                ADC_Channel_5

#define ADC_CHANNEL_COUNT 4

#define ADC_CURRENT     ADC_CHANNEL0
#define ADC_BATTERY     ADC_CHANNEL1
#define ADC_RSSI        ADC_CHANNEL2
#define ADC_EXTERNAL    ADC_CHANNEL3

#define GPS

#define LED_STRIP
#define LED_STRIP_TIMER TIM3
#define WS2811_DMA_TC_FLAG           DMA1_FLAG_TC6
#define WS2811_DMA_HANDLER_IDENTIFER DMA1Channel6Descriptor

#define TELEMETRY
#define SERIAL_RX
#define BLACKBOX
#define USE_SERVOS
#define USE_CLI
#define USE_EXTI

// IO - assuming all IOs on smt32f103rb LQFP64 package
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
