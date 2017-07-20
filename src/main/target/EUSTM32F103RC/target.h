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

#define TARGET_BOARD_IDENTIFIER "EUF1"

#define LED0                    PB3
#define LED1                    PB4

#define INVERTER_PIN_UART2      PB2

#define MPU6000_CS_PIN          PB12
#define MPU6000_SPI_INSTANCE    SPI2

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOB
#define MPU6500_CS_PIN          PB12
#define MPU6500_SPI_INSTANCE    SPI2

#define GYRO
#define USE_FAKE_GYRO
#define USE_GYRO_L3G4200D
//#define USE_GYRO_L3GD20
//#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500

#define GYRO_MPU6050_ALIGN      CW0_DEG

#define ACC
#define USE_FAKE_ACC
#define USE_ACC_ADXL345
//#define USE_ACC_BMA280
//#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
//#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6050_ALIGN       CW0_DEG

#define BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_AK8975
#define MAG_AK8975_ALIGN        CW180_DEG_FLIP

// #define USE_RANGEFINDER
//#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB0
#define RANGEFINDER_HCSR04_ECHO_PIN          PB1
#define RANGEFINDER_HCSR04_TRIGGER_PIN_PWM   PB8
#define RANGEFINDER_HCSR04_ECHO_PIN_PWM      PB9

//#define USE_DASHBOARD

#define USE_UART1
#define USE_UART2
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       4

#define SOFTSERIAL_1_RX_PIN     PA6
#define SOFTSERIAL_1_TX_PIN     PA7
#define SOFTSERIAL_2_RX_PIN     PB0
#define SOFTSERIAL_2_TX_PIN     PB1

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PB1
#define ADC_CHANNEL_2_PIN               PA4
#define ADC_CHANNEL_3_PIN               PA1
#define ADC_CHANNEL_4_PIN               PA5
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

// IO - stm32f103RCT6 in 64pin package (TODO)
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(0)|BIT(1)|BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
