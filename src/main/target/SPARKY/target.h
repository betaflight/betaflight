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

#define TARGET_BOARD_IDENTIFIER "SPKY" // SParKY

#define LED0                    PB4  // Blue (Rev 1 & 2) - PB4
#define LED1                    PB5  // Green (Rev 1) / Red (Rev 2) - PB5

#define BEEPER                  PA1
#define BEEPER_INVERTED

// MPU6050 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PA15
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL

// MPU 9150 INT connected to PA15, pulled up to VCC by 10K Resistor, contains MPU6050 and AK8975 in single component.
#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW270_DEG

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW270_DEG

#define BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280

#define MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_MAG3110
#define USE_MAG_QMC5883
#define MAG_AK8975_ALIGN        CW180_DEG_FLIP

#define USB_IO
#define USE_VCP
#define USE_UART1 // Conn 1 - TX (PB6) RX PB7 (AF7)
#define USE_UART2 // Input - RX (PA3)
#define USE_UART3 // Servo out - 10/RX (PB11) 11/TX (PB10)
#define SERIAL_PORT_COUNT       4
#define AVOID_UART2_FOR_PWM_PPM

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2 // PA2 - Clashes with PWM6 input.
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

// Note: PA5 and PA0 are N/C on the sparky - potentially use for ADC or LED STRIP?

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // SDA (PA10/AF4), SCL (PA9/AF4)

#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC_CHANNEL_1_PIN               PA4
#define ADC_CHANNEL_2_PIN               PA7
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2

// LED strip configuration using PWM motor output pin 5.
#define LED_STRIP
#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_PIN                      PA6 // TIM16_CH1
#define WS2811_DMA_STREAM               DMA1_Channel3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

// #define USE_RANGEFINDER
// #define USE_RANGEFINDER_HCSR04
// #define RANGEFINDER_HCSR04_TRIGGER_PIN       PA2   // PWM6 (PA2) - only 3.3v ( add a 1K Ohms resistor )
// #define RANGEFINDER_HCSR04_ECHO_PIN          PB1   // PWM7 (PB1) - only 3.3v ( add a 1K Ohms resistor )

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

// available IO pins (from schematics)
//#define TARGET_IO_PORTA         (BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15))
//#define TARGET_IO_PORTB         (BIT(0)|BIT(1)|BIT(10)|BIT(11)|BIT(14)|BIT(15)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9))
// !!TODO - check following lines are correct
#define TARGET_IO_PORTA         (BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTB         (BIT(0)|BIT(1)|BIT(6)|BIT(10)|BIT(11)|BIT(14)|BIT(15)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(12)|BIT(13))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(17))

