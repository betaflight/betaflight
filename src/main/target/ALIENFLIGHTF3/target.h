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

#define TARGET_BOARD_IDENTIFIER "AFF3" // ALIENFLIGHTF3

#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN                  PB2

// LED's V1
#define LED0                    PB4
#define LED1                    PB5

// LED's V2
#define LED0_A                  PB8
#define LED1_A                  PB9

#define BEEPER                  PA5

#define USE_EXTI
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

// Using MPU6050 for the moment.
#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN CW270_DEG

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN CW270_DEG

// No baro support.
//#define BARO
//#define USE_BARO_MS5611

// No mag support for now (option to use MPU9150 in the future).
//#define MAG
//#define USE_MAG_AK8975
//#define MAG_AK8975_ALIGN CW0_DEG_FLIP

#define USE_VCP
#define USE_UART1 // Not connected - TX (PB6) RX PB7 (AF7)
#define USE_UART2 // Receiver - RX (PA3)
#define USE_UART3 // Not connected - 10/RX (PB11) 11/TX (PB10)
#define SERIAL_PORT_COUNT       4
#define AVOID_UART3_FOR_PWM_PPM

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // SDA (PA10/AF4), SCL (PA9/AF4)

#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

// SPI3
// PA15 38 SPI3_NSS
// PB3  39 SPI3_SCK
// PB4  40 SPI3_MISO
// PB5  41 SPI3_MOSI

#define USE_SPI
#define USE_SPI_DEVICE_3

#define M25P16_CS_PIN           PA15
#define M25P16_SPI_INSTANCE     SPI3

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOA
#define MPU6500_CS_PIN                   PA15
#define MPU6500_SPI_INSTANCE             SPI3

#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC_DMA_CHANNEL         DMA2_Channel1
#define ADC_AHB_PERIPHERAL      RCC_AHBPeriph_DMA2
#define VBAT_ADC_PIN            PA4

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

#define HARDWARE_BIND_PLUG
// Hardware bind plug at PB12 (Pin 25)
#define BINDPLUG_PIN            PB12

#undef BLACKBOX

#undef GPS
#undef GPS_PROTO_NMEA
#undef GPS_PROTO_UBLOX
#undef GPS_PROTO_I2C_NAV
#undef GPS_PROTO_NAZA

#undef TELEMETRY
#undef TELEMETRY_FRSKY
#undef TELEMETRY_HOTT
#undef TELEMETRY_SMARTPORT
#undef TELEMETRY_LTM

#define BRUSHED_MOTORS
#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SPEKTRUM2048
#define SERIALRX_UART           SERIAL_PORT_USART3

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(17) )

