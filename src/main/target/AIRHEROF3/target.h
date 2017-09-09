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

#define TARGET_BOARD_IDENTIFIER "AIR3"

#define LED0                    PB3
#define LED1                    PB4

#define BEEPER                  PA12
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI            PC13
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU INT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_SPI
#define USE_SPI_DEVICE_2

#define MPU6500_CS_PIN          PB12
#define MPU6500_SPI_INSTANCE    SPI2

#define BMP280_SPI_INSTANCE     SPI2
#define BMP280_CS_PIN           PB5

#define GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW0_DEG

#define ACC
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW0_DEG

#define BARO
#define USE_BARO_SPI_BMP280

#define MAG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define USE_PITOT_MS4525
#define USE_PITOT_ADC

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT       4

#define SOFTSERIAL_1_RX_PIN     PA6
#define SOFTSERIAL_1_TX_PIN     PA7

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#define BOARD_HAS_VOLTAGE_DIVIDER

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PA4
#define ADC_CHANNEL_1_INSTANCE          ADC2
#define ADC_CHANNEL_2_PIN               PA1
#define ADC_CHANNEL_2_INSTANCE          ADC1

#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define AIRSPEED_ADC_CHANNEL            ADC_CHN_2


/*
#define LED_STRIP
#define WS2811_TIMER                    TIM3
#define WS2811_PIN                      PA6
#define WS2811_DMA_CHANNEL              DMA1_Channel6
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC6
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH6_HANDLER
*/

#define DEFAULT_FEATURES        FEATURE_VBAT | FEATURE_SOFTSERIAL
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER

#ifdef AIRHEROF3_QUAD
    #define USE_I2C
    #define I2C_DEVICE  (I2CDEV_1) // SDA (PB9/AF4), SCL (PB8/AF4)

    #define I2C1_SCL    PB8
    #define I2C1_SDA    PB9

    // MAG is I2C, so it is only useful when I2C is available
    #define MAG
    #define USE_MAG_HMC5883
    #define USE_MAG_MAG3110

    #define MAX_PWM_OUTPUT_PORTS        8
    #define TARGET_MOTOR_COUNT          4
    #define USABLE_TIMER_CHANNEL_COUNT  10
#else
    #define MAX_PWM_OUTPUT_PORTS        10
    #define TARGET_MOTOR_COUNT          6
    #define USABLE_TIMER_CHANNEL_COUNT  12
#endif

#define SPEKTRUM_BIND
// USART3,
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTF         (BIT(4))

#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
