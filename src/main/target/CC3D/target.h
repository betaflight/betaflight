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

#define TARGET_BOARD_IDENTIFIER "CC3D" // CopterControl 3D

#define LED0                    PB3

#define INVERTER                PB2 // PB2 (BOOT1) used as inverter select GPIO
#define INVERTER_USART          USART1

#define BEEPER                  PA15
#define BEEPER_OPT              PA2

#define USE_EXTI
#define MPU_INT_EXTI            PA3
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // Flex port - SCL/PB10, SDA/PB11

#define MPU6000_CS_GPIO         GPIOA
#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define M25P16_CS_GPIO          GPIOB
#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW270_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW270_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL

// External I2C BARO
#define BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

// External I2C MAG
#define MAG
#define USE_MAG_HMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART3
#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT       4

#ifdef USE_UART1_RX_DMA
#undef USE_UART1_RX_DMA
#endif

#define SOFTSERIAL_1_TIMER      TIM3
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 1 // PWM 2
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 2 // PWM 3

#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_ADC
#define CURRENT_METER_ADC_PIN   PB1
#define VBAT_ADC_PIN            PA0
#define RSSI_ADC_PIN            PB0

#define LED_STRIP
#define WS2811_PIN                      PB4
#define WS2811_TIMER                    TIM3
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC6
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH6_HANDLER

#define SPEKTRUM_BIND
// USART3, PB11 (Flexport)
#define BIND_PIN   PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

//#define SONAR
//#define SONAR_ECHO_PIN          PB0
//#define SONAR_TRIGGER_PIN       PB5

#undef GPS
#define CLI_MINIMAL_VERBOSITY
#undef MAG

#ifdef CC3D_OPBL
#define SKIP_CLI_COMMAND_HELP
#define SKIP_PID_FLOAT
#undef BARO
#undef SONAR
#undef USE_SOFTSERIAL1
#undef LED_STRIP
#define SKIP_PID_FLOAT
#endif

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

// IO - from schematics
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(14) )

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
