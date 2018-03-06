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
#if defined(FF_PIKOF4OSD)
#define TARGET_BOARD_IDENTIFIER "PKOS"
#define USBD_PRODUCT_STRING     "PikoF4OSD"
#else
#define TARGET_BOARD_IDENTIFIER "PIK4"
#define USBD_PRODUCT_STRING     "PikoF4"
#endif
#define USE_TARGET_CONFIG
/*--------------LED----------------*/
#if defined(FF_PIKOF4OSD)
#define LED0_PIN                PB5
#define LED1_PIN                PB4
#else
#define LED0_PIN                PA15
#define LED1_PIN                PB6
#endif
/*---------------------------------*/

/*------------BEEPER---------------*/
#define BEEPER                  PA14
#define BEEPER_INVERTED
/*---------------------------------*/

/*----------CAMERA CONTROL---------*/
// #define CAMERA_CONTROL_PIN      PB7
/*---------------------------------*/

/*------------SENSORS--------------*/
// MPU interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PC4
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#if defined(FF_PIKOF4OSD)
#define MPU6000_CS_PIN          PA15
#define MPU6000_SPI_INSTANCE    SPI3

#define MPU6500_CS_PIN          PA15
#define MPU6500_SPI_INSTANCE    SPI3
#else
#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1
#endif

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG

#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW180_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG
/*---------------------------------*/

#if defined(FF_PIKOF4OSD)
/*-------------OSD-----------------*/
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PA4
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)
/*---------------------------------*/
#else
/*------------FLASH----------------*/
#define M25P16_CS_PIN           PB3
#define M25P16_SPI_INSTANCE     SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
/*---------------------------------*/
#endif

/*-----------USB-UARTs-------------*/
#define USE_VCP
//#define VBUS_SENSING_PIN PA8
//#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#if defined(FF_PIKOF4OSD)
#define INVERTER_PIN_UART3      PC3
#else
#define INVERTER_PIN_UART3      PC8
#endif

#define USE_UART4
#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       5


/*---------------------------------*/

/*-------------SPIs----------------*/
#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#if defined(FF_PIKOF4OSD)
#define SPI3_NSS_PIN            PA15
#else
#define SPI3_NSS_PIN            PB3
#endif
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
/*---------------------------------*/

/*-------------ADCs----------------*/
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define VBAT_ADC_PIN                    PC2
#if defined(FF_PIKOF4OSD)
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_ADC_PIN           PC1
#define CURRENT_METER_SCALE_DEFAULT     250
#endif
/*---------------------------------*/

/*-------------ESCs----------------*/
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA3  // (HARDARE=0)
/*---------------------------------*/

/*--------DEFAULT VALUES-----------*/
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#if defined(FF_PIKOF4OSD)
#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD )
#endif
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
/*---------------------------------*/

/*--------------TIMERS-------------*/
#if defined(FF_PIKOF4OSD)
#define USABLE_TIMER_CHANNEL_COUNT  7
#define USED_TIMERS             ( TIM_N(1) | TIM_N(3) | TIM_N(5) | TIM_N(8) )
#else
#define USABLE_TIMER_CHANNEL_COUNT  5
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) )
#endif
/*---------------------------------*/
