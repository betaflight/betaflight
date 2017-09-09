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

#define TARGET_BOARD_IDENTIFIER "ANYF"

#define USBD_PRODUCT_STRING     "AnyFC"

#define LED0                    PB7
#define LED1                    PB6

#define BEEPER                  PB2
#define BEEPER_INVERTED

#define INVERTER_PIN_UART1      PC3

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI PC4
#define USE_MPU_DATA_READY_SIGNAL
#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU data ready (mag disabled)

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE        I2C_DEVICE_EXT
#define MAG_HMC5883_ALIGN       CW270_DEG_FLIP
//#define MAG_HMC5883_ALIGN       CW90_DEG

#define USE_RANGEFINDER
#define USE_RANGEFINDER_VL53L0X
#define RANGEFINDER_VL53L0X_INSTANCE    I2C_DEVICE_EXT

#define BARO
#define USE_BARO_MS5611

#define USE_PITOT_MS4525
#define PITOT_I2C_INSTANCE      I2C_DEVICE_EXT

#define USB_IO
#define USE_VCP
#define VBUS_SENSING_PIN        PA8

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       7 //VCP, UART1, UART2, UART3, UART4, UART5, UART6

#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#define I2C_DEVICE_EXT          (I2CDEV_2)
#define I2C_DEVICE_EXT_SHARES_UART3
//#define USE_I2C_PULLUP

//#define HIL

#define MAG_GPS_ALIGN           CW180_DEG_FLIP

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC2
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define LED_STRIP
// LED Strip can run off Pin 6 (PA0) of the ESC outputs.
#define WS2811_PIN                      PA0
#define WS2811_TIMER                    TIM5
#define WS2811_TIMER_CHANNEL            TIM_Channel_1
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_FLAG                 DMA_FLAG_TCIF2
#define WS2811_DMA_IT                   DMA_IT_TCIF2
#define WS2811_DMA_CHANNEL              DMA_Channel_6
#define WS2811_DMA_IRQ                  DMA1_Stream2_IRQn

#define SENSORS_SET             (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    15

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 16
#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9))
