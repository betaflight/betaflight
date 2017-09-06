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

#define TARGET_BOARD_IDENTIFIER "PIKO" // Furious FPV Piko BLX
#define USE_CLI

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#define LED0                    PB9
#define LED1                    PB5

#define BEEPER                  PA0
#define BEEPER_INVERTED

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PA15
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

#define MPU6000_CS_GPIO         GPIOB
#define MPU6000_CS_PIN          PB12
#define MPU6000_SPI_INSTANCE    SPI2

#define USB_IO
#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT       4

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PB3
#define UART2_RX_PIN            PB4

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#define USE_SPI
#define USE_SPI_DEVICE_2

#define SENSORS_SET             (SENSOR_ACC)

#define TELEMETRY
#define BLACKBOX
#define SERIAL_RX
#define USE_SERVOS

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC_CHANNEL_1_PIN               PA2
#define ADC_CHANNEL_2_PIN               PA5
#define ADC_CHANNEL_3_PIN               PB2
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define LED_STRIP
#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_PIN                      PB8 // TIM16_CH1
#define WS2811_TIMER                    TIM16
#define WS2811_DMA_STREAM               DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#define TRANSPONDER
#define TRANSPONDER_GPIO                     GPIOA
#define TRANSPONDER_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define TRANSPONDER_GPIO_AF                  GPIO_AF_6
#define TRANSPONDER_PIN                      GPIO_Pin_8
#define TRANSPONDER_PIN_SOURCE               GPIO_PinSource8
#define TRANSPONDER_TIMER                    TIM1
#define TRANSPONDER_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define TRANSPONDER_DMA_CHANNEL              DMA1_Channel2
#define TRANSPONDER_IRQ                      DMA1_Channel2_IRQn
#define TRANSPONDER_DMA_TC_FLAG              DMA1_FLAG_TC2
#define TRANSPONDER_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

#define SPEKTRUM_BIND
// USART3, PB11
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
// #define TARGET_IO_PORTF        (BIT(0)|BIT(1))
// !!TODO - check the following line is correct
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(17))

// sn dec06.16 added MAX_PWM_OUTPUT_PORTS: number of available PWM outputs
// porting inav to PIKO BLX by using betaflight target files from before inav changes to timer.h/timer_def.h
// review of target.c shows definitions for PWB1-PWM9. PWM9 is for PPM input
#define MAX_PWM_OUTPUT_PORTS    9
