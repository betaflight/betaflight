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

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_4  // Blue (Rev 1 & 2) - PB4
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_5  // Green (Rev 1) / Red (Rev 2) - PB5
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOB

#define USABLE_TIMER_CHANNEL_COUNT 11

// MPU 9150 INT connected to PA15, pulled up to VCC by 10K Resistor, contains MPU6050 and AK8975 in single component.
#define GYRO
#define USE_GYRO_MPU6050

#define GYRO_MPU6050_ALIGN CW270_DEG

#define ACC
#define USE_ACC_MPU6050

#define ACC_MPU6050_ALIGN CW270_DEG

#define BARO
#define USE_BARO_MS5611

#define MAG
#define USE_MAG_AK8975

#define MAG_AK8975_ALIGN CW0_DEG_FLIP

#define LED0
#define LED1

#define USE_VCP
#define USE_USART1 // Conn 1 - TX (PB6) RX PB7 (AF7)
#define USE_USART2 // Input - RX (PA3)
#define USE_USART3 // Servo out - 10/RX (PB11) 11/TX (PB10)
#define SERIAL_PORT_COUNT 4

#define UART1_TX_PIN        GPIO_Pin_6 // PB6
#define UART1_RX_PIN        GPIO_Pin_7 // PB7
#define UART1_GPIO          GPIOB
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource6
#define UART1_RX_PINSOURCE  GPIO_PinSource7

#define UART2_TX_PIN        GPIO_Pin_2 // PA2 - Clashes with PWM6 input.
#define UART2_RX_PIN        GPIO_Pin_3 // PA3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource2
#define UART2_RX_PINSOURCE  GPIO_PinSource3

#define UART3_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

// Note: PA5 and PA0 are N/C on the sparky - potentially use for ADC or LED STRIP?

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // SDA (PA10/AF4), SCL (PA9/AF4)

#define I2C2_SCL_GPIO        GPIOA
#define I2C2_SCL_GPIO_AF     GPIO_AF_4
#define I2C2_SCL_PIN         GPIO_Pin_9
#define I2C2_SCL_PIN_SOURCE  GPIO_PinSource9
#define I2C2_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOA
#define I2C2_SDA_GPIO        GPIOA
#define I2C2_SDA_GPIO_AF     GPIO_AF_4
#define I2C2_SDA_PIN         GPIO_Pin_10
#define I2C2_SDA_PIN_SOURCE  GPIO_PinSource10
#define I2C2_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOA

#define USE_ADC

#define ADC_INSTANCE                ADC2
#define ADC_DMA_CHANNEL             DMA2_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define CURRENT_METER_ADC_GPIO      GPIOA
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_7
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_4

#define AUTOTUNE
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define GPS
#define DISPLAY
#define USE_SERVOS
#define USE_CLI

#define LED_STRIP
#if 1
// LED strip configuration using PWM motor output pin 5.
#define LED_STRIP_TIMER TIM16

#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_1
#define WS2811_PIN                      GPIO_Pin_6 // TIM16_CH1
#define WS2811_PIN_SOURCE               GPIO_PinSource6
#define WS2811_TIMER                    TIM16
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#endif

#if 0
// Alternate LED strip pin
// FIXME DMA IRQ Transfer Complete is never called because the  TIM17_DMA_RMP needs to be set in SYSCFG_CFGR1
#define LED_STRIP_TIMER TIM17

#define USE_LED_STRIP_ON_DMA1_CHANNEL7
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_1
#define WS2811_PIN                      GPIO_Pin_7 // TIM17_CH1
#define WS2811_PIN_SOURCE               GPIO_PinSource7
#define WS2811_TIMER                    TIM17
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM17
#define WS2811_DMA_CHANNEL              DMA1_Channel7
#define WS2811_IRQ                      DMA1_Channel7_IRQn
#endif


#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PORT GPIOA
#define BIND_PIN Pin_3
