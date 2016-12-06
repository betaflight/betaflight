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

#define TARGET_BOARD_IDENTIFIER "SPNE"


#ifndef SPRACINGF3NEO_REV
#define SPRACINGF3NEO_REV 5
#endif

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_9
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_2
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEP_GPIO   GPIOC
#define BEEP_PIN    Pin_15
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOC
#define BEEPER_INVERTED

#define USABLE_TIMER_CHANNEL_COUNT 12 // 2xPPM, 6xPWM, UART3 RX/TX, LED Strip, IR.

#define EXTI_CALLBACK_HANDLER_COUNT 4 // MPU_INT (PC13), SDCardDetect (PC14), PC4, PC5

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO
//#define USE_FAKE_GYRO
#define USE_GYRO_SPI_MPU6500

#define ACC
//#define USE_FAKE_ACC
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6500_ALIGN CW0_DEG
#define GYRO_MPU6500_ALIGN CW0_DEG

#define BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define MAG
#define USE_MAG_AK8975
#define USE_MAG_HMC5883

#define BEEPER
#define LED0
#define LED1

#define USB_IO

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define SERIAL_PORT_COUNT 6

#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_2 // PA2
#define UART2_RX_PIN        GPIO_Pin_3 // PA3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource2
#define UART2_RX_PINSOURCE  GPIO_PinSource3

#define UART3_TX_PIN        GPIO_Pin_10 // PB10
#define UART3_RX_PIN        GPIO_Pin_11 // PB11
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

// pins for UART4 are fixed by hardware (TX = PC10, RX = PC11)
// pins for UART5 are fixed by hardware (TX = PC12, RX = PD2)

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

#define USE_SPI
#define USE_SPI_DEVICE_1 // External (MAX7456 & RTC6705)
#define USE_SPI_DEVICE_2 // SDCard
#define USE_SPI_DEVICE_3 // MPU

// Using SPI1 default IO pins PA4/PA5/PA6/PA7
// Using SPI2 default IO pins PB12/PA13/PA14/PA15
// Using SPI3 default IO pins PA15/PB3/PB4/PB5

#define USE_SDCARD
#define USE_SDCARD_SPI2

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   GPIO_Pin_14
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line14
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource14
#define SDCARD_DETECT_GPIO_PORT             GPIOC
#define SDCARD_DETECT_GPIO_CLK              RCC_AHBPeriph_GPIOC
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOC
#define SDCARD_DETECT_EXTI_IRQn             EXTI15_10_IRQn

#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_GPIO                  SPI2_GPIO
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

// Note, this is the same DMA channel as USART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA1_FLAG_TC5

#if (SPRACINGF3NEO_REV >= 5)
#define MPU6500_CS_GPIO_CLK_PERIPHERAL  SPI1_GPIO_PERIPHERAL
#define MPU6500_CS_GPIO                 SPI1_NSS_GPIO
#define MPU6500_CS_PIN                  SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE            SPI1
#else
#define MPU6500_CS_GPIO_CLK_PERIPHERAL  SPI3_GPIO_PERIPHERAL
#define MPU6500_CS_GPIO                 SPI3_NSS_GPIO
#define MPU6500_CS_PIN                  SPI3_NSS_PIN
#define MPU6500_SPI_INSTANCE            SPI3
#endif

// Bus Switched Device, Device A.
#if (SPRACINGF3NEO_REV >= 5)
#define MAX7456_CS_GPIO                 SPI3_NSS_GPIO
#define MAX7456_CS_PIN                  SPI3_NSS_PIN
#define MAX7456_SPI_INSTANCE            SPI3

#define MAX7456_DMA_CHANNEL_TX          DMA2_Channel2
#define MAX7456_DMA_CHANNEL_RX          DMA2_Channel1
#define MAX7456_DMA_IRQ_HANDLER_ID      DMA2Channel1Descriptor
#else
#define MAX7456_CS_GPIO                 SPI1_NSS_GPIO
#define MAX7456_CS_PIN                  SPI1_NSS_PIN
#define MAX7456_SPI_INSTANCE            SPI1

#define MAX7456_DMA_CHANNEL_TX          DMA1_Channel3
#define MAX7456_DMA_CHANNEL_RX          DMA1_Channel2
#define MAX7456_DMA_IRQ_HANDLER_ID      DMA1Channel2Descriptor
#endif

#if (SPRACINGF3NEO_REV >= 5)
#define MAX7456_VSYNC_IO                PC5
#define MAX7456_HSYNC_IO                PC4
#else
#define MAX7456_VSYNC_IO                PC4
#define MAX7456_HSYNC_IO                PC5
#endif

// Bus Switched Device, Device B.
#define VTX_RTC6705
#define RTC6705_CS_PERIPHERAL           RCC_AHBPeriph_GPIOF
#define RTC6705_CS_GPIO                 GPIOF
#define RTC6705_CS_PIN                  GPIO_Pin_4

#define USE_RTC6705_CLK_HACK
#define RTC6705_CLK_GPIO                GPIOB
#define RTC6705_CLK_PIN                 GPIO_Pin_3

#if (SPRACINGF3NEO_REV >= 5)
#define RTC6705_SPI_INSTANCE            SPI3
#else
#define RTC6705_SPI_INSTANCE            SPI1
#endif

#define RTC6705_POWER_PERIPHERAL        RCC_AHBPeriph_GPIOC
#define RTC6705_POWER_GPIO              GPIOC
#define RTC6705_POWER_PIN               GPIO_Pin_3

#define SPI_SHARED_MAX7456_AND_RTC6705

#define USE_ADC
//#define DEBUG_ADC_CHANNELS
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC1
#define ADC_DMA_CHANNEL             DMA1_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1

// 6 ADC Sources on ADC1

#define ADC0_GPIO             GPIOC
#define ADC0_GPIO_PIN         GPIO_Pin_1
#define ADC0_CHANNEL          ADC_Channel_7

#define ADC1_GPIO             GPIOC
#define ADC1_GPIO_PIN         GPIO_Pin_2
#define ADC1_CHANNEL          ADC_Channel_8

#define ADC2_GPIO             GPIOC
#define ADC2_GPIO_PIN         GPIO_Pin_0
#define ADC2_CHANNEL          ADC_Channel_6

#define ADC3_GPIO             GPIOA
#define ADC3_GPIO_PIN         GPIO_Pin_0
#define ADC3_CHANNEL          ADC_Channel_1

#define ADC4_GPIO             GPIOA
#define ADC4_GPIO_PIN         GPIO_Pin_1
#define ADC4_CHANNEL          ADC_Channel_2

#define ADC_CHANNEL_COUNT 5

#define ADC_BATTERY     ADC_CHANNEL0
#define ADC_AMPERAGE    ADC_CHANNEL1
#define ADC_RSSI        ADC_CHANNEL2
#define ADC_POWER_12V   ADC_CHANNEL3
#define ADC_POWER_5V    ADC_CHANNEL4

#define MAX_VOLTAGE_METERS 3

#define BOARD_HAS_AMPERAGE_METER

#define LED_STRIP

#define LED_STRIP_TIMER TIM1
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_6
#define WS2811_PIN                      GPIO_Pin_8
#define WS2811_PIN_SOURCE               GPIO_PinSource8
#define WS2811_TIMER                    TIM1
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1Channel2Descriptor

#define TRANSPONDER
#define TRANSPONDER_GPIO                     GPIOB
#define TRANSPONDER_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOB
#define TRANSPONDER_GPIO_AF                  GPIO_AF_1
#define TRANSPONDER_PIN                      GPIO_Pin_8 // TIM16_CH1
#define TRANSPONDER_PIN_SOURCE               GPIO_PinSource8
#define TRANSPONDER_TIMER                    TIM16
#define TRANSPONDER_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
#define TRANSPONDER_DMA_CHANNEL              DMA1_Channel3
#define TRANSPONDER_IRQ                      DMA1_Channel3_IRQn
#define TRANSPONDER_DMA_TC_FLAG              DMA1_FLAG_TC3
#define TRANSPONDER_DMA_HANDLER_IDENTIFER    DMA1Channel3Descriptor

#define DEFAULT_RX_FEATURE FEATURE_RX_PPM
#define DEFAULT_FEATURES (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_AMPERAGE_METER | FEATURE_TELEMETRY)

#define OSD
#define VTX
#define GPS
#define BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define TELEMETRY
#define SERIAL_RX
#define GTUNE
#define DISPLAY
#define USE_SERVOS
#define USE_CLI
#define USE_EXTI

#define BUTTONS
#define BUTTON_A_PORT  GPIOD
#define BUTTON_A_PIN   Pin_2

#define HARDWARE_BIND_PLUG
#define BINDPLUG_PORT  BUTTON_A_PORT
#define BINDPLUG_PIN   BUTTON_A_PIN

#define SPEKTRUM_BIND
// USART3
#define BIND_PORT  GPIOA
#define BIND_PIN   Pin_3

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(4))
