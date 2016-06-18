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

#define TARGET_BOARD_IDENTIFIER "RGLE" // rgFC_LE

#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_1  // Blue (Rev 1 & 2) - PB4
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOC
#define LED1_GPIO   GPIOC
#define LED1_PIN    Pin_0  // Green (Rev 1) / Red (Rev 2) - PB5
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOC

#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_8
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOA
#define BEEPER_INVERTED

#define MPU6000_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOB
#define MPU6000_CS_GPIO                  GPIOB
#define MPU6000_CS_PIN                   GPIO_Pin_2
#define MPU6000_SPI_INSTANCE             SPI2

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI1_GPIO               GPIOB
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI1_NSS_PIN	        GPIO_Pin_12
//#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource12
#define SPI1_SCK_PIN            GPIO_Pin_3
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource3
#define SPI1_MISO_PIN           GPIO_Pin_4
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource4
#define SPI1_MOSI_PIN           GPIO_Pin_5
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource5

#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_NSS_PIN	        GPIO_Pin_2
//#define SPI2_NSS_PIN_SOURCE     GPIO_PinSource2
#define SPI2_SCK_PIN            GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN           GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN           GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15

#define USABLE_TIMER_CHANNEL_COUNT 9

#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define GYRO
//#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW180_DEG

#define ACC
//#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW180_DEG

//#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW

#define BARO
#define USE_BARO_BMP280

//#define MAG
//#define USE_MAG_AK8975

//#define MAG_AK8975_ALIGN CW180_DEG_FLIP

#define USE_SDCARD
#define USE_SDCARD_SPI1

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   GPIO_Pin_14
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line14
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource14
#define SDCARD_DETECT_GPIO_PORT             GPIOC
#define SDCARD_DETECT_GPIO_CLK              RCC_AHBPeriph_GPIOC
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOC
#define SDCARD_DETECT_EXTI_IRQn             EXTI15_10_IRQn

#define SDCARD_SPI_INSTANCE                 SPI1
#define SDCARD_SPI_CS_GPIO                  GPIOB
#define SDCARD_SPI_CS_PIN                   GPIO_Pin_12

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

// Note, this is the same DMA channel as USART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA1_FLAG_TC5
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define LED0
#define LED1
#define BEEPER

#define USB_IO

#define USE_VCP
#define USE_USART1
#define USE_USART2
#define USE_USART3
//#define USE_UART4
//#define USE_UART5
#define SERIAL_PORT_COUNT 4
//#define SERIAL_PORT_COUNT 6

#define UART1_TX_PIN        GPIO_Pin_4 // PC4
#define UART1_RX_PIN        GPIO_Pin_5 // PC5
#define UART1_GPIO          GPIOC
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource4
#define UART1_RX_PINSOURCE  GPIO_PinSource5

#define UART2_TX_PIN        GPIO_Pin_2 // PA2
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
/*
#define UART4_TX_PIN        GPIO_Pin_10 // PC10 (AF5)
#define UART4_RX_PIN        GPIO_Pin_11 // PC11 (AF5)
#define UART4_GPIO_AF       GPIO_AF_5
#define UART4_GPIO          GPIOC
#define UART4_TX_PINSOURCE  GPIO_PinSource10
#define UART4_RX_PINSOURCE  GPIO_PinSource11

#define UART5_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
#define UART5_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
#define UART5_GPIO_AF       GPIO_AF_7
#define UART5_GPIO          GPIOB
#define UART5_TX_PINSOURCE  GPIO_PinSource10
#define UART5_RX_PINSOURCE  GPIO_PinSource11
*/
#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // SDA (PA14/AF4), SCL (PA15/AF4)

//#define I2C1_SCL_GPIO        GPIOA
//#define I2C1_SCL_GPIO_AF     GPIO_AF_4
//#define I2C1_SCL_PIN         GPIO_Pin_15
//#define I2C1_SCL_PIN_SOURCE  GPIO_PinSource15
//#define I2C1_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOA
//#define I2C1_SDA_GPIO        GPIOA
//#define I2C1_SDA_GPIO_AF     GPIO_AF_4
//#define I2C1_SDA_PIN         GPIO_Pin_14
//#define I2C1_SDA_PIN_SOURCE  GPIO_PinSource14
//#define I2C1_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOA

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
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC2
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2
#define ADC_DMA_CHANNEL             DMA2_Channel1

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define EXTERNAL1_ADC_GPIO          GPIOA
#define EXTERNAL1_ADC_GPIO_PIN      GPIO_Pin_5
#define EXTERNAL1_ADC_CHANNEL       ADC_Channel_2

#define BLACKBOX
#define GPS
//#define GTUNE
//#define DISPLAY
#define SERIAL_RX
#define TELEMETRY
#define USE_SERVOS
#define USE_CLI
//#define SONAR

/*
#define LED_STRIP
#if 1 // <----------
#define LED_STRIP_TIMER TIM15

#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_9
#define WS2811_PIN                      GPIO_Pin_2
#define WS2811_PIN_SOURCE               GPIO_PinSource2
#define WS2811_TIMER                    TIM15
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM15
#define WS2811_DMA_CHANNEL              DMA1_Channel5
#define WS2811_IRQ                      DMA1_Channel5_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC5
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH5_HANDLER
#endif
*/
/*
#define TRANSPONDER
#define TRANSPONDER_GPIO                     GPIOB
#define TRANSPONDER_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOB
#define TRANSPONDER_GPIO_AF                  GPIO_AF_6
#define TRANSPONDER_PIN                      GPIO_Pin_10
#define TRANSPONDER_PIN_SOURCE               GPIO_PinSource10
#define TRANSPONDER_TIMER                    TIM1
#define TRANSPONDER_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define TRANSPONDER_DMA_CHANNEL              DMA1_Channel2
#define TRANSPONDER_IRQ                      DMA1_Channel2_IRQn
#define TRANSPONDER_DMA_TC_FLAG              DMA1_FLAG_TC2
#define TRANSPONDER_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER
*/
#define DEFAULT_RX_FEATURE FEATURE_RX_PPM

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

//#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
//#define USE_SERIAL_4WAY_SK_BOOTLOADER

//#if !(defined(USE_SERIAL_4WAY_BLHELI_BOOTLOADER) || defined(USE_SERIAL_4WAY_SK_BOOTLOADER))
//#ifdef USE_VCP
//#define USE_SERIAL_1WIRE_VCP
//#else
//#define USE_SERIAL_1WIRE
//#endif
//#endif

#define S1W_TX_GPIO         GPIOC
#define S1W_TX_PIN          GPIO_Pin_4
#define S1W_RX_GPIO         GPIOC
#define S1W_RX_PIN          GPIO_Pin_5

#define SPEKTRUM_BIND
// UART5, PD2
#define BIND_PORT GPIOD
#define BIND_PIN Pin_2
