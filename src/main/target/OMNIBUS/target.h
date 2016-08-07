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

#define TARGET_BOARD_IDENTIFIER "OMNI" // https://en.wikipedia.org/wiki/Omnibus

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define LED0                    PB3

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI PC13
#define USE_MPU_DATA_READY_SIGNAL
#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU data ready (mag disabled)
#define EXTI15_10_CALLBACK_HANDLER_COUNT 2 // MPU_INT, SDCardDetect

#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6000_CS_PIN          PA4

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW90_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW90_DEG

#define BMP280_SPI_INSTANCE     SPI1
#define BMP280_CS_PIN           PA13

#define BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280

//#define MAG
//#define USE_MAG_AK8975
//#define USE_MAG_HMC5883 // External
//
//#define MAG_AK8975_ALIGN CW90_DEG_FLIP

//#define SONAR
//#define SONAR_ECHO_PIN          PB1
//#define SONAR_TRIGGER_PIN       PB0

#define USB_IO
#define USB_CABLE_DETECTION
#define USB_DETECT_PIN          PB5

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT       4

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

//#define USE_I2C
//#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

// OSD define info:
//   feature name (includes source) -> MAX_OSD, used in target.mk
// include the osd code
#define OSD
// include the max7456 driver
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB1
//#define MAX7456_DMA_CHANNEL_TX            DMA1_Channel3
//#define MAX7456_DMA_CHANNEL_RX            DMA1_Channel2
//#define MAX7456_DMA_IRQ_HANDLER_ID        DMA1_CH3_HANDLER

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SDCARD
#define USE_SDCARD_SPI2

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PC14
#define SDCARD_SPI_INSTANCE                 SPI2
#define SDCARD_SPI_CS_PIN                   SPI2_NSS_PIN

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     2

// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA1_FLAG_TC5

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USE_ADC
//#define BOARD_HAS_VOLTAGE_DIVIDER
#define VBAT_ADC_PIN                PA0
#define CURRENT_METER_ADC_PIN       PA1
#define ADC_INSTANCE                ADC1

//#define RSSI_ADC_PIN                PB1
//#define ADC_INSTANCE                ADC3


#define LED_STRIP
#define WS2811_PIN                      PA8
#define WS2811_TIMER                    TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

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

#define REDUCE_TRANSPONDER_CURRENT_DRAW_WHEN_USB_CABLE_PRESENT

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        (FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_BLACKBOX)

#define BUTTONS
#define BUTTON_A_PORT           GPIOB
#define BUTTON_A_PIN            Pin_1
#define BUTTON_B_PORT           GPIOB
#define BUTTON_B_PIN            Pin_0

#define AVOID_UART3_FOR_PWM_PPM

#define SPEKTRUM_BIND
// USART3,
#define BIND_PIN                PB11

#define HARDWARE_BIND_PLUG
#define BINDPLUG_PIN            PB0

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 10 // 6 Outputs; PPM; LED Strip; 2 additional PWM pins also on UART3 RX/TX pins.
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15))
