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

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_EXTI
#define EXTI_CALLBACK_HANDLER_COUNT 3 // MPU data ready (mag disabled)
#define EXTI15_10_CALLBACK_HANDLER_COUNT 2 // MPU_INT, SDCardDetect

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6000_CS_PIN          PA4
#define MPU_INT_EXTI PC13
#define USE_MPU_DATA_READY_SIGNAL
#define GYRO_MPU6000_ALIGN      CW90_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW90_DEG

#define BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI1
#define BMP280_CS_PIN           PA13
#define USE_BARO_BMP085 // External
#define USE_BARO_BMP180 // External
#define USE_BARO_MS5611 // External

#define MAG
#define USE_MAG_AK8963  // External
#define USE_MAG_AK8975  // External
#define USE_MAG_HMC5883 // External
#define USE_MAG_MAG3110 // External
#define USE_MAG_QMC5883 // External

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_ECHO_PIN          PB2  // Has 1K series resistor
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB4  // FT

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

// Enable I2C instead of PWM7&8 for iNav
#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL(PWM8), PB7/SDA(PWM7)
// Because the I2C is shared with PWM7&8, there are no on-board ext. pullups.
// Turn internal pullups, they are weak, but better than nothing.
#define USE_I2C_PULLUP

#define USE_PITOT_MS4525

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//#define USE_RX_SPI
#define RX_SPI_INSTANCE SPI2
#define RX_NSS_PIN PB3

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

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB1
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)
//#define MAX7456_DMA_CHANNEL_TX            DMA1_Channel3
//#define MAX7456_DMA_CHANNEL_RX            DMA1_Channel2
//#define MAX7456_DMA_IRQ_HANDLER_ID        DMA1_CH3_HANDLER

#define USE_ADC
#define ADC_INSTANCE                ADC1
//#define BOARD_HAS_VOLTAGE_DIVIDER
#define ADC_CHANNEL_1_PIN           PA0
#define ADC_CHANNEL_2_PIN           PA1
#define ADC_CHANNEL_3_PIN           PB2
#define ADC_CHANNEL_3_INSTANCE      ADC2
#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

#define LED_STRIP
#define WS2811_PIN                      PA8
#define WS2811_DMA_STREAM               DMA1_Channel2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

//#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        (FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_BLACKBOX)

#define BUTTONS
#define BUTTON_A_PORT           GPIOB // Non-existent (PB1 used for RSSI/MAXCS)
#define BUTTON_A_PIN            Pin_1
#define BUTTON_B_PORT           GPIOB // TRIG button, used for BINDPLUG_PIN
#define BUTTON_B_PIN            Pin_0

#define SPEKTRUM_BIND
// USART3
#define BIND_PIN                PB11

#define HARDWARE_BIND_PLUG
#define BINDPLUG_PIN            PB0

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT  8

#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15))
