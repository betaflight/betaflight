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
#define TARGET_BOARD_IDENTIFIER "YPF4"

#define USBD_PRODUCT_STRING     "YupiF4"

#define LED0                    PB6
#define LED1                    PB4
#define LED2                    PB5

#define BEEPER                  PB14
#define BEEPER_PWM
#define BEEPER_INVERTED
#define BEEPER_PWM_TIMER        TIM12
#define BEEPER_PWM_TIMER_CH     TIM_Channel_1
#define BEEPER_PWM_FREQUENCY    3100


#define INVERTER_PIN_UART6      PB15

#define USE_SPI
#define USE_SPI_DEVICE_1 // Gyro
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7


/////// ICM20689 ////////
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

// ICM20689
#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI1

#define ACC
//#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW90_DEG

#define GYRO
//#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
//#define USE_GYRO_SPI_ICM20689
#define GYRO_MPU6500_ALIGN      CW90_DEG

////////////////////////

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define BARO
#define USE_BARO_BMP280

// Serial Ports
#define USB_IO
#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       4 // VCP, UART1, UART3, UART6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0


// SD Card
#define USE_SDCARD
#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PD2
#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PA15

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0


// SPI ports definition

#define USE_SPI_DEVICE_3 // SD Card
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

// I2C port definition
#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#define USE_I2C_PULLUP

// ADC inputs
#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_CHANNEL_1_PIN       PC1
#define ADC_CHANNEL_2_PIN       PC0
#define VBAT_ADC_CHANNEL        ADC_CHN_1
#define RSSI_ADC_CHANNEL        ADC_CHN_2

// LED Strip can run off Pin 5 (PB1) of the motor outputs
#define LED_STRIP
#define WS2811_PIN                      PB1
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_CHANNEL              DMA_Channel_5

// Features
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        FEATURE_BLACKBOX

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(12))

