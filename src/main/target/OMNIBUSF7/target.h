/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "OBF7"

#define USBD_PRODUCT_STRING "OmnibusF7"

#define LED0   PE0

#define BEEPER   PD15
#define BEEPER_INVERTED

#define ACC
#define GYRO

// ICM-20608-G
// #define USE_ACC_MPU6500
// #define USE_ACC_SPI_MPU6500
// #define USE_GYRO_MPU6500
// #define USE_GYRO_SPI_MPU6500
//#define ACC_MPU6500_ALIGN       CW0_DEG
//#define GYRO_MPU6500_ALIGN      CW0_DEG
// #define MPU6500_CS_PIN          SPI1_NSS_PIN
// #define MPU6500_SPI_INSTANCE    SPI1
//#define MPU_INT_EXTI            PE8

// MPU6000
// #define USE_ACC_MPU6000
// #define USE_ACC_SPI_MPU6000
// #define USE_GYRO_MPU6000
// #define USE_GYRO_SPI_MPU6000
// #define ACC_MPU6000_ALIGN       CW0_DEG
// #define GYRO_MPU6000_ALIGN      CW0_DEG
// #define MPU6000_CS_PIN          SPI3_NSS_PIN
// #define MPU6000_SPI_INSTANCE    SPI3
// #define MPU_INT_EXTI            PD0

// #define USE_DUAL_GYRO
// #define GYRO_0_CS_PIN           MPU6000_CS_PIN
// #define GYRO_1_CS_PIN           MPU6500_CS_PIN

// TODO: dual gyro support
//#define USE_MPU_DATA_READY_SIGNAL
//#define USE_EXTI

#define USE_EXTI
#define MPU_INT_EXTI            PD0
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define MPU6000_CS_PIN          PA15
#define MPU6000_SPI_INSTANCE    SPI3

#define ACC
#define USE_ACC_SPI_MPU6000

#define GYRO_MPU6000_ALIGN      CW0_DEG
#define ACC_MPU6000_ALIGN       CW0_DEG

#define USABLE_TIMER_CHANNEL_COUNT 16

#define USB_IO
#define USE_VCP
#define VBUS_SENSING_PIN PC4

#define USE_UART1
#define UART1_RX_PIN PA10
#define UART1_TX_PIN PA9

//#define AVOID_UART2_FOR_PWM_PPM
#define USE_UART2
#define UART2_TX_PIN PA2 //not wired
#define UART2_RX_PIN PA3

// Assigned to shared output I2C2
#define USE_UART3
#define UART3_RX_PIN PB10
#define UART3_TX_PIN PB11

#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

#define SERIAL_PORT_COUNT 5 //VCP, USART1, USART2, USART3, USART6

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define SPI4_NSS_PIN            PE4
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6


#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_SDCARD
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PE3
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line3
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource3
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOE
#define SDCARD_DETECT_EXTI_IRQn             EXTI3_IRQn

#define SDCARD_SPI_INSTANCE                 SPI4
#define SDCARD_SPI_CS_PIN                   SPI4_NSS_PIN

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 422kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 8 // 27MHz

#define SDCARD_DMA_CHANNEL_TX               DMA2_Stream1
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF1_5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA2
#define SDCARD_DMA_CHANNEL                  DMA_CHANNEL_4

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_2)
#define I2C_DEVICE_SHARES_UART3

#define BARO
#define USE_BARO_BMP085
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI1
#define BMP280_CS_PIN           PA1

#define MAG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN       CW270_DEG_FLIP
#define USE_MAG_MAG3110

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO)

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC2
#define ADC_CHANNEL_2_PIN               PC3
#define ADC_CHANNEL_3_PIN               PC5
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// #define LED_STRIP

//Following configuration needs to be reviewed, when LED is enabled, VCP stops to work
//Until someone with deeper knowledge od DMA fixes it, LED are disabled in target
#define WS2811_PIN                      PD12
#define WS2811_TIMER                    TIM4
#define WS2811_TIMER_CHANNEL            TIM_CHANNEL_1
#define WS2811_DMA_HANDLER_IDENTIFER    DMA2_ST4_HANDLER
#define WS2811_DMA_STREAM               DMA2_Stream4
#define WS2811_DMA_IT                   DMA_IT_TCIF2
#define WS2811_DMA_CHANNEL              DMA_CHANNEL_6
#define WS2811_TIMER_GPIO_AF            GPIO_AF2_TIM4

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_FEATURES        (FEATURE_OSD)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    4
#define TARGET_MOTOR_COUNT      4

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
