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

#define TARGET_BOARD_IDENTIFIER "AFNA" // AFroNAze - NAZE might be considered misleading on Naze clones like the flip32.

#define LED0                    PB3
#define LED1                    PB4

#define BEEPER                  PA12

#define INVERTER_PIN_UART2      PB2 // PB2 (BOOT1) abused as inverter select GPIO

#define USE_EXTI
#define MPU_INT_EXTI            PC13
#define USE_MPU_DATA_READY_SIGNAL
#define MAG_INT_EXTI            PC14
#define USE_MAG_DATA_READY_SIGNAL

// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

#define USE_SPI
#define USE_SPI_DEVICE_2

#define NAZE_SPI_INSTANCE       SPI2
#define NAZE_SPI_CS_GPIO        GPIOB
#define NAZE_SPI_CS_PIN         PB12
#define NAZE_CS_GPIO_CLK_PERIPHERAL RCC_APB2Periph_GPIOB

#define USE_UART1
#define USE_UART2
#define USE_SOFTSERIAL1

#ifdef AIRHERO32
    // MWC PARIS Sirius AirHero32
    #define USE_GYRO_SPI_MPU6500
    #define USE_ACC_SPI_MPU6500

    #define USE_UART3
    #define UART3_RX_PIN            PB11
    #define UART3_TX_PIN            PB10

    #define I2C_DEVICE_SHARES_UART3

    #define SERIAL_PORT_COUNT       4       // UART1, UART2, UART3, SS1

    #define MPU6500_CS_GPIO_CLK_PERIPHERAL  NAZE_CS_GPIO_CLK_PERIPHERAL
    #define MPU6500_CS_GPIO                 NAZE_SPI_CS_GPIO
    #define MPU6500_CS_PIN                  NAZE_SPI_CS_PIN
    #define MPU6500_SPI_INSTANCE            NAZE_SPI_INSTANCE
#else
    // Afroflight NAZE
    #define USE_HARDWARE_REVISION_DETECTION
    #define USE_SOFTSERIAL2

    #define M25P16_CS_GPIO          NAZE_SPI_CS_GPIO
    #define M25P16_CS_PIN           NAZE_SPI_CS_PIN
    #define M25P16_SPI_INSTANCE     NAZE_SPI_INSTANCE

    #define SERIAL_PORT_COUNT       4       // UART1, UART2, SS1, SS2

    #define USE_FLASHFS
    #define USE_FLASH_M25P16
#endif

#define GYRO
#define USE_GYRO_MPU6050
#define USE_GYRO_MPU6500

#define GYRO_MPU6050_ALIGN      CW0_DEG
#define GYRO_MPU6500_ALIGN      CW0_DEG

#define ACC
#define USE_ACC_MPU6050
#define USE_ACC_MPU6500

#define ACC_MPU6050_ALIGN       CW0_DEG
#define ACC_MPU6500_ALIGN       CW0_DEG

#define BARO
#define USE_BARO_MS5611 // needed for Flip32 board
#define USE_BARO_BMP280

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG

// #define USE_RANGEFINDER
// #define USE_RANGEFINDER_HCSR04
// #define USE_RANGEFINDER_SRF10
// #define RANGEFINDER_HCSR04_TRIGGER_PIN       PB0
// #define RANGEFINDER_HCSR04_ECHO_PIN          PB1
// #define RANGEFINDER_HCSR04_TRIGGER_PIN_PWM   PB8
// #define RANGEFINDER_HCSR04_ECHO_PIN_PWM      PB9

#define SOFTSERIAL_1_RX_PIN     PA6
#define SOFTSERIAL_1_TX_PIN     PA7
#define SOFTSERIAL_2_RX_PIN     PB0
#define SOFTSERIAL_2_TX_PIN     PB1

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

//#define USE_RX_NRF24
#ifdef USE_RX_NRF24
#define USE_RX_SPI

#define USE_RX_CX10
#define USE_RX_H8_3D
#define USE_RX_INAV
//#define USE_RX_SYMA
//#define USE_RX_V202
//#define NRF24_DEFAULT_PROTOCOL  NRF24RX_SYMA_X5C
//#define NRF24_DEFAULT_PROTOCOL  NRF24RX_REF
#define NRF24_DEFAULT_PROTOCOL  NRF24RX_H8_3D
//#define NRF24_DEFAULT_PROTOCOL  NRF24RX_CX10A
//#define NRF24_DEFAULT_PROTOCOL  NRF24RX_V202_1M

#define USE_SOFTSPI
#define USE_RX_SOFTSPI

// RC pinouts
// RC1              GND
// RC2              power
// RC3  PA0/TIM2    RX_PPM
// RC4  PA1/TIM2    CE / RSSI_ADC
// RC5  PA2/TIM2    USART2 TX
// RC6  PA3/TIM2    USART2 RX
// RC7  PA6/TIM3    CSN / softserial1 RX / LED_STRIP
// RC8  PA7/TIM3    SCK / softserial1 TX
// RC9  PB0/TIM3    MISO / softserial2 RX / HC-SR04 trigger
// RC10 PB1/TIM3    MOSI /softserial2 TX / HC-SR04 echo / current

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define RX_CE_PIN                   PA1
#define RX_CE_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define RX_NSS_PIN                  PA6
#define RX_NSS_GPIO_CLK_PERIPHERAL  RCC_APB2Periph_GPIOA
#define RX_SCK_PIN                  PA7
#define RX_MOSI_PIN                 PB1
#define RX_MISO_PIN                 PB0
#endif // USE_NRF24

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PB1
#define ADC_CHANNEL_2_PIN               PA4
#define ADC_CHANNEL_3_PIN               PA1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

//#define NAV_AUTO_MAG_DECLINATION
//#define NAV_GPS_GLITCH_DETECTION

// #define LED_STRIP
// #define WS2811_PIN                      PA6
// #define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC6
// #define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH6_HANDLER

#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_IBUS
//#define SPEKTRUM_BIND
//#define BIND_PIN                PA3

//#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_MOTOR_COUNT      6

#define DEFAULT_FEATURES        FEATURE_VBAT
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
