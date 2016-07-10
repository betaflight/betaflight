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
#define USE_HARDWARE_REVISION_DETECTION

#define BOARD_HAS_VOLTAGE_DIVIDER

#define LED0                PB3
#define LED1                PB4

#define BEEPER              PA12
#ifdef AFROMINI
#define BEEPER_INVERTED
#endif

#define INVERTER            PB2 // PB2 (BOOT1) abused as inverter select GPIO
#define INVERTER_USART      USART2

#define BARO_XCLR_GPIO   GPIOC
#define BARO_XCLR_PIN    Pin_13
#define BARO_EOC_GPIO    GPIOC
#define BARO_EOC_PIN     Pin_14
#define BARO_APB2_PERIPHERALS RCC_APB2Periph_GPIOC

// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

#define USE_SPI
#define USE_SPI_DEVICE_2

#define NAZE_SPI_INSTANCE     SPI2
#define NAZE_SPI_CS_GPIO      GPIOB
#define NAZE_SPI_CS_PIN       GPIO_Pin_12
#define NAZE_CS_GPIO_CLK_PERIPHERAL RCC_APB2Periph_GPIOB

// We either have this 16mbit flash chip on SPI or the MPU6500 acc/gyro depending on board revision:
#define M25P16_CS_GPIO        NAZE_SPI_CS_GPIO
#define M25P16_CS_PIN         NAZE_SPI_CS_PIN
#define M25P16_SPI_INSTANCE   NAZE_SPI_INSTANCE

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   NAZE_CS_GPIO_CLK_PERIPHERAL
#define MPU6500_CS_GPIO                  NAZE_SPI_CS_GPIO
#define MPU6500_CS_PIN                   NAZE_SPI_CS_PIN
#define MPU6500_SPI_INSTANCE             NAZE_SPI_INSTANCE


#define USE_FLASHFS

#define USE_FLASH_M25P16

#define EXTI_CALLBACK_HANDLER_COUNT 3 // MPU data ready, MAG data ready, BMP085 EOC

//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL

//#define DEBUG_MAG_DATA_READY_INTERRUPT
#define USE_MAG_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500


#define GYRO_MPU3050_ALIGN CW0_DEG
#define GYRO_MPU6050_ALIGN CW0_DEG
#define GYRO_MPU6500_ALIGN CW0_DEG

#define ACC
#define USE_ACC_ADXL345
//#define USE_ACC_BMA280
//#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500

#define ACC_ADXL345_ALIGN CW270_DEG
#define ACC_MPU6050_ALIGN CW0_DEG
#define ACC_MMA8452_ALIGN CW90_DEG
#define ACC_BMA280_ALIGN CW0_DEG
#define ACC_MPU6500_ALIGN CW0_DEG

#define BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#define MAG
#define USE_MAG_HMC5883
#define USE_MAG_AK8975
#define USE_MAG_MAG3110

#define MAG_HMC5883_ALIGN CW180_DEG

#define SONAR
#define USE_SONAR_SRF10
#define SONAR_PWM_TRIGGER_PIN       Pin_8   // PWM5 (PB8) - 5v tolerant
#define SONAR_PWM_TRIGGER_GPIO      GPIOB
#define SONAR_PWM_ECHO_PIN          Pin_9   // PWM6 (PB9) - 5v tolerant
#define SONAR_PWM_ECHO_GPIO         GPIOB
#define SONAR_PWM_EXTI_LINE         EXTI_Line9
#define SONAR_PWM_EXTI_PIN_SOURCE   GPIO_PinSource9
#define SONAR_PWM_EXTI_IRQN         EXTI9_5_IRQn
#define SONAR_TRIGGER_PIN           Pin_0   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
#define SONAR_TRIGGER_GPIO          GPIOB
#define SONAR_ECHO_PIN              Pin_1   // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
#define SONAR_ECHO_GPIO             GPIOB
#define SONAR_EXTI_LINE             EXTI_Line1
#define SONAR_EXTI_PIN_SOURCE       GPIO_PinSource1
#define SONAR_EXTI_IRQN             EXTI1_IRQn

#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 5

#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4 // PWM 5
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5 // PWM 6
#define SOFTSERIAL_2_TIMER TIM3
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6 // PWM 7
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7 // PWM 8

// USART3 only on NAZE32_SP - Flex Port
#define USART3_RX_PIN Pin_11
#define USART3_TX_PIN Pin_10
#define USART3_GPIO GPIOB
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_APB2_PERIPHERALS RCC_APB2Periph_GPIOB

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

//#define USE_RX_NRF24
#ifdef USE_RX_NRF24

#define USE_RX_SYMA
//#define USE_RX_V202
#define NRF24_DEFAULT_PROTOCOL NRF24RX_SYMA_X5C

#define USE_SOFTSPI
#define USE_NRF24_SOFTSPI
// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67
// RC pinouts
// RC3              RX_PPM
// RC4  PA1         CE / RSSI_ADC
// RC5  PA2         USART2 TX
// RC6  PA3         USART2 RX
// RC7  PA6/TIM3    CSN / softserial1 RX / LED_STRIP
// RC8  PA7         SCK / softserial1 TX
// RC9  PB0         MISO / softserial2 RX / sonar trigger
// RC10 PB1         MOSI /softserial2 TX / sonar echo / current

// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define NRF24_CE_GPIO                   GPIOA
#define NRF24_CE_PIN                    GPIO_Pin_1
#define NRF24_CE_GPIO_CLK_PERIPHERAL    RCC_APB2Periph_GPIOA
#define NRF24_CSN_GPIO                  GPIOA
#define NRF24_CSN_PIN                   GPIO_Pin_6
#define NRF24_CSN_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA
#define NRF24_SCK_GPIO                  GPIOA
#define NRF24_SCK_PIN                   GPIO_Pin_7
#define NRF24_MOSI_GPIO                 GPIOB
#define NRF24_MOSI_PIN                  GPIO_Pin_1
#define NRF24_MISO_GPIO                 GPIOB
#define NRF24_MISO_PIN                  GPIO_Pin_0
#endif // USE_NRF24

#define USE_ADC

#define CURRENT_METER_ADC_GPIO      GPIOB
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_9

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_4

#define RSSI_ADC_GPIO               GPIOA
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_1
#define RSSI_ADC_CHANNEL            ADC_Channel_1

#define EXTERNAL1_ADC_GPIO          GPIOA
#define EXTERNAL1_ADC_GPIO_PIN      GPIO_Pin_5
#define EXTERNAL1_ADC_CHANNEL       ADC_Channel_5

#define NAV
//#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION

#define LED_STRIP
#define LED_STRIP_TIMER TIM3

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PORT  GPIOA
#define BIND_PIN   Pin_3

//#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_MOTOR_COUNT 8

// alternative defaults for ALIENFLIGHTF1 F1 target
#ifdef ALIENFLIGHTF1
#undef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "AFF1" // ALIENFLIGHTF1
#undef BOARD_HAS_VOLTAGE_DIVIDER

#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define DEFAULT_FEATURES FEATURE_MOTOR_STOP

#define HARDWARE_BIND_PLUG
// Hardware bind plug at PB5 (Pin 41)
#define BINDPLUG_PORT  GPIOB
#define BINDPLUG_PIN   Pin_5
#endif // ALIENFLIGHTF1

#ifdef MICROSCISKY
#undef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "MSKY" // Micro sciSKY
#define BRUSHED_MOTORS
#define USE_QUAD_MIXER_ONLY
#undef USE_SERVOS
#undef BEEPER
#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL

#endif

#undef TELEMETRY_FRSKY
#undef TELEMETRY_HOTT
#undef TELEMETRY_SMARTPORT


// IO - assuming all IOs on 48pin package
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC ( BIT(13) | BIT(14) | BIT(15) )

#define USED_TIMERS     ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
