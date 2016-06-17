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
#define TARGET_BOARD_IDENTIFIER "REVN"

#define CONFIG_START_FLASH_ADDRESS (0x08060000) //0x08060000 to 0x08080000 (FLASH_Sector_7)
#define CONFIG_SERIALRX_PROVIDER 2
#define CONFIG_BLACKBOX_DEVICE 1
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_FEATURE_ONESHOT125
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 2

#define USBD_PRODUCT_STRING "Revo Nano"
#ifdef OPBL
	#define USBD_SERIALNUMBER_STRING "0x8010000"
#endif

#define LED0 PC14
#define LED1 PC13
#define BEEPER PC13
#define INVERTER PC15
#define INVERTER_USART USART2 //Sbus on USART 2 of nano.

#define MPU9250_CS_PIN        PB12
#define MPU9250_SPI_INSTANCE  SPI2

#define ACC
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU9250_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_MPU9250
#define USE_GYRO_SPI_MPU9250
#define GYRO_MPU9250_ALIGN CW270_DEG

//#define MAG
//#define USE_MAG_HMC5883

#define BARO
#define USE_BARO_MS5611

// MPU9250 interrupts
#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready (mag disabled)
#define MPU_INT_EXTI    PA15
#define USE_EXTI

#define USABLE_TIMER_CHANNEL_COUNT 12

#define USE_VCP
#define VBUS_SENSING_PIN PA9

#define USE_USART1
#define USART1_RX_PIN PB7
#define USART1_TX_PIN PB6

#define USE_USART2
#define USART2_RX_PIN PA3
#define USART2_TX_PIN PA2

#define SERIAL_PORT_COUNT 3 //VCP, USART1, USART2

#define USE_SPI
//#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
//#define USE_SPI_DEVICE_3

#define USE_I2C
#define I2C_DEVICE (I2CDEV_3)

#define USE_ADC

//FLEXI-IO	6
#define CURRENT_METER_ADC_PIN       PA7
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_7

//FLEXI-IO	7
#define VBAT_ADC_PIN                PA6
#define VBAT_ADC_CHANNEL            ADC_Channel_6

//FLEXI-IO	8
#define RSSI_ADC_PIN                PA5
#define RSSI_ADC_CHANNEL            ADC_Channel_5


//#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG)

//#define LED_STRIP
//#define LED_STRIP_TIMER TIM5

#define GPS
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff


#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) )

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1)