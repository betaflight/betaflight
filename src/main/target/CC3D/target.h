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

#define TARGET_BOARD_IDENTIFIER "CC3D" // CopterControl 3D

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOB
#define LED0

#define INVERTER_PIN Pin_2 // PB2 (BOOT1) used as inverter select GPIO
#define INVERTER_GPIO GPIOB
#define INVERTER_PERIPHERAL RCC_APB2Periph_GPIOB
#define INVERTER_USART USART1

#define BEEP_GPIO GPIOA
#define BEEP_PIN Pin_15 // PA15 (Beeper)
#define BEEP_PERIPHERAL RCC_APB2Periph_GPIOA

#define MPU6000_CS_GPIO       GPIOA
#define MPU6000_CS_PIN        GPIO_Pin_4
#define MPU6000_SPI_INSTANCE  SPI1

#define ACC
#define USE_ACC_SPI_MPU6000

#define GYRO
#define USE_GYRO_SPI_MPU6000

#define INVERTER
#define BEEPER

#define USE_USART1
#define USE_USART3
#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT 3

#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 2 // PWM 3
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 3 // PWM 4


#define USART3_RX_PIN Pin_11
#define USART3_TX_PIN Pin_10
#define USART3_GPIO GPIOB
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_APB2_PERIPHERALS RCC_APB2Periph_GPIOB



#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2


#define SENSORS_SET (SENSOR_ACC)

#define GPS
#define LED_STRIP
#define LED_STRIP_TIMER TIM3

#define TELEMETRY
#define SERIAL_RX
#define AUTOTUNE
