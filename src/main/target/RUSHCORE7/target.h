/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "RSF7"
#define USBD_PRODUCT_STRING  "RUSHCORE7"

#define ENABLE_DSHOT_DMAR       true
#define LED0_PIN                PC13

#define USE_BEEPER
#define BEEPER_PIN              PB1
#define BEEPER_INVERTED
//#define BEEPER_PWM_HZ           1100

#define USE_EXTI
#define USE_GYRO_EXTI
#define USE_MPU_DATA_READY_SIGNAL                   

#define USE_ACC
#define USE_GYRO
#define USE_GYRO_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

#define GYRO_1_CS_PIN           PA4 
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_EXTI_PIN PC4
#define GYRO_1_ALIGN      CW270_DEG_FLIP

#define USE_BARO
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_HMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PC10
#define UART3_RX_PIN            PC11

#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2
#define SERIAL_PORT_COUNT       6 

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              I2CDEV_1
#define I2C1_SCL                PB8      
#define I2C1_SDA                PB9

#define USE_SPI
#define USE_SPI_DEVICE_1 //GYRO/ACC
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2 //MAX7456
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12

#define USE_SPI_DEVICE_3 // FLASH 
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PC15
#define FLASH_SPI_INSTANCE      SPI3

#define USE_ADC
#define ADC_INSTANCE                        ADC3
#define ADC3_DMA_STREAM                     DMA2_Stream0

#define VBAT_ADC_PIN                        PC1
#define CURRENT_METER_ADC_PIN               PC3
#define RSSI_ADC_PIN                        PA0

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_OSD
#define USE_LED_STRIP

#define DEFAULT_RX_FEATURE                  FEATURE_RX_SERIAL
#define DEFAULT_FEATURES                    FEATURE_OSD
#define SERIALRX_UART                       SERIAL_PORT_USART2
#define SERIALRX_PROVIDER                   SERIALRX_SBUS


#define USE_SERIAL_4WAY_BLHELI_INTERFACE


#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(9) )
