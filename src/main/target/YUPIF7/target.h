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
#define TARGET_BOARD_IDENTIFIER "YPF7"

#define USBD_PRODUCT_STRING     "YUPIF7"

#define ENABLE_DSHOT_DMAR       true

#define LED0_PIN                PB4

#define USE_BEEPER
#define BEEPER_PIN              PB14
#define BEEPER_PWM_HZ           3150

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4

#define USE_MPU_DATA_READY_SIGNAL

// ICM 20689
#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define ACC_1_ALIGN             CW90_DEG

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_1_ALIGN            CW90_DEG

// Serial ports
#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PA8

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_RX_PIN      PB0 // PWM5
#define SOFTSERIAL1_TX_PIN      PB1 // PWM7

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3, USART5, USART6, SOFTSERIAL1

// *************** Dataflash ***********************
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PB5

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PA15
#define FLASH_SPI_INSTANCE      SPI3

// *************** Baro ****************************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_PULLUP
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define I2C_DEVICE              (I2CDEV_1)

#define BARO_I2C_INSTANCE       (I2CDEV_1)
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

//*********** Magnetometer / Compass *************
#define MAG_I2C_INSTANCE       (I2CDEV_1)
#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN CW270_DEG_FLIP
#define USE_MAG_QMC5883
#define MAG_QMC5883_ALIGN CW270_DEG_FLIP
#define USE_MAG_LIS3MDL

// *************** OSD *****************************
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PA14

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE         ADC1  // Default added
#define ADC1_DMA_OPT            0  // DMA 2 Stream 0 Channel 0 

#define RSSI_ADC_PIN                    PC0
#define VBAT_ADC_PIN                    PC1
#define CURRENT_METER_ADC_PIN           PC2
#define CURRENT_METER_SCALE_DEFAULT     235


// *************** Target Config *******************
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define DEFAULT_FEATURES        (FEATURE_OSD)

#define USE_ESCSERIAL

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12))
