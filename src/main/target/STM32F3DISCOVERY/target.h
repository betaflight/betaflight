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

/*
 * Supports the GY-91 MPU9250 and BMP280 development board via SPI1
 *
 * Put the MAX7456 on SPI2 instead of an SDCARD
 *  MAX7456 CS -> PB12 (default)
 *  Uses the default pins for SPI2:
 *    #define SPI2_NSS_PIN    PB12
 *    #define SPI2_SCK_PIN    PB13
 *    #define SPI2_MISO_PIN   PB14
 *    #define SPI2_MOSI_PIN   PB15
 *
 * @author Nathan Tsoi
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SDF3" // STM Discovery F3

#define USE_SENSOR_NAMES

#define CURRENT_TARGET_CPU_VOLTAGE 3.0

#define LED0_PIN                PE8  // Blue LEDs - PE8/PE12
#define LED0_INVERTED
#define LED1_PIN                PE10 // Orange LEDs - PE10/PE14
#define LED1_INVERTED

#define USE_BEEPER
#define BEEPER_PIN              PD12
#define BEEPER_PWM_HZ           2000  // Beeper PWM frequency in Hz

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//#define USE_SD_CARD

//#define SD_DETECT_PIN           PC14
//#define SD_CS_PIN               PB12
//#define SD_SPI_INSTANCE         SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2
// SPI1
// PB5  SPI1_MOSI
// PB4  SPI1_MISO
// PB3  SPI1_SCK
// PA15 SPI1_NSS

// SPI2
// PB15 SPI2_MOSI
// PB14 SPI2_MISO
// PB13 SPI2_SCK
// PB12 SPI2_NSS

#define USE_GYRO

// The on-board gyro
#define USE_GYRO_L3GD20
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PE3
#define GYRO_1_ALIGN            CW270_DEG

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PE1
#define USE_MPU_DATA_READY_SIGNAL

// Other gyros:
#define USE_FAKE_GYRO
//#define USE_GYRO_L3G4200D
#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250
#define USE_ACCGYRO_BMI160

#define USE_ACC

// The on-board acc:
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define MPU_I2C_INSTANCE        (I2CDEV_1)
#define USE_ACC_LSM303DLHC

#define USE_FAKE_ACC
//#define USE_ACC_ADXL345
//#define USE_ACC_BMA280
//#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250

#define USE_BARO
#define USE_FAKE_BARO
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611

//#define USE_MAX7456
//#define MAX7456_SPI_INSTANCE    SPI2
//#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

//#define USE_SDCARD
//#define SDCARD_SPI_INSTANCE     SPI2
//#define SDCARD_SPI_CS_PIN       PB12
//// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
//#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USE_MAG
#define USE_FAKE_MAG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8  // (HARDARE=0,PPM)

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB0
#define RANGEFINDER_HCSR04_ECHO_PIN          PB1

#define MAX_SUPPORTED_MOTORS    12

// IO - 303 in 100pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         0x00ff

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16) | TIM_N(17))
