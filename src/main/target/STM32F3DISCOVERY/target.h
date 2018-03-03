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
 *
 * This software is free software: you can redistribute it and/or modify
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

#define TARGET_BOARD_IDENTIFIER "SDF3" // STM Discovery F3

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#undef USE_OSD // ROM SAVING

#define CURRENT_TARGET_CPU_VOLTAGE 3.0

#define LED0_PIN                PE8  // Blue LEDs - PE8/PE12
#define LED0_INVERTED
#define LED1_PIN                PE10 // Orange LEDs - PE10/PE14
#define LED1_INVERTED

#define BEEPER                  PD12
#define BEEPER_PWM_HZ           2000  // Beeper PWM frequency in Hz

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//#define USE_SD_CARD
//
//#define SD_DETECT_PIN           PC14
//#define SD_CS_PIN               PB12
//#define SD_SPI_INSTANCE         SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2
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
#define USE_FAKE_GYRO
#define USE_GYRO_L3GD20
#define L3GD20_SPI              SPI1
#define L3GD20_CS_PIN           PE3
#define GYRO_L3GD20_ALIGN       CW270_DEG
#define USE_GYRO_L3G4200D
#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_SPI_MPU6000
#define MPU6000_CS_PIN          SPI2_NSS_PIN
#define MPU6000_SPI_INSTANCE    SPI2
// Support the GY-91 MPU9250 dev board
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define MPU6500_CS_PIN          PC14
#define MPU6500_SPI_INSTANCE    SPI2
#define GYRO_MPU6500_ALIGN      CW270_DEG_FLIP
#define USE_GYRO_SPI_MPU9250
#define MPU9250_CS_PIN          SPI2_NSS_PIN
#define MPU9250_SPI_INSTANCE    SPI2
// BMI160 gyro support
//#define USE_ACCGYRO_BMI160
#ifdef USE_ACCGYRO_BMI160
#define BMI160_CS_PIN           SPI2_NSS_PIN
#define BMI160_SPI_INSTANCE     SPI2
#define BMI160_SPI_DIVISOR      16
#define BMI160_INT_EXTI         PC13
#define USE_MPU_DATA_READY_SIGNAL
#define USE_EXTI
#endif

#define USE_ACC
#define USE_FAKE_ACC
#define USE_ACC_ADXL345
#define USE_ACC_BMA280
#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
#define USE_ACC_LSM303DLHC
#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU6500_ALIGN       CW270_DEG_FLIP

#define USE_BARO
#define USE_FAKE_BARO
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

//#define USE_SDCARD
//
//#define SDCARD_SPI_INSTANCE     SPI2
//#define SDCARD_SPI_CS_PIN       PB12
//// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
//#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
//// Divide to under 25MHz for normal operation:
//#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 2
//
//// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
//#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USE_MAG
#define USE_FAKE_MAG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883

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

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define LSM303DLHC_I2C                       I2C1
#define LSM303DLHC_I2C_SCK_PIN               PB6
#define LSM303DLHC_I2C_SDA_PIN               PB7
#define LSM303DLHC_DRDY_PIN                  PE2
#define LSM303DLHC_I2C_INT1_PIN              PE4
#define LSM303DLHC_I2C_INT2_PIN              PE5

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

#define USE_ESC_SENSOR

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN       PB0
#define RANGEFINDER_HCSR04_ECHO_PIN          PB1

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

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
