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

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "M7B"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight X32M7B"
#endif

#undef BARO_I2C_INSTANCE
#undef MAG_I2C_INSTANCE


#undef USE_MAG_DATA_READY_SIGNAL
#undef USE_MAG_HMC5883
#undef USE_MAG_SPI_HMC5883
#undef USE_MAG_QMC5883
#undef USE_MAG_LIS2MDL
#undef USE_MAG_LIS3MDL
#undef USE_MAG_AK8963
#undef USE_MAG_MPU925X_AK8963
#undef USE_MAG_SPI_AK8963
#undef USE_MAG_AK8975
#undef USE_MAG_IST8310
#undef USE_MAG_MMC560X

#undef USE_BARO_MS5611
#undef USE_BARO_SPI_MS5611
#undef USE_BARO_BMP280
#undef USE_BARO_SPI_BMP280
#undef USE_BARO_BMP388
#undef USE_BARO_SPI_BMP388
#undef USE_BARO_LPS
#undef USE_BARO_SPI_LPS
#undef USE_BARO_QMP6988
#undef USE_BARO_SPI_QMP6988
#undef USE_BARO_DPS310
#undef USE_BARO_SPI_DPS310
#undef USE_BARO_BMP085
#undef USE_BARO_2SMBP_02B
#undef USE_BARO_SPI_2SMBP_02B
#undef USE_BARO_LPS22DF
#undef USE_BARO_SPI_LPS22DF


#undef USE_PWM
#undef USE_PWM_OUTPUT
#undef USE_DSHOT
#undef USE_DSHOT_BITBANG
#undef USE_DSHOT_TELEMETRY
#undef USE_DSHOT_TELEMETRY_STATS
#undef USE_ESC_SENSOR
#undef USE_ESC_SENSOR_TELEMETRY
#undef USE_ESC_SENSOR_INFO
#undef USE_ADC
#undef USE_TIMER
#undef USE_FLASH
#undef USE_FLASH_CHIP
#undef USE_FLASHFS
#undef USE_FLASH_TOOLS
#undef USE_FLASH_M25P16
#undef USE_FLASH_W25N01G
#undef USE_FLASH_W25N02K
#undef USE_FLASH_W25N
#undef USE_FLASH_W25M
#undef USE_FLASH_W25M512
#undef USE_FLASH_W25M02G
#undef USE_FLASH_W25Q128FV
#undef USE_FLASH_PY25Q128HA
#undef USE_FLASH_SPI
#undef USE_TRANSPONDER
#undef USE_SDCARD
#undef USE_LED_STRIP
#undef USE_SOFTSERIAL
#undef USE_VCP
#undef USE_ESCSERIAL
#undef USE_I2C
#undef USE_USB_DETECT
#undef USE_BEEPER
#undef USE_EXTI
#undef USE_TIMER_UP_CONFIG
#undef USE_RX_SPI
#undef USE_RX_CC2500
#undef USE_BARO
#undef USE_I2C_GYRO
#undef USE_SPI_GYRO
#undef USE_GYRO
#undef USE_ACC
#undef USE_MAG
#undef USE_MAX7456
#undef USE_VTX_RTC6705
#undef USE_VTX_RTC6705_SOFTSPI
// #undef USE_CLI
#undef USE_CAMERA_CONTROL
#undef USE_RX_PWM
#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERVO

#define USE_ADC
// #define CONFIG_IN_RAM
#define CONFIG_IN_FLASH
#define FLASH_PAGE_SIZE ((uint32_t)0x1000) // 4K sectors
#define USE_MOTOR
#define USE_EXTI
#define USE_TIMER
#define USE_TIMER_UP_CONFIG
#define USE_DMA
#define USE_PWM_OUTPUT
// #define USE_DSHOT
#define USE_DSHOT_DMAR

#define USE_USB_DETECT
#define USE_USBHS1
#define USE_VCP
#define USE_USB_MSC
#ifdef USE_USB_MSC
#define USB_MSC_BUTTON_PIN  PA0
#define MSC_BUTTON_IPU      false
#endif 

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_EARLY

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_5
#define USE_SPI_DEVICE_6

#define USE_I2C
#define USE_I2C_DEVICE_1    
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_ACC
#define USE_GYRO
#define USE_SPI_GYRO
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_ICM42688P

#define USE_PID_DENOM_CHECK
#define USE_PID_DENOM_OVERCLOCK_LEVEL 2


#define EEPROM_SIZE                     8192

#ifndef X32M7B
#define X32M7B
#endif

#define DEFAULT_BLACKBOX_DEVICE         BLACKBOX_DEVICE_SDCARD
#define UNIFIED_SERIAL_PORT_COUNT       1

#define USB_DM_PIN                      PA11
#define USB_DP_PIN                      PA12

#define GYRO_1_SPI_INSTANCE             SPI1
#define GYRO_1_CS_PIN                   PA4
#define GYRO_1_EXTI_PIN                 PE10
#define GYRO_1_ALIGN                    CW0_DEG

// #define FLASH_SPI_INSTANCE              SPI1
// #define FLASH_CS_PIN                    PB1

#define USE_SPI_DMA_ENABLE_LATE

#define SPI1_SCK_PIN                    PA5
#define SPI1_SDI_PIN                    PA6
#define SPI1_SDO_PIN                    PA7
#define SPI1_TX_DMA_OPT                 22
#define SPI1_RX_DMA_OPT                 23

#define USE_ADC_INTERNAL
#define ADC_INTERNAL_VBAT4_ENABLED      1

#define ADC_VBAT_PIN                    PC0
#define ADC_RSSI_PIN                    PC5
#define ADC_CURR_PIN                    PJ0
#define ADC_EXTERNAL1_PIN               PC4

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE  110
#define DEFAULT_CURRENT_METER_SCALE  800
 
#define ADC1_DMA_OPT                     10
#define ADC2_DMA_OPT                      9
#define ADC3_DMA_OPT                      8

#define MOTOR1_PIN                      PB6
#define MOTOR2_PIN                      PB7
#define MOTOR3_PIN                      PB8
#define MOTOR4_PIN                      PB9

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP(0, PB6, 1, 0) \
    TIMER_PIN_MAP(1, PB7, 1, 1) \
    TIMER_PIN_MAP(2, PB8, 1, 2) \
    TIMER_PIN_MAP(3, PB9, 1, 3) \
    TIMER_PIN_MAP(4, PC6, 1, 4) \
    TIMER_PIN_MAP(5, PC7, 1, 5) \
    TIMER_PIN_MAP(6, PC8, 1, 6) \
    TIMER_PIN_MAP(7, PC9, 1, 7)

#define TIMUP5_DMA_OPT                  8
#define TIMUP6_DMA_OPT                  9

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff
#define TARGET_IO_PORTH 0xffff
#define TARGET_IO_PORTI 0xffff
#define TARGET_IO_PORTJ 0xffff
#define TARGET_IO_PORTK 0xffff


#define USE_SDCARD
#define USE_BLACKBOX
#define USE_BLACKBOX_LOGGING_TO_SDCARD

#ifdef USE_SDCARD
#define USE_SDCARD_SDIO
#define PLATFORM_TRAIT_SDIO_INIT 1
// #define CONFIG_IN_SDCARD
#endif

#define SDIO_CK_PIN          PC12
#define SDIO_CMD_PIN         PD2
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11

#define SDIO_DEVICE                  SDIODEV_1
#define SDIO_USE_4BIT                1

#define USE_UART1
#define USE_UART10

#define USE_UART
#define USE_GPS
#ifdef USE_GPS
#define DEFAULT_FEATURES FEATURE_GPS
#define GPS_UART SERIAL_PORT_USART1
#endif

#define USE_RX_SX1280
#define SERIALRX_UART                SERIAL_PORT_UART10
#define DEFAULT_RX_FEATURE           FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER            SERIALRX_CRSF
#define UART10_TX_DMA_OPT                18

#define UART1_TX_PIN                    PA9
#define UART1_RX_PIN                    PA10
#define UART1_TX_DMA_OPT                20
#define UART1_RX_DMA_OPT                21
#define UART3_RX_PIN         PC10
#define UART3_TX_PIN         PC11
#define UART4_RX_PIN         PD0
#define UART4_TX_PIN         PD1
#define UART10_RX_PIN         PB12
#define UART10_TX_PIN         PB13
#define UART6_RX_PIN         PC7
#define UART6_TX_PIN         PC6
#define UART7_RX_PIN         PE7
#define UART7_TX_PIN         PE8
#define UART8_RX_PIN         PE0
#define UART8_TX_PIN         PE1
#define UART9_RX_PIN         PB8
#define UART9_TX_PIN         PB9


#define USE_BARO
#define USE_BARO_DPS310
#define BARO_I2C_INSTANCE            I2CDEV_3

#define USE_MAG
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE            I2CDEV_2

#define I2C1_SCL_PIN            PG5
#define I2C1_SDA_PIN            PG4
#define USE_I2C_PULLUP
#define I2C1_PULLUP             true
#define I2C1_CLOCKSPEED         400

#define I2C2_SCL_PIN            PB10
#define I2C2_SDA_PIN            PB11
#define I2C2_PULLUP             true
#define I2C2_CLOCKSPEED         400

#define I2C3_SCL_PIN            PJ14
#define I2C3_SDA_PIN            PJ13
#define I2C3_PULLUP             true
#define I2C3_CLOCKSPEED         400

#define USE_BEEPER
#define BEEPER_PIN              PA15
#define BEEPER_INVERTED