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

#define TARGET_BOARD_IDENTIFIER "FWF4"
#define USBD_PRODUCT_STRING "FLYWOOF405"

#define USE_TARGET_CONFIG

#define LED0_PIN                PC14/////////////

#define USE_BEEPER
#define BEEPER_PIN              PC13/////////////
#define BEEPER_INVERTED
#define INVERTER_PIN_UART3      PB15

//define camera control
#define CAMERA_CONTROL_PIN      PA9////////////


// ICM20689 interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PC5
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW



#define USE_GYRO
#define USE_ACC
//------ICM20689
#define ICM20689_CS_PIN          PC4 
#define ICM20689_SPI_INSTANCE    SPI1

#define USE_GYRO_SPI_ICM20689
#define GYRO_ICM20689_ALIGN      CW270_DEG

#define USE_ACC_SPI_ICM20689
#define ACC_ICM20689_ALIGN       CW270_DEG
//------MPU6000
#define MPU6000_CS_PIN           PC4 
#define MPU6000_SPI_INSTANCE     SPI1

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG

#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG

//Baro & MAG------------------------------- 
#define USE_I2C    
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6        // SCL pad
#define I2C1_SDA                PB9        // SDA pad
#define BARO_I2C_INSTANCE       I2C_DEVICE
#define MAG_I2C_INSTANCE        I2C_DEVICE

#define USE_MAG
#define USE_MAG_HMC5883                   //External, connect to I2C1
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG

#define USE_BARO
#define USE_BARO_MS5611                  //External, connect to I2C1
#define USE_BARO_BMP280                  //onboard

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PB14
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3

#define USE_VCP
#define USB_DETECT_PIN          PA8
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PB6 //SCL/UART1_TX/TIM4_CH1
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART4                // Uart4 can be used for GPS or  RunCam Split
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5               //Uart5 can be used for ESC sensor
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            NONE

#define USE_SOFTSERIAL1         
#define SERIAL_PORT_COUNT 7     //vcp, uart1, uart3, uart4, uart5, uart6, softSerial1


#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8  // (HARDARE=0,PPM)

#define USE_SPI

#define USE_SPI_DEVICE_1 //ICM20689-MPU6000
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3 //dataflash
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC1_DMA_STREAM 			DMA2_Stream0
#define VBAT_ADC_PIN                PC3
#define CURRENT_METER_ADC_PIN       PC2
#define RSSI_ADC_PIN                PC1

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD )
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD        (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(4)|  TIM_N(8))

