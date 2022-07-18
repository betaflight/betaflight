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
#if defined(FF_FORTINIF4_REV03)
#define TARGET_BOARD_IDENTIFIER "FOR3"
#define USBD_PRODUCT_STRING     "FortiniF4_Rev03"
#else
#define TARGET_BOARD_IDENTIFIER "FORT"
#define USBD_PRODUCT_STRING     "FortiniF4"
#endif
#define USE_TARGET_CONFIG
#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN_A                PC14
#define HW_PIN_B                PC8
/*--------------LED----------------*/
#define LED0_PIN                PB5
#if defined(FF_FORTINIF4_REV03)
#define LED1_PIN                PB4
#else
#define LED1_PIN                PB6
#endif
/*---------------------------------*/

/*------------BEEPER---------------*/
#define USE_BEEPER
#if defined(FF_FORTINIF4_REV03)
#define BEEPER_PIN              PB3
#else
#define BEEPER_PIN              PB4
#endif
#define BEEPER_INVERTED
/*---------------------------------*/

/*------------SENSORS--------------*/
// MPU interrupt
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_ICM20689
#define GYRO_1_ALIGN            CW180_DEG

#define GYRO_1_SPI_INSTANCE     SPI1

#if defined(FF_FORTINIF4_REV03)
#define GYRO_1_CS_PIN           PA4
#else
#define GYRO_1_CS_PIN           PA8
#endif

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_ICM20689

/*---------------------------------*/

#if !defined(FF_FORTINIF4_REV03)
/*------------FLASH----------------*/
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
/*---------------------------------*/
#endif

/*-------------OSD-----------------*/
#define USE_MAX7456
#if defined(FF_FORTINIF4_REV03)
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN
#else
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      SPI3_NSS_PIN
#endif
/*---------------------------------*/

/*-----------USB-UARTs-------------*/
#define USE_VCP
//#define USB_DETECT_PIN          NONE
//#define USE_USB_DETECT

#define USE_UART1
#if defined(FF_FORTINIF4_REV03)
#define UART1_RX_PIN            PB7
#else 
#define UART1_RX_PIN            PA10
#endif  
#define UART1_TX_PIN            PA9

#define USE_UART4
#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#if defined(FF_FORTINIF4_REV03)
#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10
#define INVERTER_PIN_UART3      PC15

#define SERIAL_PORT_COUNT       7
#else
#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#define INVERTER_PIN_UART3      PC15

#define SERIAL_PORT_COUNT       6
#endif
/*---------------------------------*/

/*-------------SPIs----------------*/
#define USE_SPI

#define USE_SPI_DEVICE_1
#if defined(FF_FORTINIF4_REV03)
#define SPI1_NSS_PIN            PA4
#else
#define SPI1_NSS_PIN            PA8
#endif
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#if defined(FF_FORTINIF4_REV03)
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#else
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
#endif
/*---------------------------------*/

#if defined(FF_FORTINIF4_REV03)
/*-------------I2C-----------------*/
#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_3
#define I2C_DEVICE              (I2CDEV_3)
#define I2C1_SCL                PA8
#define I2C1_SDA                PC9
/*---------------------------------*/
#endif

/*-------------ADCs----------------*/
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define VBAT_ADC_PIN                    PC2
#define CURRENT_METER_ADC_PIN           PC1
/*---------------------------------*/

/*-------------ESCs----------------*/
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB0  // (HARDARE=0)
/*---------------------------------*/

/*--------DEFAULT VALUES-----------*/
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
/*---------------------------------*/

/*--------------TIMERS-------------*/
#if defined(FF_FORTINIF4_REV03)
#define USABLE_TIMER_CHANNEL_COUNT  7
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
#else
#define USABLE_TIMER_CHANNEL_COUNT  6
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) )
#endif
/*---------------------------------*/
