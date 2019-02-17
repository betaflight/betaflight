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

#if defined(PIRXF4)

#define TARGET_BOARD_IDENTIFIER "PIRX"
#define USBD_PRODUCT_STRING     "Pirx F4"

#else

#define TARGET_BOARD_IDENTIFIER "RRF4"
#define USBD_PRODUCT_STRING     "Worm FC"

#endif

//LEDs
#if defined(PIRXF4)
#define LED0_PIN                PC13
#define LED1_PIN                PC14
#else
#define LED0_PIN                PA15
#define LED1_PIN                PC14
#endif

#define ENABLE_DSHOT_DMAR       true

//define camera control
#if defined(PIRXF4)
#define USE_CAMERA_CONTROL
#define CAMERA_CONTROL_PIN PA4
#endif

//BEEPER
#define USE_BEEPER
#if defined(PIRXF4)
#define BEEPER_PIN              PA15
#else
#define BEEPER_PIN              PB14
#endif
#define BEEPER_INVERTED

// MPU6500 interrupt
#define USE_EXTI
#if defined(PIRXF4)
#define GYRO_1_EXTI_PIN         PC5
#else
#define GYRO_1_EXTI_PIN         PC4
#endif
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

#if defined(PIRXF4)
#define GYRO_1_CS_PIN          PC4
#else
#define GYRO_1_CS_PIN          PA4
#endif
#define GYRO_1_SPI_INSTANCE    SPI1

// ACC section -- start
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#if defined(PIRXF4)
#define ACC_1_ALIGN       CW0_DEG
#else
#define ACC_1_ALIGN       CW180_DEG_FLIP
#endif
// ACC section -- end

// GYRO section -- start
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#if defined(PIRXF4)
#define GYRO_1_ALIGN      CW0_DEG
#else
#define GYRO_1_ALIGN      CW180_DEG_FLIP
#endif
// GYRO section -- end

//BARO
#if !defined(PIRXF4)
#define USE_BARO
#define USE_BARO_SPI_LPS
#define BARO_SPI_INSTANCE       SPI3
#define BARO_CS_PIN             PB8
#endif

//UARTs
#if defined(PIRXF4)
#define INVERTER_PIN_UART6      PA8
#define INVERTER_PIN_UART3      PB1
#else
#define INVERTER_PIN_UART6      PB13
#define INVERTER_PIN_UART3      PB12
#endif

#define USE_VCP
#define USB_DETECT_PIN          PA9
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#if defined(PIRXF4)
#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2
#endif

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#if defined(PIRXF4)
#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART2, USART3, USART4, USART6
#else
#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART3, USART4, USART6
#endif

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB9

//SPI
#define USE_SPI

#define USE_SPI_DEVICE_1

#if defined(PIRXF4)
#define USE_SPI_DEVICE_2
#else
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#endif

//OSD
#define USE_MAX7456
#if defined(PIRXF4)
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#else
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PC0
#endif
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#if defined(PIRXF4)
#define VBAT_ADC_PIN            PC2
#define RSSI_ADC_PIN            PC1
#define CURRENT_METER_ADC_PIN   PC3
#else
#define VBAT_ADC_PIN            PC1
#define CURRENT_METER_ADC_PIN   PC2
#endif

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

//SD CARD
#define USE_SDCARD
#define USE_SDCARD_SDIO

#define SDCARD_SDIO_DMA_OPT     0  // DMA 2 Stream 3 Channel 4
#define SDCARD_SPI_CS_PIN NONE //This is not used on SDIO, has to be kept for now to keep compiler happy
#if defined(PIRXF4)
#define SDCARD_DETECT_PIN PC15
#else
#define SDCARD_DETECT_PIN PB15
#endif

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
