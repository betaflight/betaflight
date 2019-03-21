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

#if defined(CRAZYBEEF4FR)
#define TARGET_BOARD_IDENTIFIER "C4FR"
#define USBD_PRODUCT_STRING     "CrazyBee F4 FR"
#elif defined(CRAZYBEEF4FS)
#define TARGET_BOARD_IDENTIFIER "C4FS"
#define USBD_PRODUCT_STRING     "CrazyBee F4 FS"
#elif defined(CRAZYBEEF4DX)
#define TARGET_BOARD_IDENTIFIER "C4DX"
#define USBD_PRODUCT_STRING     "CrazyBee F4 DX"
#else
#define TARGET_BOARD_IDENTIFIER "M41R"
#define USBD_PRODUCT_STRING     "MATEKF411RX"
#endif

#define LED0_PIN                PC13

#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define USE_SPI

// *************** SPI1 Gyro & ACC **********************
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA1
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#if defined(CRAZYBEEF4FS) || defined(CRAZYBEEF4FR) || defined(CRAZYBEEF4DX)
#define GYRO_1_ALIGN            CW90_DEG
#else
#define GYRO_1_ALIGN            CW180_DEG
#endif
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#if defined(CRAZYBEEF4FS) || defined(CRAZYBEEF4FR) || defined(CRAZYBEEF4DX)
#define ACC_1_ALIGN            CW90_DEG
#else
#define ACC_1_ALIGN             CW180_DEG
#endif
// *************** SPI2 OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12

#if defined(CRAZYBEEF4DX)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SPEKTRUM2048
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER

#else
// *************** SPI3 CC2500 ***************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#define RX_NSS_PIN              PA15

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI3

#define BINDPLUG_PIN            PB2

#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI

#if defined(CRAZYBEEF4FS)
#define USE_RX_FLYSKY
#define RX_CHANNELS_AETR
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_A7105_FLYSKY_2A
#define FLYSKY_2A_CHANNEL_COUNT 14
#define RX_SPI_EXTI_PIN         PA14
#define USE_RX_FLYSKY_SPI_LED
#define RX_SPI_LED_PIN          PB9

#elif defined(CRAZYBEEF4FR)
#define RX_CC2500_SPI_DISABLE_CHIP_DETECTION
#define RX_SPI_EXTI_PIN             PC14
#define RX_SPI_LED_PIN              PB9
#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_SFHSS_SPI
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKY_X
#define USE_RX_FRSKY_SPI_TELEMETRY

#else // MATEKF411RX
#define RX_CC2500_SPI_DISABLE_CHIP_DETECTION
#define RX_SPI_EXTI_PIN             PC14
#define RX_SPI_LED_PIN              PB9
#define RX_SPI_LED_INVERTED

#define USE_RX_CC2500_SPI_PA_LNA
#define RX_CC2500_SPI_TX_EN_PIN      PA8
#define RX_CC2500_SPI_LNA_EN_PIN     PA13

#define USE_RX_CC2500_SPI_DIVERSITY
#define RX_CC2500_SPI_ANT_SEL_PIN    PA14

#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_SFHSS_SPI
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKY_X
#define USE_RX_FRSKY_SPI_TELEMETRY
#endif
#endif // CRAZYBEEF4DX

// *************** UART *****************************
#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT       4

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE         ADC1  // Default added
#define ADC1_DMA_OPT            0  // DMA 2 Stream 0 Channel 0 

#define VBAT_ADC_PIN            PB0
#define CURRENT_METER_ADC_PIN   PB1

#define USE_ESCSERIAL

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY )
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 179

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             ( TIM_N(1)|TIM_N(2)|TIM_N(4)|TIM_N(5)|TIM_N(9))
