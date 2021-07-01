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

#define TARGET_BOARD_IDENTIFIER "SP7E"
#define USBD_PRODUCT_STRING "SPRacingH7EF"

#define USE_TARGET_CONFIG

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND

#define LED0_PIN                PE5
#define LED1_PIN                PE6

#define USE_BEEPER
#define BEEPER_PIN              PD11
#define BEEPER_INVERTED

// Force two buttons to look at the single button so config reset on button works
#define USE_BUTTONS
#define BUTTON_A_PIN            PD10
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PD10
#define BUTTON_B_PIN_INVERTED // Active high

// FC has 2 flash chips, one for logging on SPI6, one for code/data storage on OCTOSPIM_P1 (Memory mapped mode)
// Config is to be stored on the flash chip used for code/data storage, this requires that the
// config load/save routines must run from RAM and a method to enable/disable memory mapped mode is needed.

// There is no support for using OCTOSPI *and* SPI for 2 flash chips.

//#define USE_FLASH_MEMORY_MAPPED
//#define USE_OCTOSPI
#if defined(USE_OCTOSPI)
#define USE_OCTOSPI_DEVICE_1

#if !defined(USE_FLASH_MEMORY_MAPPED)
// Bootloader will have configured the OCTOSPI and OCTOSPIM peripherals and pins, firmware needs no-knowledge of the configuration.
#define OCTOSPIM_P1_SCK_PIN PB2
#define OCTOSPIM_P1_CS_PIN PB10

// Not using IO0:3
#define OCTOSPIM_P1_IO0_PIN NONE
#define OCTOSPIM_P1_IO1_PIN NONE
#define OCTOSPIM_P1_IO2_PIN NONE
#define OCTOSPIM_P1_IO3_PIN NONE

// Using IO4:7
#define OCTOSPIM_P1_IO4_PIN PE7
#define OCTOSPIM_P1_IO5_PIN PE8
#define OCTOSPIM_P1_IO6_PIN PE9
#define OCTOSPIM_P1_IO7_PIN PE10

#define OCTOSPIM_P1_MODE OCTOSPIM_P1_MODE_IO47_ONLY
#define OCTOSPIM_P1_CS_FLAGS (OCTOSPIM_P1_CS_HARDWARE)
#endif
#endif

#define USE_FLASH_CHIP
//#define CONFIG_IN_MEMORY_MAPPED_FLASH
#define CONFIG_IN_EXTERNAL_FLASH // Note: Since this FC has TWO flash chips, it's possible to use the non-memory-mapped flash to store the config instead.
//#define CONFIG_IN_SDCARD
//#define CONFIG_IN_RAM
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH) && !defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
#error "Config storage location not defined"
#endif

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PB7 // NC
#define UART1_TX_PIN            PB6 // NC

#define USE_UART2
#define UART2_RX_PIN            PD6 // J8:7
#define UART2_TX_PIN            PD5 // J8:8
#define UART2_RTS_PIN           PD4 // J8:9
#define UART2_CTS_PIN           PD3 // J8:10

#define USE_UART3
#define UART3_RX_PIN            PD9 // NC
#define UART3_TX_PIN            PD8 // Jumper to select between J14:3/4

#define USE_UART4
#define UART4_RX_PIN            PD0 // J5:3 - FDCAN_RX
#define UART4_TX_PIN            PD1 // J5:4 - FDCAN_TX

#define USE_UART5
#define UART5_RX_PIN            PD2   // J6:5, J9:5 ESC TLM
#define UART5_TX_PIN            NONE

#define USE_UART8
#define UART8_RX_PIN            PE0 // NC
#define UART8_TX_PIN            PE1 // NC

#define USE_UART9               // Main receiver connection, on connector and though-hole pads.
#define UART9_RX_PIN            PD14 // J11:3, J8:3 
#define UART9_TX_PIN            PD15 // J11:4, J8:4

#define USE_UART10
#define UART10_RX_PIN           PE2 // NC
#define UART10_TX_PIN           PE3 // NC

#define USE_VCP
#define USE_USB_ID

#define SERIAL_PORT_COUNT       9

#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
#define SPI3_NSS_PIN            PA15

#define USE_SPI_DEVICE_6
#define SPI6_SCK_PIN            PB3
#define SPI6_MISO_PIN           PB4
#define SPI6_MOSI_PIN           PB5
#define SPI6_NSS_PIN            NONE

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6 // NC
#define I2C1_SDA                PB7 // NC

#define USE_I2C_DEVICE_4
#define I2C4_SCL                PD12 // J8:5
#define I2C4_SDA                PD13 // J8:6

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE        (I2CDEV_4)

#define USE_GYRO
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#define USE_MULTI_GYRO
#undef USE_GYRO_REGISTER_DUMP

#define USE_EXTI
#define USE_GYRO_EXTI

#define GYRO_1_EXTI_PIN         PC6 // TIM8 CH1
#define GYRO_1_FSYNC_PIN        PC7 // TIM8 CH2
#define GYRO_2_EXTI_PIN         PC8 // TIM8 CH3
#define GYRO_2_FSYNC_PIN        PC9 // TIM8 CH4

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI3

#define GYRO_2_CS_PIN           SPI2_NSS_PIN
#define GYRO_2_SPI_INSTANCE     SPI2

#define USE_ACC
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P

#define GYRO_1_ALIGN            CW0_DEG
#define GYRO_2_ALIGN            CW180_DEG

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH

#define USE_FLASHFS
#define USE_FLASH_TOOLS

// For the SPI flash chip for logging
#define USE_FLASH_W25Q128
#define USE_FLASH_M25P16

#define FLASH_CS_PIN            PD7 // *NOT* SPI6_NSS, SOFTWARE CS ONLY
#define FLASH_SPI_INSTANCE      SPI6

// For the Octo-Spi connected
#ifdef USE_OCTOSPI
#define USE_FLASH_W25Q128FV
#define FLASH_OCTOSPI_INSTANCE  OCTOSPI1
#endif


#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#ifdef USE_DMA_SPEC
//#define UART1_TX_DMA_OPT        0
//#define UART2_TX_DMA_OPT        1
//#define UART3_TX_DMA_OPT        2
//#define UART4_TX_DMA_OPT        3
//#define UART5_TX_DMA_OPT        4
//#define UART6_TX_DMA_OPT        5
//#define UART7_TX_DMA_OPT        6
//#define UART8_TX_DMA_OPT        7
//#define ADC1_DMA_OPT 8 // ADC1 reserved for VIDEO
//#define ADC2_DMA_OPT 9 // ADC2 not used
#define ADC3_DMA_OPT 10
#else
//#define ADC1_DMA_STREAM DMA2_Stream0 // ADC1 reserved for VIDEO
//#define ADC2_DMA_STREAM DMA2_Stream1 // ADC2 not used
#define ADC3_DMA_STREAM DMA2_Stream2
#endif

#define USE_ADC

#define ADC_INSTANCE                  ADC3 // Use ADC3 by default, for as many pins as possible.

#define ADC1_INSTANCE                 ADC1 // ADC1 reserved for VIDEO
//#define ADC2_INSTANCE               ADC2 // ADC2 not used
#define ADC3_INSTANCE                 ADC3 // ADC3 for monitoring, core temp and vrefint

// 2 Current meter ADC inputs, one on each 4in1ESC connector. NO RSSI input due to on-board SX1280 RF chip.
#define CURRENT_METER_1_ADC_PIN       PC0 // ADC3_INP10
#define CURRENT_METER_1_ADC_INSTANCE  ADC3
#define CURRENT_METER_2_ADC_PIN       PC1 // ADC3_INP10
#define CURRENT_METER_2_ADC_INSTANCE  ADC3
#define RSSI_ADC_PIN                  PC2 // ADC3_INP0 - NOT CONNECTED
#define RSSI_ADC_INSTANCE             ADC3
#define VBAT_ADC_PIN                  PC3 // ADC3_INP1
#define VBAT_ADC_INSTANCE             ADC3

// ADC mapping from actual intended use to BF inputs
//#define RSSI_ADC_PIN                RSSI_ADC_PIN
//#define RSSI_ADC_INSTANCE           RSSI_ADC_INSTANCE
#define CURRENT_METER_ADC_PIN         CURRENT_METER_1_ADC_PIN
#define CURRENT_METER_ADC_INSTANCE    ADC3
//#define VBAT_ADC_PIN                VBAT_ADC_PIN
//#define VBAT_ADC_INSTANCE           VBAT_ADC_INSTANCE
#define EXTERNAL1_ADC_PIN             CURRENT_METER_2_ADC_PIN
#define EXTERNAL1_ADC_INSTANCE        CURRENT_METER_2_ADC_INSTANCE

#define VIDEO_IN_ADC_PIN              PC5 // ADC1_INP4 - Reserved for video
#define VIDEO_OUT_ADC_PIN             PC4 // ADC1_INP8 - Reserved for video

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_LED_STRIP)

#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE12
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE13
#define SPRACING_PIXEL_OSD_MASK_ENABLE_PIN              PE14
#define SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN      PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 PA8  // TIM1_CH1

#define SPRACING_PIXEL_OSD_WHITE_SOURCE_PIN             PA4  // DAC1_OUT1
#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5

#define VTX_ENABLE_PIN          PB11

#define USE_PINIO
#define PINIO1_PIN              VTX_ENABLE_PIN
#define USE_PINIOBOX

// Disable OCTOSPI pins PB2/CLK, PB10/NCS
// PE7/IO4, PE8/IO5, PE9/IO6, PE10/IO7

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)|BIT(10)))
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE (0xffff & ~(BIT(7)|BIT(8)|BIT(9)|BIT(10)))
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff
#define TARGET_IO_PORTH 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 16

#define USED_TIMERS  ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17) )
