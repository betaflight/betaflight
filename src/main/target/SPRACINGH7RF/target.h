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

#define TARGET_BOARD_IDENTIFIER "SP7R"
#define USBD_PRODUCT_STRING "SPRacingH7RF"

#define USE_TARGET_CONFIG

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND

#define LED0_PIN                PE5
#define LED1_PIN                PE6

#define USE_BEEPER
#define BEEPER_PIN              PE4
#define BEEPER_INVERTED

// Force two buttons to look at the single button so config reset on button works
#define USE_BUTTONS
#define BUTTON_A_PIN            PC14
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PC14
#define BUTTON_B_PIN_INVERTED // Active high

// FC has an a OctoSPI flash chip, for code/data storage on OCTOSPIM_P1 (Memory mapped mode)
// Config is to be stored on the flash chip used for code/data storage, this requires that the
// config load/save routines must run from RAM and a method to enable/disable memory mapped mode is needed.

#define USE_OCTOSPI
#define USE_OCTOSPI_DEVICE_1

#define OCTOSPIM_P1_SCK_PIN PB2
#define OCTOSPIM_P1_CS_PIN PB6

// Using IO0:3
#define OCTOSPIM_P1_IO0_PIN PD11
#define OCTOSPIM_P1_IO1_PIN PD12
#define OCTOSPIM_P1_IO2_PIN PE2
#define OCTOSPIM_P1_IO3_PIN PD13

// Not using IO4:7
#define OCTOSPIM_P1_IO4_PIN NONE
#define OCTOSPIM_P1_IO5_PIN NONE
#define OCTOSPIM_P1_IO6_PIN NONE
#define OCTOSPIM_P1_IO7_PIN NONE

#define OCTOSPIM_P1_MODE OCTOSPIM_P1_MODE_IO03_ONLY
#define OCTOSPIM_P1_CS_FLAGS (OCTOSPIM_P1_CS_HARDWARE)

// TODO No support yet for config using a single flash device in memory mapped mode, using SDCARD until support is added.

//#define USE_FLASH_CHIP
//#define CONFIG_IN_EXTERNAL_FLASH
#define CONFIG_IN_SDCARD
//#define CONFIG_IN_RAM
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH)
#error "EEPROM storage location not defined"
#endif

#define USE_UART

//#define USE_UART1
#define UART1_RX_PIN            NONE
#define UART1_TX_PIN            NONE

#define USE_UART2
#define UART2_RX_PIN            PD6 // J8:8
#define UART2_TX_PIN            PD5 // J8:7

#define USE_UART3
#define UART3_RX_PIN            PD9 // J7:5
#define UART3_TX_PIN            PD8 // J7:4

#define USE_UART4
#define UART4_RX_PIN            PD0 // J8:12
#define UART4_TX_PIN            PD1 // J8:11

#define USE_UART5
// Note: PB13 by default is a UART 5 TX pin, this configuration requires use of PINSWAP in the UART initialisation code.
#define UART5_RX_PIN            PB13 // ESC TLM IN
#define UART5_TX_PIN            NONE

//#define USE_UART6
#define UART6_RX_PIN            NONE
#define UART6_TX_PIN            NONE

//#define USE_UART7
#define UART7_RX_PIN            NONE
#define UART7_TX_PIN            NONE

#define USE_UART8
#define UART8_RX_PIN            PE0 // J8:4
#define UART8_TX_PIN            PE1 // J8:3

//#define USE_UART9
#define UART9_RX_PIN            NONE
#define UART9_TX_PIN            NONE

//#define USE_UART10
#define UART10_RX_PIN           NONE
#define UART10_TX_PIN           NONE

#define USE_VCP
#define USE_USB_ID

#define SERIAL_PORT_COUNT       6

#define USE_SPI

// SX1280 on SPI2

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define SPI2_NSS_PIN            PB12

#define SX1280_BUSY_PIN         PC7 // TIM8_CH2
#define SX1280_DIO1_PIN         PC6 // TIM8_CH1
#define SX1280_DIO2_PIN         PD4 // EXTI4
#define SX1280_DIO3_PIN         NONE
// RevA has 0ohm jumper on R26.  Delete schematic connection and support if SX1280 doesn't need NRESET to free a pin.
// The idea is that NRESET should not be needed at all.  Production boards probably won't have jumper installed.
#define SX1280_NRESET_PIN       PD10

// ICM42605/ICM42688P/LSM6DSO/LSM6DSO32/BMI270/LSM6DSRX on SPI3

// SPI3 and SPI6 use the same pins however SPI6 supports independent clock configuration, thus SPI6 used to allow separation between SX1280 and Gyro SPI clocks.
#define USE_SPI_DEVICE_6
#define SPI6_SCK_PIN            PB3
#define SPI6_MISO_PIN           PB4
#define SPI6_MOSI_PIN           PB5
#define SPI6_NSS_PIN            PA15

#define USE_I2C
// I2C1 for external MAG connection on J8
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8 // J8:5
#define I2C1_SDA                PB9 // J8:6

// I2C2 has BMP388 internally connected, 10k pull-ups
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE        (I2CDEV_1)

#define USE_BARO
#define USE_BARO_BMP388
#define BARO_I2C_INSTANCE       (I2CDEV_2)

#define USE_GYRO
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#undef USE_GYRO_REGISTER_DUMP

#define USE_EXTI
#define USE_GYRO_EXTI

#define GYRO_1_EXTI_PIN         PD15 // TIM4_CH4
#define GYRO_1_FSYNC_PIN        PD14 // TIM4_CH3

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN           SPI6_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI6

#define USE_ACC
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P

#define GYRO_1_ALIGN            CW270_DEG_FLIP

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

#define USE_FLASHFS
#define USE_FLASH_TOOLS
#define USE_FLASH_W25Q128
#define USE_FLASH_M25P16

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

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
#define EXTERNAL1_ADC_PIN             PC2 // ADC3_INP0 - NOT CONNECTED
#define EXTERNAL1_ADC_INSTANCE        ADC3
#define VBAT_ADC_PIN                  PC3 // ADC3_INP1
#define VBAT_ADC_INSTANCE             ADC3

// ADC mapping from actual intended use to BF inputs
#define RSSI_ADC_PIN                  CURRENT_METER_2_ADC_PIN
#define RSSI_ADC_INSTANCE             CURRENT_METER_2_ADC_INSTANCE
#define CURRENT_METER_ADC_PIN         CURRENT_METER_1_ADC_PIN
#define CURRENT_METER_ADC_INSTANCE    CURRENT_METER_1_ADC_INSTANCE
//#define VBAT_ADC_PIN                VBAT_ADC_PIN
//#define VBAT_ADC_INSTANCE           VBAT_ADC_INSTANCE
//#define EXTERNAL1_ADC_PIN           EXTERNAL1_ADC_PIN
//#define EXTERNAL1_ADC_INSTANCE        EXTERNAL1_ADC_INSTANCE


#define VIDEO_IN_ADC_PIN        PC5 // ADC1_INP4 - Reserved for video
#define VIDEO_OUT_ADC_PIN       PC4 // ADC1_INP8 - Reserved for video

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_DETECT_PIN PC13
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           true
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_EXPRESSLRS

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_RSSI_ADC | FEATURE_LED_STRIP)

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

#define USE_RX_SPI
#define USE_RX_EXPRESSLRS
#define USE_RX_SX1280

#define RX_SPI_INSTANCE                                 SPI2
#define RX_NSS_PIN                                      SPI2_NSS_PIN
#define RX_SPI_EXTI_PIN                                 SX1280_DIO1_PIN // ?
//#define RX_SPI_BIND_PIN                                 ?
//#define RX_SPI_LED_PIN                                  ?

#define RX_EXPRESSLRS_SPI_RESET_PIN                     SX1280_NRESET_PIN
#define RX_EXPRESSLRS_SPI_BUSY_PIN                      SX1280_BUSY_PIN

// These are connected, DIO1 is on a timer with a spare Capture Compare channel.  DIO2 is on a free EXTI capable pin.
// These defines are not currently used anywhere in the code.
//#define RX_EXPRESSLRS_SPI_DIO1_PIN                      SX1280_DIO1_PIN
//#define RX_EXPRESSLRS_SPI_DIO2_PIN                      SX1280_DIO2_PIN

#define VTX_ENABLE_PIN          PC15

#define USE_PINIO
#define PINIO1_PIN              VTX_ENABLE_PIN
#define USE_PINIOBOX


// Disable OCTOSPI pins PB2/CLK, PB6/NCS, PD11/IO0, PD12/IO1, PD13/IO3, PE2/IO2
// PE7/IO4, PE8/IO5, PE9/IO6, PE10/IO7

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)|BIT(6)))
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (0xffff & ~(BIT(11)|BIT(12)|BIT(13)))
#define TARGET_IO_PORTE (0xffff & ~(BIT(2)|BIT(7)|BIT(8)|BIT(9)|BIT(10)))
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff
#define TARGET_IO_PORTH 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 15

// TIM1, TIM2, TIM15 reserved for OSD
#define USED_TIMERS  ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(15) | TIM_N(17))
