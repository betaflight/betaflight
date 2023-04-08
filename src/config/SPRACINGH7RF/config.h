/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define FC_TARGET_MCU           STM32H730

#define BOARD_NAME              SPRACINGH7RF
#define MANUFACTURER_ID         SPRO

#define TARGET_BOARD_IDENTIFIER "SP7R"
#define USBD_PRODUCT_STRING     "SPRacingH7RF"

#define FC_VMA_ADDRESS          0x90100000

#define EEPROM_SIZE             8192

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND

#define USE_BUTTONS
#define BUTTON_A_PIN            PE4
#define BUTTON_A_PIN_INVERTED
#define BUTTON_B_PIN            PE4
#define BUTTON_B_PIN_INVERTED

#define USE_OCTOSPI
#define USE_OCTOSPI_DEVICE_1

#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_SDI_PIN            PB14
#define SPI2_SDO_PIN            PB15
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_6
#define SPI6_SCK_PIN            PB3
#define SPI6_SDI_PIN            PB4
#define SPI6_SDO_PIN            PB5
#define SPI6_NSS_PIN            PA15

#define SX1280_BUSY_PIN         PC7
#define SX1280_DIO1_PIN         PC6
#define SX1280_DIO2_PIN         PD4
#define SX1280_DIO3_PIN         NONE

#define USE_FLASH_MEMORY_MAPPED
#define FLASH_OCTOSPI_INSTANCE  OCTOSPI1
#define CONFIG_IN_MEMORY_MAPPED_FLASH
#define USE_FIRMWARE_PARTITION

#define SDCARD_DETECT_PIN PC13
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           1
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         (0xffff & ~(BIT(2)|BIT(6)))
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (0xffff & ~(BIT(11)|BIT(12)|BIT(13)))
#define TARGET_IO_PORTE         (0xffff & ~(BIT(2)|BIT(7)|BIT(8)|BIT(9)|BIT(10)))
#define TARGET_IO_PORTF         0xffff
#define TARGET_IO_PORTG         0xffff
#define TARGET_IO_PORTH         0xffff

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN            PB8
#define I2C1_SDA_PIN            PB9

#define USE_I2C_DEVICE_2
#define I2C2_SCL_PIN            PB10
#define I2C2_SDA_PIN            PB11

#define MAG_I2C_INSTANCE        (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE       (I2CDEV_2)

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define SX1280_BUSY_PIN         PC7
#define SX1280_DIO1_PIN         PC6
#define SX1280_DIO2_PIN         PD4
#define SX1280_DIO3_PIN         NONE

#define SX1280_NRESET_PIN       PD10

#define USE_RX_SPI
#define USE_RX_EXPRESSLRS
#define USE_RX_SX1280
#define RX_SPI_INSTANCE               SPI2
#define RX_SPI_EXTI_PIN               SX1280_DIO1_PIN
#define RX_EXPRESSLRS_SPI_RESET_PIN   SX1280_NRESET_PIN
#define RX_EXPRESSLRS_SPI_BUSY_PIN    SX1280_BUSY_PIN
#define RX_EXPRESSLRS_TIMER_INSTANCE  TIM6

#define DEFAULT_RX_FEATURE            FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL       RX_SPI_EXPRESSLRS
#define RX_SPI_PROTOCOL               EXPRESSLRS

#define ADC3_DMA_OPT                  10
#define ADC_INSTANCE                  ADC3
#define ADC1_INSTANCE                 ADC1
#define ADC3_INSTANCE                 ADC3
#define CURRENT_METER_2_ADC_PIN       PC0
#define CURRENT_METER_2_ADC_INSTANCE  ADC3
#define CURRENT_METER_1_ADC_PIN       PC1
#define CURRENT_METER_1_ADC_INSTANCE  ADC3
#define ADC_EXTERNAL1_PIN             PC2
#define ADC_EXTERNAL1_INSTANCE        ADC3
#define VBAT_ADC_INSTANCE             ADC3
#define VIDEO_IN_ADC_PIN              PC5
#define VIDEO_OUT_ADC_PIN             PC4

#define VTX_ENABLE_PIN                PC15
#define PINIO1_PIN                    VTX_ENABLE_PIN

#define USE_ACC
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP388
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_FLASH_W25Q128FV
#define USE_SDCARD

#ifndef USE_OSD
#define USE_OSD
#endif

#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE12
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE13
#define SPRACING_PIXEL_OSD_MASK_ENABLE_PIN              PE14
#define SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN      PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 PA8

#define SPRACING_PIXEL_OSD_WHITE_SOURCE_PIN             PA4
#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5

#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1

#define BEEPER_PIN           PE4
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PA6
#define MOTOR4_PIN           PA7
#define MOTOR5_PIN           PA0
#define MOTOR6_PIN           PA1
#define MOTOR7_PIN           PA2
#define MOTOR8_PIN           PA3
#define LED_STRIP_PIN        PB7

#define UART1_TX_PIN         NONE
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PD1
#define UART5_TX_PIN         NONE
#define UART6_TX_PIN         NONE
#define UART7_TX_PIN         NONE
#define UART8_TX_PIN         PE1
#define UART9_TX_PIN         NONE
#define UART10_TX_PIN        NONE
#define UART1_RX_PIN         NONE
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PD0
#define UART5_RX_PIN         PB13
#define UART6_RX_PIN         NONE
#define UART7_RX_PIN         NONE
#define UART8_RX_PIN         PE0
#define UART9_RX_PIN         NONE
#define UART10_RX_PIN        NONE

#define I2C1_SCL_PIN         PB8
#define I2C2_SCL_PIN         PB10
#define I2C3_SCL_PIN         NONE
#define I2C4_SCL_PIN         NONE
#define I2C1_SDA_PIN         PB9
#define I2C2_SDA_PIN         PB11
#define I2C3_SDA_PIN         NONE
#define I2C4_SDA_PIN         NONE

#define LED0_PIN             PE5
#define LED1_PIN             PE6
#define LED2_PIN             NONE
#define SPEKTRUM_RX_BIND_PIN NONE
#define RX_BIND_PLUG_PIN     NONE

#define ADC_VBAT_PIN         PC3
#define ADC_RSSI_PIN         PC0
#define ADC_CURR_PIN         PC1
#define ADC_EXTERNAL1_PIN    PC2

#define SDCARD_DETECT_PIN    PC13
#define SDIO_CK_PIN          PC12
#define SDIO_CMD_PIN         PD2
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11

//#define PINIO1_PIN           PC15
#define RX_SPI_CS_PIN        PB12
//#define RX_SPI_EXTI_PIN      PC6
#define RX_SPI_BIND_PIN      NONE
#define RX_SPI_LED_PIN       NONE
#define RX_SPI_EXPRESSLRS_RESET_PIN PD10
#define RX_SPI_EXPRESSLRS_BUSY_PIN PC7

#define GYRO_1_EXTI_PIN      PD15
#define GYRO_2_EXTI_PIN      NONE
#define GYRO_1_CS_PIN        PA15
#define GYRO_2_CS_PIN        NONE

#define ADC3_DMA_OPT       10

#define MAG_I2C_INSTANCE     (I2CDEV_1)
#define BARO_I2C_INSTANCE    (I2CDEV_2)

#define PINIO1_BOX 40

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI6
#define GYRO_1_ALIGN CW270_DEG_FLIP

// Prevent flash being used for logs as target requires EXST
#ifdef USE_FLASH
#undef USE_FLASH
#endif
