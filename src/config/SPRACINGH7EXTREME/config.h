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

/*
   This file has been auto generated from unified-targets repo.

   The auto generation is transitional only, please ensure you update unified targets and not this file until the transition has complete.
*/

#define FC_TARGET_MCU     STM32H750

#define BOARD_NAME        SPRACINGH7EXTREME
#define MANUFACTURER_ID   SPRO

#define TARGET_BOARD_IDENTIFIER "SP7E"
#define USBD_PRODUCT_STRING "SPRacingH7EXTREME"

#define FC_VMA_ADDRESS    0x97CE0000

#define EEPROM_SIZE 8192

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND

#define USE_BUTTONS
#define BUTTON_A_PIN            PE4
#define BUTTON_A_PIN_INVERTED
#define BUTTON_B_PIN            PE4
#define BUTTON_B_PIN_INVERTED

#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1
#define QUADSPI1_SCK_PIN PB2
#define QUADSPI1_BK1_IO0_PIN PD11
#define QUADSPI1_BK1_IO1_PIN PD12
#define QUADSPI1_BK1_IO2_PIN PE2
#define QUADSPI1_BK1_IO3_PIN PD13
#define QUADSPI1_BK1_CS_PIN PB10
#define QUADSPI1_BK2_IO0_PIN PE7
#define QUADSPI1_BK2_IO1_PIN PE8
#define QUADSPI1_BK2_IO2_PIN PE9
#define QUADSPI1_BK2_IO3_PIN PE10
#define QUADSPI1_BK2_CS_PIN NONE
#define QUADSPI1_MODE QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS (QUADSPI_BK1_CS_HARDWARE | QUADSPI_BK2_CS_NONE | QUADSPI_CS_MODE_LINKED)

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define FLASH_QUADSPI_INSTANCE    QUADSPI

#define CONFIG_IN_EXTERNAL_FLASH

#define SDCARD_DETECT_PIN PD10
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           1
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11

#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PD6
#define SPI3_NSS_PIN            PA15

#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14
#define SPI4_NSS_PIN            PE11

#define USE_USB_ID

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN PB8
#define I2C1_SDA_PIN PB9
#define I2C_DEVICE (I2CDEV_1)

#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_PID_AUDIO

#define VTX_RTC6705_OPTIONAL

#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9

#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500
#define USE_BARO_BMP388
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_FLASH_W25N01G
#define USE_SDCARD
#define USE_CAMERA_CONTROL
#define USE_MAX7456

#define BEEPER_PIN           PD7
#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PB6
#define MOTOR6_PIN           PB7
#define MOTOR7_PIN           PC6
#define MOTOR8_PIN           PC7
#define RX_PPM_PIN           PB15
#define RX_PWM1_PIN          PB15
#define LED_STRIP_PIN        PA8

#define UART1_TX_PIN         PB14
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PD1
#define UART5_TX_PIN         PB13
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PNONE
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PB15
#define UART2_RX_PIN         PNONE
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PD0
#define UART5_RX_PIN         PB5
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PNONE
#define UART8_RX_PIN         PE0

#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define I2C2_SCL_PIN         PNONE
#define I2C2_SDA_PIN         PNONE
#define I2C3_SCL_PIN         PNONE
#define I2C3_SDA_PIN         PNONE
#define I2C4_SCL_PIN         PNONE
#define I2C4_SDA_PIN         PNONE

#define LED0_PIN             PE3
#define TRANSPONDER_PIN      PB11

#define CAMERA_CONTROL_PIN   PE5

#define ADC_VBAT_PIN         PC1
#define ADC_RSSI_PIN         PC4
#define ADC_CURR_PIN         PC0
#define ADC_EXTERNAL1_PIN    PC5

#define SDCARD_DETECT_PIN    PD10
#define SDIO_CK_PIN          PC12
#define SDIO_CMD_PIN         PD2
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11
#define OSD_CS_PIN           PE11

#define GYRO_1_EXTI_PIN      PD4
#define GYRO_2_EXTI_PIN      PE15
#define GYRO_1_CS_PIN        PA15
#define GYRO_2_CS_PIN        PB12
#define VTX_POWER_PIN        PB1
#define VTX_CS_PIN           PB0
#define VTX_DATA_PIN         PA6
#define VTX_CLK_PIN          PA7

#define MAG_I2C_INSTANCE I2C1

#define BARO_I2C_INSTANCE I2C1

#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE SPI3
#define GYRO_1_ALIGN CW180_DEG
#define USE_SPI_GYRO
#define GYRO_2_SPI_INSTANCE SPI2
#define GYRO_2_ALIGN CUSTOM
#define MAX7456_SPI_INSTANCE SPI4
