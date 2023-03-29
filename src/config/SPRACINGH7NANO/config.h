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

   The auto generation is transitional only, please remove this comment once the file is edited.
*/

#define FC_TARGET_MCU     STM32H750

#define BOARD_NAME        SPRACINGH7NANO
#define MANUFACTURER_ID   SPRO

#define TARGET_BOARD_IDENTIFIER "SP7N"
#define USBD_PRODUCT_STRING "SPRacingH7NANO"

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
#define QUADSPI1_BK1_CS_PIN NONE
#define QUADSPI1_BK2_IO0_PIN PE7
#define QUADSPI1_BK2_IO1_PIN PE8
#define QUADSPI1_BK2_IO2_PIN PE9
#define QUADSPI1_BK2_IO3_PIN PE10
#define QUADSPI1_BK2_CS_PIN PB10
#define QUADSPI1_MODE QUADSPI_MODE_BK2_ONLY
#define QUADSPI1_CS_FLAGS (QUADSPI_BK1_CS_NONE | QUADSPI_BK2_CS_SOFTWARE | QUADSPI_CS_MODE_SEPARATE)

#define FLASH_QUADSPI_INSTANCE    QUADSPI

#define CONFIG_IN_EXTERNAL_FLASH

// SD card not present on hardware, but pins are reserved.
//#define USE_SDCARD
#ifdef USE_SDCARD
#define SDCARD_DETECT_PIN PD10
#define SDCARD_DETECT_INVERTED
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#else
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#endif

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
#define I2C1_SCL_PIN            PB8
#define I2C1_SCL                PB8 // alternate name to compile on 4.4
#define I2C1_SDA_PIN            PB9
#define I2C1_SDA                PB9 // alternate name to compile on 4.4
#define I2C_DEVICE (I2CDEV_1)

#define USE_I2C_DEVICE_4        // Shared with motor outputs 5/6
#define I2C4_SCL_PIN            PB6
#define I2C4_SCL                PB6 // alternate name to compile on 4.4
#define I2C4_SDA_PIN            PB7
#define I2C4_SDA                PB7 // alternate name to compile on 4.4

#define ENSURE_MPU_DATA_READY_IS_LOW

#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_BARO
#define USE_BARO_BMP388
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_CAMERA_CONTROL
#define USE_MAX7456
