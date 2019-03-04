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

typedef enum {
    OWNER_FREE = 0,
    OWNER_PWMINPUT,
    OWNER_PPMINPUT,
    OWNER_MOTOR,
    OWNER_SERVO,
    OWNER_LED,
    OWNER_ADC,
    OWNER_ADC_BATT,
    OWNER_ADC_CURR,
    OWNER_ADC_EXT,
    OWNER_ADC_RSSI,
    OWNER_SERIAL_TX,
    OWNER_SERIAL_RX,
    OWNER_PINDEBUG,
    OWNER_TIMER,
    OWNER_SONAR_TRIGGER,
    OWNER_SONAR_ECHO,
    OWNER_SYSTEM,
    OWNER_SPI_SCK,
    OWNER_SPI_MISO,
    OWNER_SPI_MOSI,
    OWNER_I2C_SCL,
    OWNER_I2C_SDA,
    OWNER_SDCARD,
    OWNER_SDCARD_CS,
    OWNER_SDCARD_DETECT,
    OWNER_FLASH_CS,
    OWNER_BARO_CS,
    OWNER_GYRO_CS,
    OWNER_OSD_CS,
    OWNER_RX_SPI_CS,
    OWNER_SPI_CS,
    OWNER_GYRO_EXTI,
    OWNER_BARO_EXTI,
    OWNER_COMPASS_EXTI,
    OWNER_USB,
    OWNER_USB_DETECT,
    OWNER_BEEPER,
    OWNER_OSD,
    OWNER_RX_BIND,
    OWNER_INVERTER,
    OWNER_LED_STRIP,
    OWNER_TRANSPONDER,
    OWNER_VTX,
    OWNER_COMPASS_CS,
    OWNER_RX_BIND_PLUG,
    OWNER_ESCSERIAL,
    OWNER_CAMERA_CONTROL,
    OWNER_TIMUP,
    OWNER_RANGEFINDER,
    OWNER_RX_SPI,
    OWNER_PINIO,
    OWNER_USB_MSC_PIN,
    OWNER_MCO,
    OWNER_RX_SPI_BIND,
    OWNER_RX_SPI_LED,
    OWNER_PREINIT,
    OWNER_RX_SPI_EXTI,
    OWNER_RX_SPI_CC2500_TX_EN,
    OWNER_RX_SPI_CC2500_LNA_EN,
    OWNER_RX_SPI_CC2500_ANT_SEL,
    OWNER_TOTAL_COUNT
} resourceOwner_e;

extern const char * const ownerNames[OWNER_TOTAL_COUNT];

#define RESOURCE_INDEX(x) (x + 1)
#define RESOURCE_SOFT_OFFSET    10
