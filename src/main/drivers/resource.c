/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation,either version 3 of the License,or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not,see <http://www.gnu.org/licenses/>.
 */

#include "resource.h"

const char * const ownerNames[OWNER_TOTAL_COUNT] = {
    "FREE",
    "PWM",
    "PPM",
    "MOTOR",
    "SERVO",
    "LED",
    "ADC",
    "ADC_BATT",
    "ADC_CURR",
    "ADC_EXT",
    "ADC_RSSI",
    "SERIAL_TX",
    "SERIAL_RX",
    "DEBUG",
    "TIMER",
    "SONAR_TRIGGER",
    "SONAR_ECHO",
    "SYSTEM",
    "SPI_SCK",
    "SPI_MISO",
    "SPI_MOSI",
    "I2C_SCL",
    "I2C_SDA",
    "SDCARD_CS",
    "FLASH_CS",
    "BARO_CS",
    "MPU_CS",
    "OSD_CS",
    "RX_SPI_CS",
    "SPI_CS",
    "MPU_EXTI",
    "BARO_EXTI",
    "USB",
    "USB_DETECT",
    "BEEPER",
    "OSD",
    "SDCARD_DETECT",
    "RX_BIND",
    "INVERTER",
    "LED_STRIP",
    "TRANSPONDER",
    "VTX",
};

