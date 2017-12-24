/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <platform.h>

#ifdef USE_SPI

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "bus_spi.h"

// Pin defaults for backward compatibility
#ifndef SPI1_SCK_PIN
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#ifndef SPI2_SCK_PIN
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

typedef struct spiDefaultConfig_s {
    SPIDevice device;
    ioTag_t sck;
    ioTag_t miso;
    ioTag_t mosi;
} spiDefaultConfig_t;

const spiDefaultConfig_t spiDefaultConfig[] = {
#ifdef USE_SPI_DEVICE_1
    { SPIDEV_1, IO_TAG(SPI1_SCK_PIN), IO_TAG(SPI1_MISO_PIN), IO_TAG(SPI1_MOSI_PIN) },
#endif
#ifdef USE_SPI_DEVICE_2
    { SPIDEV_2, IO_TAG(SPI2_SCK_PIN), IO_TAG(SPI2_MISO_PIN), IO_TAG(SPI2_MOSI_PIN) },
#endif
#ifdef USE_SPI_DEVICE_3
    { SPIDEV_3, IO_TAG(SPI3_SCK_PIN), IO_TAG(SPI3_MISO_PIN), IO_TAG(SPI3_MOSI_PIN) },
#endif
#ifdef USE_SPI_DEVICE_4
    { SPIDEV_4, IO_TAG(SPI4_SCK_PIN), IO_TAG(SPI4_MISO_PIN), IO_TAG(SPI4_MOSI_PIN) },
#endif
};

PG_REGISTER_WITH_RESET_FN(spiPinConfig_t, spiPinConfig, PG_SPI_PIN_CONFIG, 0);

void pgResetFn_spiPinConfig(spiPinConfig_t *spiPinConfig)
{
    for (size_t i = 0 ; i < ARRAYLEN(spiDefaultConfig) ; i++) {
        const spiDefaultConfig_t *defconf = &spiDefaultConfig[i];
        spiPinConfig->ioTagSck[defconf->device] = defconf->sck;
        spiPinConfig->ioTagMiso[defconf->device] = defconf->miso;
        spiPinConfig->ioTagMosi[defconf->device] = defconf->mosi;
    }
}
#endif
