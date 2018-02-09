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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SDCARD

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sdcard.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/dma.h"

PG_REGISTER_WITH_RESET_FN(sdcardConfig_t, sdcardConfig, PG_SDCARD_CONFIG, 0);

void pgResetFn_sdcardConfig(sdcardConfig_t *config)
{
    config->useDma = false;
#ifdef SDCARD_SPI_INSTANCE
    config->enabled = 1;
    config->device = spiDeviceByInstance(SDCARD_SPI_INSTANCE);
#else
    config->enabled = 0;
    config->device = 0;
#endif
#ifdef SDCARD_DETECT_PIN
    config->cardDetectTag = IO_TAG(SDCARD_DETECT_PIN);
#else
    config->cardDetectTag = IO_TAG_NONE;
#endif
#ifdef SDCARD_SPI_CS_PIN
    config->chipSelectTag = IO_TAG(SDCARD_SPI_CS_PIN);
#else
    config->chipSelectTag = IO_TAG_NONE;
#endif

#ifdef SDCARD_DETECT_INVERTED
    config->cardDetectInverted = 1;
#else
    config->cardDetectInverted = 0;
#endif

#if defined(SDCARD_DMA_STREAM_TX_FULL)
    config->dmaIdentifier = (uint8_t)dmaGetIdentifier(SDCARD_DMA_STREAM_TX_FULL);
#elif defined(SDCARD_DMA_CHANNEL_TX)
    config->dmaIdentifier = (uint8_t)dmaGetIdentifier(SDCARD_DMA_CHANNEL_TX);
#endif

#if (defined(STM32F4) || defined(STM32F7)) && defined(SDCARD_DMA_CHANNEL)
    config->dmaChannel = SDCARD_DMA_CHANNEL;
#endif
}
#endif
