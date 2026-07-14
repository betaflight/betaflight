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
 * DMA stub for original ESP32 (no GDMA).
 * The original ESP32 has per-peripheral DMA built into SPI/I2S,
 * not a centralized GDMA controller. This stub satisfies the
 * linker for code that references DMA functions when USE_DMA is
 * not defined.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifndef USE_DMA

#include "drivers/bus_spi.h"

bool spiUseDMA(const extDevice_t *dev)
{
    (void)dev;
    return false;
}

bool spiUseSDO_DMA(const extDevice_t *dev)
{
    (void)dev;
    return false;
}

#endif // !USE_DMA
