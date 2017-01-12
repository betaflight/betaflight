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

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"
#include "build/debug.h"

#if defined(VTX_TRAMP)
#include "drivers/vtx_var.h"

bool vtx58_Freq2Bandchan(uint16_t freq, uint8_t *pBand, uint8_t *pChan)
{
    uint8_t band;
    uint8_t chan;

    for (band = 0 ; band < 5 ; band++) {
        for (chan = 0 ; chan < 8 ; chan++) {
            if (vtx58FreqTable[band][chan] == freq) {
                *pBand = band + 1;
                *pChan = chan + 1;
                return true;
            }
        }
    }

    *pBand = 0;
    *pChan = 0;

    return false;
}

#endif
