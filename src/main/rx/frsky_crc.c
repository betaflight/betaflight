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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "rx/frsky_crc.h"

void frskyCheckSumStep(uint16_t *checksum, uint8_t byte)
{
    *checksum += byte;
}

void frskyCheckSumFini(uint16_t *checksum)
{
    while (*checksum > 0xFF) {
        *checksum = (*checksum & 0xFF) + (*checksum >> 8);
    }

    *checksum = 0xFF - *checksum;
}

static uint8_t frskyCheckSum(uint8_t *data, uint8_t length)
{
    uint16_t checksum = 0;
    for (unsigned i = 0; i < length; i++) {
        frskyCheckSumStep(&checksum, *data++);
    }
    frskyCheckSumFini(&checksum);
    return checksum;
}

bool frskyCheckSumIsGood(uint8_t *data, uint8_t length)
{
    return !frskyCheckSum(data, length);
}
