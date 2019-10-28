/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#include "platform.h"

#include "common/uvarint.h"

int uvarintEncode(uint32_t val, uint8_t *ptr, size_t size)
{
    unsigned ii = 0;
    while (val > 0x80) {
        if (ii >= size) {
            return -1;
        }
        ptr[ii] = (val & 0xFF) | 0x80;
        val >>= 7;
        ii++;
    }
    if (ii >= size) {
        return -1;
    }
    ptr[ii] = val & 0xFF;
    return ii + 1;
}

int uvarintDecode(uint32_t *val, const uint8_t *ptr, size_t size)
{
    unsigned s = 0;
    *val = 0;
    for (size_t ii = 0; ii < size; ii++) {
        uint8_t b = ptr[ii];
        if (b < 0x80) {
            if (ii > 5 || (ii == 5 && b > 1)) {
                // uint32_t overflow
                return -2;
            }
            *val |= ((uint32_t)b) << s;
            return ii + 1;
        }
        *val |= ((uint32_t)(b & 0x7f)) << s;
        s += 7;
    }
    // no value could be decoded and we have no data left
    return -1;
}
