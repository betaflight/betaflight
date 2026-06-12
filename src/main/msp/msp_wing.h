/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/streambuf.h"
#include "flight/pid.h"

#ifdef USE_WING

// MSP2_WING_TUNING: serialize 26 wing config fields from profile.
// Wire format: see .plan/WIRE_FORMAT.md (39 bytes, little-endian).
void serializeWingTuning(sbuf_t *dst, const pidProfile_t *profile);

// MSP2_SET_WING_TUNING: deserialize into profile. Uses sbufBytesRemaining()
// guards; returns true if all fields present, false if short. Does NOT
// write EEPROM -- caller handles persistence via MSP_EEPROM_WRITE.
bool deserializeWingTuning(sbuf_t *src, pidProfile_t *profile);

#endif // USE_WING
