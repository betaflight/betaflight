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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "io/serial.h"
#include "io/serial_feature_map.h"

// Skeleton: returns a synthesized view of which function bits would be
// set on the given port by looking at each feature's PG.  Individual
// feature PGs are wired in subsequent commits; until then the legacy
// serialConfig functionMask remains the source of truth.

uint32_t serialSynthesizeFunctionMask(serialPortIdentifier_e identifier)
{
    (void)identifier;
    return 0;
}

bool serialApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask)
{
    (void)identifier;
    (void)mask;
    return true;
}
