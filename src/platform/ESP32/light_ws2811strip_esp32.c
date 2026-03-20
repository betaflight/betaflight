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

#ifdef USE_LED_STRIP

#include "common/color.h"
#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/light_ws2811strip.h"

// Stub: WS2812 LED strip via RMT peripheral
// TODO: implement using rmt_new_tx_channel()

bool ws2811LedStripHardwareInit(void)
{
    return false;
}

void ws2811LedStripDMAEnable(void)
{
    // NOOP
}

#endif // USE_LED_STRIP
