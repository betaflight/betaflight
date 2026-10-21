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
 * NavQ95 board default configuration (targetConfiguration).
 *
 * Config storage is CONFIG_IN_RAM (volatile), so the SBUS receiver is set up
 * here as a firmware default rather than via CLI + save. targetConfiguration()
 * is called from resetConfig() on every boot. RC/SBUS is on LPUART2 (Betaflight
 * UART2): we enable FEATURE_RX_SERIAL and select the SBUS provider (inverted).
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#include "config/config.h"
#include "config/feature.h"

#include "pg/rx.h"

#include "rx/rx.h"

void targetConfiguration(void)
{
    /* SBUS needs FEATURE_RX_SERIAL. featureConfigSet() writes the config read at
     * boot; featureEnableImmediate() sets the live mask too. */
    featureConfigSet(FEATURE_RX_SERIAL);
    featureEnableImmediate(FEATURE_RX_SERIAL);

    rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
    rxConfigMutable()->serialrx_inverted = 1;   /* SBUS idles high / is inverted */

    /*
     * The UART2 -> FUNCTION_RX_SERIAL port mapping is not set here (the
     * serialFindPortConfigurationMutable finder / serialPortConfig_t layout
     * differs across Betaflight versions). Assign it over the CLI instead:
     *     serial 1 64 115200 57600 0 115200   (UART2 -> FUNCTION_RX_SERIAL)
     *     set serialrx_provider = SBUS
     *     set serialrx_inverted = ON
     */
}
