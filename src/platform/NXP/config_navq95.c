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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/serial.h"

#include "pg/rx.h"

#include "io/serial.h"
#include "rx/rx.h"

void targetConfiguration(void)
{
    featureConfigSet(FEATURE_RX_SERIAL);
    featureEnableImmediate(FEATURE_RX_SERIAL);

    rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
    // The SBUS line is inverted; sbus.c applies the inversion when this is off.
    rxConfigMutable()->serialrx_inverted = 0;
    rxConfigMutable()->rx_uart = SERIAL_PORT_UART5;
}
