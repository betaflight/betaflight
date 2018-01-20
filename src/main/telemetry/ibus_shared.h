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

/*
 * The ibus_shared module implements the ibus telemetry packet handling
 * which is shared between the ibus serial rx and the ibus telemetry.
 *
 * There is only one 'user' active at any time, serial rx will open the
 * serial port if both functions are enabled at the same time
 */

#pragma once

#include "platform.h"
#include "drivers/serial.h"

#define IBUS_CHECKSUM_SIZE (2)
#define IBUS_SENSOR_COUNT 15
#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)

uint8_t respondToIbusRequest(uint8_t const * const ibusPacket);
void initSharedIbusTelemetry(serialPort_t * port);

#endif //defined(TELEMETRY) && defined(TELEMETRY_IBUS)


bool isChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length);
