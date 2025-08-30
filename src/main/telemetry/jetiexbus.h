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

#pragma once

#define JETI_EXBUS_TELEMETRY_FRAME_LEN 128

// Maximum length of the EX telemetry message payload (bytes)
// Used to derive the maximum EXBUS request buffer size on the producer side.
#define EXTEL_MAX_LEN 26

// Producer-side buffer size for incoming EXBUS request frames (bytes).
// EXBUS_OVERHEAD is 8 bytes (6 header + 2 CRC), so 8 + EXTEL_MAX_LEN = 34.
// Keep this as the single source of truth; consumers should size their frames
// to be >= EXBUS_MAX_REQUEST_BUFFER_SIZE.
#define EXBUS_MAX_REQUEST_BUFFER_SIZE   (8 + EXTEL_MAX_LEN)

void initJetiExBusTelemetry(void);
void checkJetiExBusTelemetryState(void);
void handleJetiExBusTelemetry(void);
