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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Military Grid Reference System (MGRS) — forward encoder.
 *
 * Produces an MGRS reference string from a WGS84 lat/lon pair so the OSD
 * can display a single, easy-to-radio-back grid for downed-aircraft
 * recovery. MGRS is friendlier than decimal degrees in the field because
 * it is short, line-of-sight chunked (100 km grid -> 1 km square ->
 * 10 m / 1 m offset), and tied directly to printed military maps.
 *
 * Forward only: a flight controller emits an MGRS string, it never
 * parses one. Decode/inverse paths from upstream proj4js/mgrs are
 * deliberately omitted to save flash.
 *
 * Math is a port of proj4js/mgrs (MIT licensed; see LICENSE in this
 * directory). The port differs from upstream as follows:
 *   - single-precision floats only (Cortex-M FPU is single-precision);
 *   - integer 1e-7 deg inputs (matches gpsLocation_t in io/gps.h);
 *   - no exceptions / allocations / errno: a single bool return signals
 *     the only failure mode (latitude outside MGRS coverage, ie polar);
 *   - output string is ready for OSD: zone is zero-padded to 2 digits
 *     and the easting / northing groups are space-separated.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Maximum supported precision in MGRS digits per axis. 5 = 1 m. The MGRS
 * spec also defines a 6 (0.1 m) but consumer GPS modules cannot resolve
 * to that level so it is intentionally not supported here.
 */
#define MGRS_PRECISION_MAX 5

/*
 * Buffer size needed for the longest MGRS output, ie at MGRS_PRECISION_MAX:
 *
 *   "NN Z SS DDDDD DDDDD\0"
 *    └┬┘ │ └┬┘  └─┬─┘ └─┬─┘ NUL
 *  zone  │  100k square     5 east   5 north
 *        │
 *        latitude band letter
 *
 * 2 (zone) + 1 (band) + 2 (100k) + 1 (sp) + 5 (east) + 1 (sp) + 5 (north) + 1 (NUL) = 18
 *
 * Lower precision values produce shorter strings; callers using a fixed
 * MGRS_PRECISION_MAX precision can size the buffer with this constant.
 */
#define MGRS_BUFFER_SIZE 18

/*
 * Encode a WGS84 lat/lon to an MGRS reference string.
 *
 * @param lat_e7    Latitude  scaled by 1e7 (matches gpsLocation_t.lat).
 * @param lon_e7    Longitude scaled by 1e7 (matches gpsLocation_t.lon).
 * @param precision Digits per axis, 0..MGRS_PRECISION_MAX.
 *                    5 = 1 m, 4 = 10 m, 3 = 100 m, 2 = 1 km,
 *                    1 = 10 km, 0 = 100 km (grid square only, no
 *                    easting/northing groups in the output).
 *                  Values above MGRS_PRECISION_MAX are clamped.
 * @param out       Caller-supplied output buffer. Must hold at least
 *                  MGRS_BUFFER_SIZE bytes for the worst case. Receives a
 *                  NUL-terminated string on success, and a fixed
 *                  placeholder ("--MGRS---" style) on failure.
 *
 * @return  true on success;
 *          false if the latitude is outside the MGRS coverage band
 *          (below 80S or above 84N — the polar UPS regions, which are
 *          intentionally not supported on the flight controller).
 */
bool mgrsEncode(int32_t lat_e7, int32_t lon_e7, uint8_t precision, char *out);
