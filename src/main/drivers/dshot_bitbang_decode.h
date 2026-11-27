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

#include "platform.h"

#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)

#include "common/time.h"

// Decode one motor's answer from the port input buffer. 'now' is the time of
// the capture, read once per port group by the caller; the decoder uses it to
// re-check its preamble skip every MARGIN_CHECK_INTERVAL_US.
#ifdef USE_DSHOT_BITBAND
uint32_t decode_bb_bitband( uint16_t buffer[], uint32_t count, uint32_t bit, timeUs_t now);
#else
uint32_t decode_bb(uint16_t buffer[], uint32_t count, uint32_t bit, timeUs_t now);
#endif

#endif
