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

// Wing-specific MSP2 message serialization.
//
// Wire contract (both messages, little-endian, append-only): see
// .plan/WIRE_FORMAT.md. A shared golden hex vector lives in
// .plan/GOLDEN_VECTOR.md and is round-tripped by the firmware unit
// test (src/test/unit/wing_msp_unittest.cc) AND the configurator
// vitest. If a field is reordered or its sign flipped in either repo,
// both tests fail in sync.
//
// Signed fields use the bit-pattern cast convention (uint write, int
// read) -- matches MSP_SET_PID_ADVANCED convention, since streambuf
// exposes only U8/U16/U32.

#include "platform.h"

#ifdef USE_WING

#include "msp/msp_wing.h"

// 39 bytes. Signed fields: angle_pitch_offset, tpa_speed_pitch_offset,
// tpa_curve_expo.
void serializeWingTuning(sbuf_t *dst, const pidProfile_t *profile)
{
    sbufWriteU8(dst, profile->pid[PID_ROLL].S);
    sbufWriteU8(dst, profile->pid[PID_PITCH].S);
    sbufWriteU8(dst, profile->pid[PID_YAW].S);
    sbufWriteU8(dst, profile->yaw_type);
    sbufWriteU16(dst, (uint16_t)profile->angle_pitch_offset);
    sbufWriteU8(dst, profile->angle_earth_ref);
    sbufWriteU8(dst, profile->tpa_mode);
    sbufWriteU8(dst, profile->tpa_speed_type);
    sbufWriteU16(dst, profile->tpa_speed_basic_delay);
    sbufWriteU16(dst, profile->tpa_speed_basic_gravity);
    sbufWriteU16(dst, profile->tpa_speed_max_voltage);
    sbufWriteU16(dst, (uint16_t)profile->tpa_speed_pitch_offset);
    sbufWriteU8(dst, profile->tpa_curve_type);
    sbufWriteU8(dst, profile->tpa_curve_stall_throttle);
    sbufWriteU16(dst, profile->tpa_curve_pid_thr0);
    sbufWriteU16(dst, profile->tpa_curve_pid_thr100);
    sbufWriteU8(dst, (uint8_t)profile->tpa_curve_expo);
    sbufWriteU16(dst, profile->spa_center[FD_ROLL]);
    sbufWriteU16(dst, profile->spa_width[FD_ROLL]);
    sbufWriteU8(dst, profile->spa_mode[FD_ROLL]);
    sbufWriteU16(dst, profile->spa_center[FD_PITCH]);
    sbufWriteU16(dst, profile->spa_width[FD_PITCH]);
    sbufWriteU8(dst, profile->spa_mode[FD_PITCH]);
    sbufWriteU16(dst, profile->spa_center[FD_YAW]);
    sbufWriteU16(dst, profile->spa_width[FD_YAW]);
    sbufWriteU8(dst, profile->spa_mode[FD_YAW]);
}

// V1 payload is fixed at 39 bytes. Fields appended in a future minor API
// version must add their own sbufBytesRemaining() >= N guard before reading,
// per the append-only contract. If this guard fails on a truncated V1
// payload, the profile is left untouched; caller sees false return.
bool deserializeWingTuning(sbuf_t *src, pidProfile_t *profile)
{
    if (sbufBytesRemaining(src) < 39) {
        return false;
    }
    profile->pid[PID_ROLL].S          = sbufReadU8(src);
    profile->pid[PID_PITCH].S         = sbufReadU8(src);
    profile->pid[PID_YAW].S           = sbufReadU8(src);
    profile->yaw_type                 = sbufReadU8(src);
    profile->angle_pitch_offset       = (int16_t)sbufReadU16(src);
    profile->angle_earth_ref          = sbufReadU8(src);
    profile->tpa_mode                 = sbufReadU8(src);
    profile->tpa_speed_type           = sbufReadU8(src);
    profile->tpa_speed_basic_delay    = sbufReadU16(src);
    profile->tpa_speed_basic_gravity  = sbufReadU16(src);
    profile->tpa_speed_max_voltage    = sbufReadU16(src);
    profile->tpa_speed_pitch_offset   = (int16_t)sbufReadU16(src);
    profile->tpa_curve_type           = sbufReadU8(src);
    profile->tpa_curve_stall_throttle = sbufReadU8(src);
    profile->tpa_curve_pid_thr0       = sbufReadU16(src);
    profile->tpa_curve_pid_thr100     = sbufReadU16(src);
    profile->tpa_curve_expo           = (int8_t)sbufReadU8(src);
    profile->spa_center[FD_ROLL]      = sbufReadU16(src);
    profile->spa_width[FD_ROLL]       = sbufReadU16(src);
    profile->spa_mode[FD_ROLL]        = sbufReadU8(src);
    profile->spa_center[FD_PITCH]     = sbufReadU16(src);
    profile->spa_width[FD_PITCH]      = sbufReadU16(src);
    profile->spa_mode[FD_PITCH]       = sbufReadU8(src);
    profile->spa_center[FD_YAW]       = sbufReadU16(src);
    profile->spa_width[FD_YAW]        = sbufReadU16(src);
    profile->spa_mode[FD_YAW]         = sbufReadU8(src);
    return true;
}

#endif // USE_WING
