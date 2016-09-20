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

typedef enum {
    // NOTE: only add new ones to the end of this list (before DEBUG_MODE_COUNT), otherwise a PG version bump is required.
    DEBUG_NONE,
    DEBUG_CYCLETIME,
    DEBUG_NOTCH,
    DEBUG_GYRO,
    DEBUG_PIDLOOP,

    DEBUG_MODE_COUNT
} debugMode_e;

typedef struct debugConfig_s {
    uint8_t debug_mode;
} debugConfig_t;

PG_DECLARE(debugConfig_t, debugConfig);
