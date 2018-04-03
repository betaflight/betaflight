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

void pidAudioUpdate(void);
void pidAudioInit(void);

typedef enum {
    PID_AUDIO_OFF = 0,
    PID_AUDIO_PIDSUM_X,
    PID_AUDIO_PIDSUM_Y,
    PID_AUDIO_PIDSUM_XY,
} pidAudioModes_e;
pidAudioModes_e pidAudioGetMode(void);
void pidAudioSetMode(pidAudioModes_e mode);

