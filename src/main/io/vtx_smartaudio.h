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

/* Created by jflyper */

#pragma once

typedef struct smartAudioConfig_s {
    uint16_t autobaud_min;
    uint16_t autobaud_max;
} smartAudioConfig_t;

#define SMARTAUDIO_AUTOBAUD_DEFAULT_MIN 4800
#define SMARTAUDIO_AUTOBAUD_DEFAULT_MAX 4950

bool smartAudioInit(smartAudioConfig_t *configToUse);
void smartAudioProcess(uint32_t);
