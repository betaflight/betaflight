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

void audioSetupIO(void);
void audioGenerateWhiteNoise(void);

#define TONE_COUNT 64
#define TONE_MIN 0
#define TONE_MAX (TONE_COUNT - 1)
#define TONE_MID ((TONE_COUNT / 2) + TONE_MIN)
void audioPlayTone(uint8_t tone); // TONE_MIN to TONE_MAX

void audioSilence(void);
