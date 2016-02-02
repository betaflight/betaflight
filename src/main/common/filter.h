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

#pragma once

typedef struct filterStatePt1_s {
	float state;
	float RC;
	float constdT;
} filterStatePt1_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquad_s {
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
} biquad_t;

float filterApplyPt1(float input, filterStatePt1_t *filter, float f_cut, float dt);
void filterResetPt1(filterStatePt1_t *filter, float input);

void filterInitBiQuad(uint8_t filterCutFreq, biquad_t *newState, int16_t samplingRate);
float filterApplyBiQuad(float sample, biquad_t *state);
