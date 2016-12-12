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

#include <stdint.h>

typedef enum {
    RANGEFINDER_NONE    = 0,
    RANGEFINDER_HCSR04  = 1,
    RANGEFINDER_SRF10   = 2,
} rangefinderType_e;

struct rangefinder_s;
typedef void (*rangefinderInitFunctionPtr)(struct rangefinder_s *rangefinderRange);
typedef void (*rangefinderUpdateFunctionPtr)(void);
typedef int32_t (*rangefinderReadFunctionPtr)(void);

typedef struct rangefinderFunctionPointers_s {
    rangefinderInitFunctionPtr init;
    rangefinderUpdateFunctionPtr update;
    rangefinderReadFunctionPtr read;
} rangefinderFunctionPointers_t;

rangefinderType_e rangefinderDetect(void);
int32_t rangefinderCalculateAltitude(int32_t rangefinderDistance, float cosTiltAngle);
int32_t rangefinderGetLatestAltitude(void);
rangefinderType_e rangefinderDetect(void);
void rangefinderInit(rangefinderType_e rangefinderType);
void rangefinderUpdate(void);
int32_t rangefinderRead(void);
bool isRangefinderHealthy(void);
