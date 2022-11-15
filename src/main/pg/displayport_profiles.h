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

#include "io/displayport_msp.h"
#include "pg/pg.h"

typedef struct displayPortProfile_s {
    int8_t colAdjust;
    int8_t rowAdjust;
    bool invert;
    uint8_t blackBrightness;
    uint8_t whiteBrightness;

    // For attribute-rich OSDs

    uint8_t fontSelection[DISPLAYPORT_ATTR_COUNT];
    uint8_t useDeviceBlink;    // Use device local blink capability
} displayPortProfile_t;

PG_DECLARE(displayPortProfile_t, displayPortProfileMsp);

PG_DECLARE(displayPortProfile_t, displayPortProfileMax7456);
