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

#include <stdint.h>

#include "pg/pg.h"

typedef struct rpmFilterConfig_s
{
    uint8_t  rpm_filter_harmonics;     // how many harmonics should be covered with notches? 0 means filter off
    uint8_t  rpm_filter_min_hz;        // minimum frequency of the notches
    uint16_t rpm_filter_fade_range_hz; // range in which to gradually turn off notches down to minHz
    uint16_t rpm_filter_q;             // q of the notches

    uint16_t rpm_filter_lpf_hz;        // the cutoff of the lpf on reported motor rpm

} rpmFilterConfig_t;

PG_DECLARE(rpmFilterConfig_t, rpmFilterConfig);
