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

#include "pg/pg.h"
#include "drivers/io.h"

typedef struct sdcardConfig_s {
    uint8_t useDma;
    uint8_t enabled;
    uint8_t device;
    ioTag_t cardDetectTag;
    ioTag_t chipSelectTag;
    uint8_t cardDetectInverted;
    uint8_t dmaIdentifier;
    uint8_t dmaChannel;
} sdcardConfig_t;

PG_DECLARE(sdcardConfig_t, sdcardConfig);
