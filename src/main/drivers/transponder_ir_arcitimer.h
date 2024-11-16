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

#include "drivers/transponder_ir.h"

// aRCiTimer transponder codes:
//
// ID1 0x1F, 0xFC, 0x8F, 0x3, 0xF0, 0x1, 0xF8, 0x1F, 0x0           // E00370FC0FFE07E0FF
// ID2 0xFF, 0x83, 0xFF, 0xC1, 0x7, 0xFF, 0x3, 0xF0, 0x1           // 007C003EF800FC0FFE
// ID3 0x7, 0x7E, 0xE0, 0x7, 0x7E, 0xE0, 0x0, 0x38, 0x0            // F8811FF8811FFFC7FF
// ID4 0xFF, 0x83, 0xFF, 0xC1, 0x7, 0xE0, 0x7F, 0xF0, 0x1          // 007C003EF81F800FFE
// ID5 0xF, 0xF0, 0x0, 0xFF, 0x0, 0xF, 0xF0, 0xF, 0x0              // F00FFF00FFF00FF0FF
// ID6 0xFF, 0x83, 0xF, 0x3E, 0xF8, 0xE0, 0x83, 0xFF, 0xF          // 007CF0C1071F7C00F0
// ID7 0x1F, 0xFC, 0xF, 0xC0, 0xFF, 0x0, 0xFC, 0xF, 0x3E           // E003F03F00FF03F0C1
// ID8 0xFF, 0x3, 0xF0, 0x1, 0xF8, 0xE0, 0xC1, 0xFF, 0x1           // 00FC0FFE071F3E00FE
// ID9 0x1F, 0x7C, 0x40, 0xF, 0xF0, 0x61, 0xC7, 0x3F, 0x0          // E083BFF00F9E38C0FF

void transponderIrInitArcitimer(transponder_t *transponder);
void updateTransponderDMABufferArcitimer(transponder_t *transponder, const uint8_t* transponderData);
