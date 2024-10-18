/*
 * This file is part of Betaflight and INAV
 *
 * Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "sensors/rangefinder.h"
#include "drivers/rangefinder/rangefinder_virtual.h"

extern virtualRangefinderVTable_t rangefinderMSPVtable;
extern virtualRangefinderVTable_t rangefinderBenewakeVtable;
extern virtualRangefinderVTable_t rangefinderUSD1Vtable;
extern virtualRangefinderVTable_t rangefinderNanoradarVtable; //NRA15/NRA24
extern virtualRangefinderVTable_t rangefinderFakeVtable;

void mspRangefinderReceiveNewData(uint8_t * bufferPtr);
void fakeRangefindersSetData(int32_t data);
