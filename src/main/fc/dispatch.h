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

struct dispatchEntry_s;
typedef void dispatchFunc(struct dispatchEntry_s* self);

typedef struct dispatchEntry_s {
    dispatchFunc *dispatch;
    uint32_t delayedUntil;
    struct dispatchEntry_s *next;
    bool inQue;
} dispatchEntry_t;

bool dispatchIsEnabled(void);
void dispatchEnable(void);
void dispatchProcess(uint32_t currentTime);
void dispatchAdd(dispatchEntry_t *entry, int delayUs);
