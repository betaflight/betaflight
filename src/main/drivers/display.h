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

struct displayPortVTable_s;
typedef struct displayPort_s {
    const struct displayPortVTable_s *vTable;
    uint8_t rows;
    uint8_t cols;

    // CMS state
    bool cleared;
    int8_t cursorRow;
    bool isGrabbed;
} displayPort_t;

typedef struct displayPortVTable_s {
    int (*grab)(displayPort_t *displayPort);
    int (*release)(displayPort_t *displayPort);
    int (*clear)(displayPort_t *displayPort);
    int (*write)(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *text);
    int (*heartbeat)(displayPort_t *displayPort);
    void (*resync)(displayPort_t *displayPort);
    uint32_t (*txBytesFree)(const displayPort_t *displayPort);
} displayPortVTable_t;

void displayGrab(displayPort_t *instance);
void displayRelease(displayPort_t *instance);
bool displayIsGrabbed(const displayPort_t *instance);
void displayClear(displayPort_t *instance);
int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, const char *s);
void displayHeartbeat(displayPort_t *instance);
void displayResync(displayPort_t *instance);
uint16_t displayTxBytesFree(const displayPort_t *instance);
