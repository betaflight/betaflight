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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "display.h"

void displayClear(displayPort_t *instance)
{
    instance->vTable->clear(instance);
    instance->cleared = true;
    instance->cursorRow = -1;
}

void displayGrab(displayPort_t *instance)
{
    instance->vTable->grab(instance);
    instance->vTable->clear(instance);
    instance->isGrabbed = true;
}

void displayRelease(displayPort_t *instance)
{
    instance->vTable->release(instance);
    instance->isGrabbed = false;
}

bool displayIsGrabbed(const displayPort_t *instance)
{
    // can be called before initialised
    return (instance && instance->isGrabbed);
}

int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, const char *s)
{
    return instance->vTable->write(instance, x, y, s);
}

void displayHeartbeat(displayPort_t *instance)
{
    instance->vTable->heartbeat(instance);
}

void displayResync(displayPort_t *instance)
{
    instance->vTable->resync(instance);
}

uint16_t displayTxBytesFree(const displayPort_t *instance)
{
    return instance->vTable->txBytesFree(instance);
}

